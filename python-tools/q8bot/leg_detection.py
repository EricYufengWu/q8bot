"""Startup leg detection by gentle limit discovery.

With the robot lifted so the legs hang free, every joint is swept a little at a
time, at low output, until it meets a physical stop; the stop angles are then
matched against stops recorded for each leg design.

Two things keep the sweeps safe:

* The robot firmware (legProbe.h) caps the output, stops a joint as soon as it
  stops following its goal, holds the partner joint still and aborts if the
  partner is pushed out of place.
* This module bounds every sweep before it starts. Before detection the design
  is unknown, so each sweep is checked against *every* candidate: no SQuRo
  parallelogram may come within 15 deg of its change point (where it can fold
  into its crossed form with almost no force), and no five-bar may bring its
  knee pins within 8 mm of each other (where it can flip). A sweep stops at the
  first angle any candidate would object to.

Stop angles depend on the physical parts, so they are recorded per design
on the robot ("learning") rather than computed. Until a design has a
recording, detection can say only that nothing recorded matches.
"""

import json
import math
import os
import time
from pathlib import Path

from espnow import PROBE_STATUS_OK
from leg_designs import LEG_DESIGNS, design_for, selectable

PROBE_PWM = 140              # Goal PWM during sweeps, ~16 % duty
MAX_SWEEP = 120.0            # deg per direction
MIN_SWEEP = 5.0              # a direction with less room is not swept
CHANGE_POINT_MARGIN = 15.0   # deg kept from any parallelogram change point
KNEE_GAP = 8.0               # mm kept between the five-bar knee pins
CONTACT_TOLERANCE = 8.0      # deg; a recorded stop must recur within this
MIN_MATCHES = 4              # agreeing stops needed before naming a design
MAX_RUNS = 5                 # recordings kept per design
LEGS = ('FL', 'FR', 'BL', 'BR')
ROUNDS = (('q1', +1), ('q1', -1), ('q2', +1), ('q2', -1))
HEAD_ID = 19
HEAD_RANGE = 90.0            # deg either side of centre the head is ever swept
HEAD_MARGIN = 8.0            # deg kept inside a measured head contact
HEAD_ROUNDS = (+1, -1)


def state_dir():
    return Path(os.environ.get('Q8BOT_HOME', Path.home() / '.q8bot'))


def wrap(angle):
    """Angle folded into [-180, 180)."""
    return (angle + 180.0) % 360.0 - 180.0


# --- safety bounds -----------------------------------------------------------

def near_change_point(solver, q1, q2):
    """True if this pose is too close to where the linkage could fold or flip."""
    if hasattr(solver, 'branch'):
        # SQuRo: the hip parallelogram degenerates when the shoulder crank lines
        # up with the servo-to-servo line; the knee parallelogram when the elbow
        # crank lines up with the humerus.
        for value in (q2, q2 - 180.0, q1 - q2, q1 - q2 - 180.0):
            if abs(wrap(value)) < CHANGE_POINT_MARGIN:
                return True
        return False
    # Five-bar: its passive chain can only flip where the two knee pins meet.
    a1, a2 = math.radians(q1), math.radians(q2)
    k1 = (solver.d + solver.l1 * math.cos(a1), solver.l1 * math.sin(a1))
    k2 = (solver.l1p * math.cos(a2), solver.l1p * math.sin(a2))
    return math.hypot(k1[0] - k2[0], k1[1] - k2[1]) < KNEE_GAP


def candidates():
    """Designs with a kinematic model, each as its per-leg solvers."""
    return [design for design in selectable() if design.available]


def sweep_bound(leg_solvers, angles, joint, direction):
    """Degrees ``joint`` may travel in ``direction`` before any candidate objects.

    ``leg_solvers`` holds, for this joint's leg, one solver per candidate
    design. The partner joint is assumed held where it is (the firmware aborts
    if it is pushed more than 5 deg, well inside the margins).
    """
    leg = joint // 2
    pose = [angles[leg * 2], angles[leg * 2 + 1]]
    index = joint % 2
    travelled = 0.0
    while travelled < MAX_SWEEP:
        trial = list(pose)
        trial[index] = pose[index] + direction * (travelled + 1.0)
        if any(near_change_point(solver, *trial) for solver in leg_solvers):
            break
        travelled += 1.0
    return travelled


def plan_round(angles, designs, part, direction):
    """Signed excursions for one round: the same joint of every leg, one direction."""
    solvers = [design.leg_solvers() for design in designs]
    excursions, bounds = [0.0] * 8, [0.0] * 8
    for leg in range(4):
        joint = leg * 2 + (0 if part == 'q1' else 1)
        bound = sweep_bound([s[leg] for s in solvers], angles, joint, direction)
        bounds[joint] = bound
        if bound >= MIN_SWEEP:
            excursions[joint] = direction * bound
    return excursions, bounds


# --- telemetry ---------------------------------------------------------------

def telemetry_problems(telemetry):
    """Reasons the legs cannot be probed right now; empty when they can."""
    problems = []
    for servo_id in range(11, 19):
        servo = telemetry['servos'].get(servo_id)
        if not servo or not servo['responding']:
            problems.append(f'ID{servo_id} does not answer')
        elif servo['fault']:
            problems.append(f'ID{servo_id} reports hardware error {servo["error"]}')
        elif servo['torque']:
            problems.append(f'ID{servo_id} has torque on')
    return problems


def joint_angles(telemetry):
    return [telemetry['servos'][11 + i]['deg'] for i in range(8)]


def head_ready(telemetry):
    """True when the head answers, is released and reports no fault."""
    head = telemetry['servos'].get(HEAD_ID)
    return bool(head and head['responding'] and not head['torque'] and not head['fault'])


def head_bound(start, direction):
    """Signed sweep keeping the head inside +-HEAD_RANGE of its centre."""
    room = HEAD_RANGE - start if direction > 0 else HEAD_RANGE + start
    return direction * max(0.0, room)


def head_limits(head):
    """Yaw limits from the two head sweeps: each contact, less HEAD_MARGIN.

    A direction that met nothing is limited to where the sweep ended. Returns
    None if the limits would not leave the centre reachable.
    """
    ends = {}
    for direction in HEAD_ROUNDS:
        entry = head.get(direction)
        if entry is None or entry['kind'] not in ('contact', 'free'):
            return None
        ends[direction] = entry['stop'] - direction * HEAD_MARGIN
    low, high = ends[-1], ends[+1]
    return (low, high) if low <= 0.0 <= high else None


# --- measurements and signatures --------------------------------------------

def key_for(leg, part, direction):
    return f'{LEGS[leg]}.{part}{"+" if direction > 0 else "-"}'


def measurements_from(reply, part, direction, bounds):
    """One probe reply as {key: {kind, start, stop, bound, direction}}."""
    found = {}
    for leg in range(4):
        joint = leg * 2 + (0 if part == 'q1' else 1)
        data = reply['joints'][joint]
        if not data['swept']:
            kind = 'blocked'
        elif data['reason'] in ('contact', 'overcurrent'):
            kind = 'contact'
        elif data['reason'] == 'bound':
            kind = 'free'
        else:
            kind = 'incomplete'
        found[key_for(leg, part, direction)] = {
            'kind': kind, 'start': data['start'], 'stop': data['stop'],
            'bound': bounds[joint], 'direction': direction,
        }
    return found


def _within(angle, start, bound, direction, slack=0.0):
    travelled = wrap(angle - start) * direction
    return -slack <= travelled <= bound + slack


def _circular_mean(angles):
    x = sum(math.cos(math.radians(a)) for a in angles)
    y = sum(math.sin(math.radians(a)) for a in angles)
    return math.degrees(math.atan2(y, x))


def compare(runs, measurement):
    """(matches, mismatches) of one measurement against a design's recordings."""
    matches = mismatches = 0
    for key, now in measurement.items():
        past = [run[key] for run in runs if key in run]
        contacts = [p['stop'] for p in past if p['kind'] == 'contact']
        frees = [p for p in past if p['kind'] == 'free']
        if contacts:
            expected = _circular_mean(contacts)
            spread = max(abs(wrap(c - expected)) for c in contacts)
            tolerance = max(CONTACT_TOLERANCE, 3 * spread)
        if now['kind'] == 'contact':
            if contacts:
                if abs(wrap(now['stop'] - expected)) <= tolerance:
                    matches += 1
                else:
                    mismatches += 1
            elif any(_within(now['stop'], f['start'], f['bound'], f['direction']) for f in frees):
                # Recorded as free over this range, yet something is in the way.
                mismatches += 1
        elif now['kind'] == 'free' and contacts:
            # The recorded stop lay inside today's sweep but was never met.
            if _within(expected, now['start'], now['bound'], now['direction'], -tolerance):
                mismatches += 1
    return matches, mismatches


class SignatureStore:
    """Recorded stop angles per design, kept between sessions."""

    def __init__(self, path=None):
        self.path = Path(path) if path else state_dir() / 'leg_signatures.json'
        try:
            self.data = json.loads(self.path.read_text())
        except (OSError, ValueError):
            self.data = {}

    def runs(self, design_key):
        return self.data.get(design_key, {}).get('runs', [])

    def record(self, design_key, measurement):
        runs = (self.runs(design_key) + [measurement])[-MAX_RUNS:]
        self.data[design_key] = {'runs': runs}
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.path.write_text(json.dumps(self.data, indent=1))
        return len(runs)


def classify(measurement, store, designs=None):
    """Name the fitted design, or explain why not."""
    designs = designs or candidates()
    contacts = sum(1 for m in measurement.values() if m['kind'] == 'contact')
    scores, recorded = {}, []
    for design in designs:
        runs = store.runs(design.key)
        if not runs:
            continue
        recorded.append(design.key)
        scores[design.key] = compare(runs, measurement)
    fits = sorted(((matches, key) for key, (matches, wrong) in scores.items()
                   if wrong == 0 and matches >= MIN_MATCHES), reverse=True)
    if contacts < MIN_MATCHES:
        verdict, reason = None, f'only {contacts} joints met a stop; too few to identify'
    elif not recorded:
        verdict, reason = None, 'no design has a recording yet'
    elif not fits:
        verdict, reason = None, 'no recorded design matches these stops'
    elif len(fits) > 1 and fits[0][0] - fits[1][0] < 2:
        verdict, reason = None, f'{fits[0][1]} and {fits[1][1]} both match'
    else:
        verdict, reason = fits[0][1], f'{fits[0][0]} stops agree, none disagree'
    return {'design': verdict, 'reason': reason, 'scores': scores,
            'recorded': recorded, 'contacts': contacts}


# --- remembered design -------------------------------------------------------

def load_last_design():
    try:
        key = json.loads((state_dir() / 'state.json').read_text()).get('design')
        return key if key in LEG_DESIGNS and design_for(key).available else None
    except (OSError, ValueError):
        return None


def save_last_design(key):
    path = state_dir() / 'state.json'
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps({'design': key}))


# --- startup state machine ---------------------------------------------------

class LegDetector:
    """Non-blocking detection run; the GUI feeds replies and calls update()."""

    TELEMETRY_TIMEOUT = 1.5
    PROBE_TIMEOUT = 20.0

    def __init__(self, robot, store=None, clock=time.monotonic, designs=None):
        self.robot = robot
        self.store = store or SignatureStore()
        self.clock = clock
        self.designs = designs or candidates()
        self.stage = 'idle'
        self.telemetry = None
        self.problems = []
        self.measurement = {}
        self.result = None
        self.error = None
        self.restore_failed = False
        self.round = 0
        self.head = {}               # {+1/-1: {kind, start, stop, bound}}
        self.head_round = None       # index into HEAD_ROUNDS while the head is swept
        self._deadline = 0.0
        self._bounds = None

    @property
    def active(self):
        return self.stage in ('telemetry', 'probing')

    def request_telemetry(self):
        self.robot.request_telemetry()
        self.stage = 'telemetry'
        self.error = None
        self._deadline = self.clock() + self.TELEMETRY_TIMEOUT

    def start(self):
        """Begin the four sweep rounds. Telemetry must be fresh and clean."""
        if self.telemetry is None or self.problems:
            self.error = 'telemetry missing or legs not ready'
            return False
        self.measurement = {}
        self.result = None
        self.head = {}
        self.head_round = None
        self.round = 0
        return self._send_round()

    def abort(self):
        if self.stage == 'probing':
            self.robot.abort_probe()
        self.stage, self.error = 'failed', 'detection cancelled'

    def feed(self, reply):
        if reply['kind'] == 'telemetry' and self.stage == 'telemetry':
            self.telemetry = reply
            self.problems = telemetry_problems(reply)
            self.stage = 'ready'
        elif reply['kind'] == 'probe' and self.stage == 'probing':
            if reply['status'] != PROBE_STATUS_OK:
                self.stage, self.error = 'failed', reply['status_text']
                self.restore_failed = reply['status'] == 11
                return
            if self.head_round is not None:
                return self._head_reply(reply)
            part, direction = ROUNDS[self.round]
            self.measurement.update(measurements_from(reply, part, direction, self._bounds))
            self.round += 1
            self._send_round()

    def update(self):
        if self.active and self.clock() > self._deadline:
            if self.stage == 'telemetry':
                self.stage = 'no_reply'
                self.error = 'the robot did not answer; firmware without detection, or robot off'
            else:
                self.robot.abort_probe()
                self.stage, self.error = 'failed', 'no reply from the robot during a sweep'

    def record(self, design_key):
        """Store the last measurement as a recording of ``design_key``."""
        count = self.store.record(design_key, self.measurement)
        self.result = classify(self.measurement, self.store, self.designs)
        return count

    def _send_round(self):
        # Rounds whose every joint lacks room are skipped, not sent.
        while self.round < len(ROUNDS):
            part, direction = ROUNDS[self.round]
            angles = joint_angles(self.telemetry)
            excursions, self._bounds = plan_round(angles, self.designs, part, direction)
            if any(excursions):
                self.robot.start_probe(excursions, PROBE_PWM)
                self.stage = 'probing'
                self._deadline = self.clock() + self.PROBE_TIMEOUT
                return True
            self.measurement.update(measurements_from(
                {'joints': [{'swept': False, 'reason': 'none', 'start': None, 'stop': None}] * 8},
                part, direction, self._bounds))
            self.round += 1
        # Legs done. The head is swept on its own, if it is there and released;
        # its stops never enter the leg classification.
        self.result = classify(self.measurement, self.store, self.designs)
        if self.head_round is None and head_ready(self.telemetry):
            self.head_round = 0
            return self._send_head_round()
        return self._finish()

    def _send_head_round(self):
        while self.head_round < len(HEAD_ROUNDS):
            direction = HEAD_ROUNDS[self.head_round]
            start = self.telemetry['servos'][HEAD_ID]['deg']
            bound = head_bound(start, direction)
            if abs(bound) >= MIN_SWEEP:
                self._head_bound = bound
                self.robot.start_head_probe(bound, PROBE_PWM)
                self.stage = 'probing'
                self._deadline = self.clock() + self.PROBE_TIMEOUT
                return True
            self.head[direction] = {'kind': 'blocked', 'start': start, 'stop': start, 'bound': 0.0}
            self.head_round += 1
        return self._finish()

    def _head_reply(self, reply):
        direction = HEAD_ROUNDS[self.head_round]
        data = reply['joints'][8] if len(reply['joints']) > 8 else None
        if not data or not data['swept']:
            kind = 'incomplete'
        elif data['reason'] in ('contact', 'overcurrent'):
            kind = 'contact'
        elif data['reason'] == 'bound':
            kind = 'free'
        else:
            kind = 'incomplete'
        self.head[direction] = {'kind': kind, 'start': data and data['start'],
                                'stop': data and data['stop'], 'bound': abs(self._head_bound)}
        self.head_round += 1
        return self._send_head_round()

    def _finish(self):
        self.stage = 'done'
        return True

    def progress(self):
        if self.stage == 'probing' and self.head_round is not None:
            direction = HEAD_ROUNDS[self.head_round]
            return f'Head sweep {self.head_round + 1} of {len(HEAD_ROUNDS)}, {"+" if direction > 0 else "-"} direction'
        if self.stage == 'probing':
            part, direction = ROUNDS[self.round]
            joint = 'elbow (q1)' if part == 'q1' else 'shoulder (q2)'
            return f'Sweep {self.round + 1} of {len(ROUNDS)}: {joint}, {"+" if direction > 0 else "-"} direction'
        return self.stage
