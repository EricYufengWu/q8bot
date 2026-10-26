"""Stand-up and lie-down for the biomimetic (SQuRo) legs.

Unpowered, a SQuRo robot cannot lie on its belly: with the feet under the hips
the front elbows reach the floor at about 27.7 mm below the servo axes, just
below the 26.1 mm belly plate, so it comes to rest on its feet and elbows.
Standing up therefore starts from wherever the legs are, as measured by
telemetry, and moves every foot along a straight line to its CAD stance, so the
body rises as the feet slide under it. Lying down reverses that to a crouch with
the belly about 8 mm off the floor.

Before anything moves the whole path is checked, per leg and per waypoint:
inverse kinematics stays on the leg's own branch (elbow behind on the front,
knee forward on the hind), no joint steps more than 12 deg between waypoints,
no linkage approaches a change point, and no elbow or knee is pushed further
into the floor than it started. Servo output is capped while the body is being
raised or lowered, so a jammed leg meets limited force.
"""

import time
from collections import deque

from leg_detection import near_change_point

STAND_PWM = 530            # Goal PWM while rising or lowering, ~60 % duty
FULL_PWM = 885
STANCE_DEPTH = 64.213      # ground below the servo axes when standing (CAD)
CROUCH_DEPTH = 34.0        # ground below the servo axes when crouched
STEPS = 24
STEP_MS = 100
MAX_STEP_DEG = 12.0
MIN_CLEARANCE = 2.0        # mm between elbow/knee and the floor
START_TOLERANCE = 5.0      # deg between a measured joint and its IK solution
REPLY_TIMEOUT = 1.5


def model_angles(design, present_deg):
    """Readings with whole turns removed, so each sits near its mounting angle."""
    mounts = design.mount_positions()
    return [p - 360.0 * round((p - m) / 360.0) for p, m in zip(present_deg, mounts)]


def _clearance(solver, q1, q2):
    """Floor (at this leg's own contact depth) minus the elbow/knee depth, mm."""
    pivots = solver.pivots(q1, q2)
    return pivots['F'][1] + solver.contact_radius - pivots['E'][1]


def plan_motion(design, start, depth):
    """Waypoints moving every foot straight to its stance x at ``depth``.

    Returns (waypoints, None) or (None, reason). ``start`` is the 8 joint
    angles in model terms (see model_angles).
    """
    solvers = design.leg_solvers()
    feet, targets = [], []
    for leg, solver in enumerate(solvers):
        q1, q2 = start[leg * 2], start[leg * 2 + 1]
        foot = solver.pivots(q1, q2)['F']
        a1, a2, ok = solver.ik_solve(*foot)
        if not ok or abs(a1 - q1) > START_TOLERANCE or abs(a2 - q2) > START_TOLERANCE:
            return None, f'{("FL", "FR", "BL", "BR")[leg]} is folded the other way; set it by hand'
        feet.append(foot)
        targets.append((solver.spec.stance[0], depth - solver.contact_radius))
    previous = list(start)
    first_clearance = [_clearance(s, start[i * 2], start[i * 2 + 1]) for i, s in enumerate(solvers)]
    waypoints = []
    for step in range(1, STEPS + 1):
        t = step / STEPS
        positions = []
        for leg, solver in enumerate(solvers):
            (x0, y0), (x1, y1) = feet[leg], targets[leg]
            q1, q2, ok = solver.ik_solve(x0 + (x1 - x0) * t, y0 + (y1 - y0) * t)
            name = ('FL', 'FR', 'BL', 'BR')[leg]
            if not ok:
                return None, f'{name} cannot reach its path'
            q1, q2 = float(q1), float(q2)
            if (abs(q1 - previous[leg * 2]) > MAX_STEP_DEG or
                    abs(q2 - previous[leg * 2 + 1]) > MAX_STEP_DEG):
                return None, f'{name} would jump between waypoints'
            if near_change_point(solver, q1, q2):
                return None, f'{name} would pass a linkage change point'
            if _clearance(solver, q1, q2) < min(first_clearance[leg], MIN_CLEARANCE) - 1.0:
                return None, f'{name} would push its elbow or knee into the floor'
            positions += [q1, q2]
        previous = positions
        waypoints.append(positions)
    return waypoints, None


class StandUp:
    """Non-blocking stand-up / lie-down, driven like the other static poses."""

    POSE_NAME = 'STANDING'

    def __init__(self, design, robot, clock=time.monotonic):
        self.design = design
        self.robot = robot
        self.clock = clock
        self.stage = 'released'
        self.error = None
        self.active = False            # True while standing (picture follows)
        self._steps = deque()
        self._until = 0.0
        self._deadline = 0.0
        self._commanded = None
        self._start = None
        self._after_cap = None

    # --- control ---------------------------------------------------------
    def toggle(self):
        """T: stand up from rest, or lie down from standing."""
        if self.stage in ('released', 'crouched', 'failed'):
            self.error = None
            self.robot.request_telemetry()
            self.stage = 'measuring'
            self._deadline = self.clock() + REPLY_TIMEOUT
            return True
        if self.stage == 'standing':
            waypoints, error = plan_motion(self.design, self._commanded, CROUCH_DEPTH)
            if error:
                return self._fail(error)
            self._steps = deque(waypoints)
            self._request_cap(STAND_PWM, then='lowering')
            return True
        return False

    def hold(self, positions):
        """Adopt a pose the robot is already holding under torque (after calibration)."""
        self._commanded = list(positions)
        self._steps.clear()
        self.stage = 'standing'
        self.active = True
        self.error = None

    def set_commanded(self, positions):
        """Record the pose last sent by another driver (the trot) while standing."""
        self._commanded = list(positions)

    def feed(self, reply):
        if reply['kind'] == 'telemetry' and self.stage == 'measuring':
            self._begin_rise(reply)
        elif reply['kind'] == 'cap' and self.stage == 'capping':
            if reply['confirmed'] != 0xFF:
                return self._fail('not every leg servo confirmed the output cap')
            if self._after_cap == 'arm':
                self.robot.arm(self._start)
                self._commanded = list(self._start)
                self.stage = 'rising'
            elif self._after_cap == 'lowering':
                self.stage = 'lowering'
            elif self._after_cap == 'standing':
                self.stage = 'standing'
                self.active = True
                return
            self._until = self.clock()

    def update(self):
        now = self.clock()
        if self.stage in ('measuring', 'capping') and now > self._deadline:
            return self._fail('no reply from the robot')
        if self.stage not in ('rising', 'lowering') or now < self._until:
            return None
        if self._steps:
            positions = self._steps.popleft()
            if not self.robot.move_all(positions, STEP_MS, False):
                return self._fail('could not send a waypoint')
            self._commanded = positions
            self._until = now + STEP_MS / 1000.0
            return None
        if self.stage == 'rising':
            # Full output once standing, so the legs can carry the body.
            self._request_cap(FULL_PWM, then='standing')
        else:
            self.stage = 'crouched'
            self.active = False
        return None

    # --- helpers ---------------------------------------------------------
    def _begin_rise(self, telemetry):
        servos = telemetry['servos']
        if not all(servos.get(11 + i, {}).get('responding') for i in range(8)):
            return self._fail('a leg servo does not answer')
        present = [servos[11 + i]['deg'] for i in range(8)]
        start = model_angles(self.design, present)
        waypoints, error = plan_motion(self.design, start, STANCE_DEPTH)
        if error:
            return self._fail(error)
        # Commands follow each servo's own turn, so nothing spins a revolution.
        self.robot.set_turn_offsets(present, start)
        self._start = start
        self._steps = deque(waypoints)
        self._request_cap(STAND_PWM, then='arm')

    def _request_cap(self, pwm, then):
        self.robot.set_output_cap(pwm)
        self.stage = 'capping'
        self._after_cap = then
        self._deadline = self.clock() + REPLY_TIMEOUT

    def _fail(self, reason):
        self.stage = 'failed'
        self.error = reason
        self._steps.clear()
        return reason

    # --- presentation ----------------------------------------------------
    def blocks_movement(self):
        return self.stage not in ('released',)

    def label(self):
        return {
            'released': 'T: Stand up  |  Legs released  |  Esc: Exit',
            'measuring': 'Reading joint angles...',
            'capping': 'Setting servo output...',
            'rising': 'Standing up...  |  Esc: Exit and release torque',
            'standing': 'T: Lie down  |  STANDING  |  Esc: Exit and release torque',
            'lowering': 'Lying down...  |  Esc: Exit and release torque',
            'crouched': 'T: Stand up  |  CROUCHED - holding  |  Esc: Exit and release torque',
            'failed': f'T: Retry  |  {self.error}  |  Esc: Exit',
        }[self.stage]
