"""Trot gaits for the biomimetic (SQuRo) legs: a slow test trot and a fast one.

Both follow the Q8 trot (gait_generator.generate_trot_trajectories): each foot
swings forward on a sine-shaped lift, then pushes back flat along the ground;
FL with BR and FR with BL move together, the two pairs half a cycle apart; and
turning on the spot runs the left and right sides in opposite directions.

They are built per leg: each foot path is centred on that leg's own stance and
turned into joint angles with that leg's own solver. The stride window can be
moved off the CAD stance - front feet further forward, hind feet further back -
which lengthens the support base and, above all, moves the front and hind legs
of each side apart.

A gait can also walk lower than the robot stands. With the legs slightly
more bent, the front leg can reach further forward before its knee
parallelogram straightens towards its change point; at standing height that
straightening, not the hind leg, is what stops the front reach at ~20 mm.

That spacing is what limits a long stride. Front and hind legs of one side swing
in the same plane, half a cycle apart, so a long stride carries the front leg's
hip pushrod and the hind leg's forward-reaching knee pushrod towards each other.
Reaching further forward and back helps only so far: beyond about 16 mm each
leg nears its own parallelogram change point.

Every frame of every direction of every gait must keep, with margin for loose
screws and overshoot:

    MIN_LEG_GAP            18 mm between any front and hind part of one side,
                           centreline to centreline, in side view; the CAD puts
                           some parts in different layers, but play can tilt them
    CHANGE_POINT_MARGIN    25 deg from any parallelogram change point
    MIN_FOLD               40 deg between humerus/femur and ulna
    MIN_CLEARANCE           8 mm between elbow/knee and the floor
    MAX_JOINT_SPEED       800 deg/s, about a third of the XL330-M077's no-load
                           speed, including the wrap back to frame 0

and the inverse kinematics must stay on the leg's own branch. A gait that fails
any of these refuses to load and says why.
"""

import math
import time

from leg_detection import wrap

# Shoulder-crank axis (A) of each leg along the body, nose towards -x (CAD:
# front pair at -50 / -30.5 mm, hind pair at +30.5 / +50 mm).
SHOULDER_AXIS_X = (-30.5, -30.5, 50.0, 50.0)

MIN_LEG_GAP = 18.0
CHANGE_POINT_MARGIN = 25.0
MIN_FOLD = 40.0
MIN_CLEARANCE = 8.0
MAX_JOINT_SPEED = 800.0
STANCE_DEPTH = 64.213      # ground below the servo axes in the CAD stance (mm)

DIRECTIONS = ('f', 'b', 'l', 'r')
# Keyboard and joystick directions these trots do not have fall back, as in
# GaitManager: diagonals to straight ahead or back.
FALLBACK = {'fl': 'f', 'fr': 'f', 'fl_0.75': 'f', 'fl_0.5': 'f', 'fr_0.75': 'f', 'fr_0.5': 'f',
            'bl': 'b', 'br': 'b', 'bl_0.75': 'b', 'bl_0.5': 'b', 'br_0.75': 'b', 'br_0.5': 'b'}
LEGS = ('FL', 'FR', 'BL', 'BR')
SEGMENTS = (('A', 'P1'), ('P1', 'P2'), ('B', 'Q1'), ('Q1', 'Q2'),
            ('B', 'E'), ('E', 'Q2'), ('E', 'F'))


class GaitProfile:
    def __init__(self, key, title, stride, lift, swing, stance, frame_ms, ease_ms,
                 reach_front, reach_hind, depth=STANCE_DEPTH):
        self.key, self.title = key, title
        self.depth = depth                                 # ground below the servo axes (mm)
        self.stride, self.lift = stride, lift              # mm
        self.swing, self.stance = swing, stance            # frames in the air / on the ground
        self.frame_ms, self.ease_ms = frame_ms, ease_ms
        self.reach_front = reach_front                     # mm forward of the front stance
        self.reach_hind = reach_hind                       # mm behind the hind stance

    @property
    def cycle_s(self):
        return (self.swing + self.stance) * self.frame_ms / 1000.0

    @property
    def speed(self):
        """Body speed in mm/s: one stride per stance phase."""
        return self.stride / (self.cycle_s * self.stance / (self.swing + self.stance))


# Slow test trot: short stride, long cycle. The small reach is what keeps its
# front and hind legs 18 mm apart.
SLOW = GaitProfile('TROT', 'slow trot', stride=16.0, lift=8.0, swing=16, stance=32,
                   frame_ms=40, ease_ms=400, reach_front=6.0, reach_hind=6.0)
# Fast trot: 40 mm stride with the feet lifted 20 mm, front feet reaching 20 mm
# further forward and hind feet 20 mm further back, walking 6 mm lower than
# standing so the front legs have that reach. A shorter stance share (16 of 26
# frames, still over half, so both pairs touch down together briefly) and a
# 0.52 s cycle give ~125 mm/s. Kept 1.4 mm above the leg-gap margin and 4 deg
# above the change-point margin rather than at the edge of either.
FAST = GaitProfile('TROT_FAST', 'fast trot', stride=40.0, lift=20.0, swing=10, stance=16,
                   frame_ms=20, ease_ms=500, reach_front=20.0, reach_hind=20.0, depth=58.0)
PROFILES = (SLOW, FAST)


def foot_path(stance, sign, profile=SLOW):
    """One cycle of foot positions; sign +1 walks forward, -1 backward."""
    x0, y0 = stance
    x = x0 - sign * profile.stride / 2
    path = []
    for i in range(profile.swing):
        x += sign * profile.stride / profile.swing
        path.append((x, y0 - math.sin((i + 1) * math.pi / profile.swing) * profile.lift))
    for _ in range(profile.stance):
        x -= sign * profile.stride / profile.stance
        path.append((x, y0))
    return path


def walking_stance(solver, leg, profile):
    """The leg's foot reference while walking: shifted by the gait's reach
    (front forward, hind back) and set for the gait's walking height."""
    x0 = solver.spec.stance[0]
    y0 = profile.depth - solver.contact_radius
    return (x0 + profile.reach_front, y0) if leg < 2 else (x0 - profile.reach_hind, y0)


# --- margins ---------------------------------------------------------------

def _point_segment(p, a, b):
    bx, by = b[0] - a[0], b[1] - a[1]
    length = bx * bx + by * by
    t = 0.0 if length == 0 else max(0.0, min(1.0, ((p[0] - a[0]) * bx + (p[1] - a[1]) * by) / length))
    return math.hypot(p[0] - a[0] - t * bx, p[1] - a[1] - t * by)


def _segment_distance(p, q, r, s):
    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    if cross(r, s, p) * cross(r, s, q) < 0 and cross(p, q, r) * cross(p, q, s) < 0:
        return 0.0
    return min(_point_segment(p, r, s), _point_segment(q, r, s),
               _point_segment(r, p, q), _point_segment(s, p, q))


def _in_body(leg, point):
    return (SHOULDER_AXIS_X[leg] - point[0], point[1])


def leg_gap(front_pivots, hind_pivots, front_pad_radius):
    """Closest approach between a front and a hind leg of one side, in mm."""
    f = {k: _in_body(0, v) for k, v in front_pivots.items()}
    h = {k: _in_body(2, v) for k, v in hind_pivots.items()}
    parts = min(_segment_distance(f[a], f[b], h[c], h[d]) for a, b in SEGMENTS for c, d in SEGMENTS)
    # The front foot is a round pad: its rim, not its centre, can touch.
    pad = min(_point_segment(f['F'], h[c], h[d]) for c, d in SEGMENTS) - front_pad_radius
    return min(parts, pad)


def margins(design, frames, frame_ms, depth=STANCE_DEPTH):
    """The worst value of every margin over a cyclic list of frames.

    ``depth`` is where the ground is (below the servo axes) for this gait.
    """
    solvers = design.leg_solvers()
    worst = {'gap': math.inf, 'change': math.inf, 'fold': math.inf,
             'clearance': math.inf, 'speed': 0.0}
    for index, frame in enumerate(frames):
        pivots = [s.pivots(*frame[i * 2: i * 2 + 2]) for i, s in enumerate(solvers)]
        for front, hind in ((0, 2), (1, 3)):
            worst['gap'] = min(worst['gap'],
                               leg_gap(pivots[front], pivots[hind], solvers[front].contact_radius))
        for leg, solver in enumerate(solvers):
            q1, q2 = frame[leg * 2: leg * 2 + 2]
            p = pivots[leg]
            worst['change'] = min(worst['change'], *[abs(wrap(v)) for v in
                                                      (q2, q2 - 180, q1 - q2, q1 - q2 - 180)])
            to_b = math.atan2(p['B'][1] - p['E'][1], p['B'][0] - p['E'][0])
            to_f = math.atan2(p['F'][1] - p['E'][1], p['F'][0] - p['E'][0])
            worst['fold'] = min(worst['fold'], abs(wrap(math.degrees(to_b - to_f))))
            worst['clearance'] = min(worst['clearance'], depth - p['E'][1])
        following = frames[(index + 1) % len(frames)]
        step = max(abs(b - a) for a, b in zip(frame, following))
        worst['speed'] = max(worst['speed'], step / (frame_ms / 1000.0))
    return worst


def check(design, frames, frame_ms, label, depth=STANCE_DEPTH):
    """Raise ValueError naming the first margin these frames break."""
    m = margins(design, frames, frame_ms, depth)
    if m['gap'] < MIN_LEG_GAP:
        raise ValueError(f'{label}: front and hind legs come within {m["gap"]:.1f} mm')
    if m['change'] < CHANGE_POINT_MARGIN:
        raise ValueError(f'{label}: a linkage comes within {m["change"]:.0f} deg of a change point')
    if m['fold'] < MIN_FOLD:
        raise ValueError(f'{label}: an elbow or knee folds to {m["fold"]:.0f} deg')
    if m['clearance'] < MIN_CLEARANCE:
        raise ValueError(f'{label}: an elbow or knee comes within {m["clearance"]:.1f} mm of the floor')
    if m['speed'] > MAX_JOINT_SPEED:
        raise ValueError(f'{label}: a joint needs {m["speed"]:.0f} deg/s')
    return m


# --- building ----------------------------------------------------------------

def _leg_cycle(solver, leg, sign, profile):
    frames = []
    for x, y in foot_path(walking_stance(solver, leg, profile), sign, profile):
        q1, q2, ok = solver.ik_solve(x, y)
        if not ok:
            raise ValueError(f'{profile.title}: {LEGS[leg]} cannot reach its path')
        frames.append((float(q1), float(q2)))
    return frames


def build_trot(design, profile=SLOW):
    """{direction: [8 joint angles per frame]}; raises ValueError if unsafe."""
    solvers = design.leg_solvers()
    half = (profile.swing + profile.stance) // 2
    cycles = {}
    for leg, solver in enumerate(solvers):
        for sign in (+1, -1):
            frames = _leg_cycle(solver, leg, sign, profile)
            cycles[leg, sign] = frames
            cycles[leg, sign, 'shifted'] = frames[half:] + frames[:half]
    # Per direction, the stride sign of FL, FR, BL, BR; FR and BL run half a
    # cycle behind, exactly as in the Q8 trot.
    signs = {'f': (+1, +1, +1, +1), 'b': (-1, -1, -1, -1),
             'l': (-1, +1, -1, +1), 'r': (+1, -1, +1, -1)}
    trajectories = {}
    for direction, leg_signs in signs.items():
        legs = [cycles[leg, leg_signs[leg]] if leg in (0, 3) else cycles[leg, leg_signs[leg], 'shifted']
                for leg in range(4)]
        frames = [[angle for leg in range(4) for angle in legs[leg][frame]]
                  for frame in range(len(legs[0]))]
        check(design, frames, profile.frame_ms, f'{profile.title} {direction}', profile.depth)
        trajectories[direction] = frames
    return trajectories


class SquroTrot:
    """GaitManager-like driver over the gait profiles, paced by the clock."""

    def __init__(self, design, clock=None, profiles=PROFILES):
        self.design = design
        self.profiles = list(profiles)
        self.trajectories = {p.key: build_trot(design, p) for p in self.profiles}
        self.home = design.mount_positions()
        self.clock = clock or time.monotonic
        self.index = 0
        self.direction = None
        self.frame = 0
        self._next = 0.0
        self._easing = False

    @property
    def profile(self):
        return self.profiles[self.index]

    @property
    def current_gait(self):
        return self.profile.key

    @property
    def walking(self):
        return self.direction is not None

    def switch_gait(self):
        """Next gait; only while stopped, so no frame ever jumps between gaits."""
        if self.walking:
            return False
        self.index = (self.index + 1) % len(self.profiles)
        self.frame = 0
        return True

    def start_movement(self, direction):
        direction = FALLBACK.get(direction, direction)
        if direction not in DIRECTIONS:
            return False
        if self.direction is None:
            # From standing: ease to the first frame instead of jumping to it.
            self._easing = True
            self._next = self.clock()
        self.direction = direction       # the phase carries over on a change
        return True

    def tick(self):
        """(positions, move_ms) when a frame is due, else None."""
        if self.direction is None:
            return None
        now = self.clock()
        if now < self._next:
            return None
        profile = self.profile
        frames = self.trajectories[profile.key][self.direction]
        if self._easing:
            self._easing = False
            self._next = now + profile.ease_ms / 1000.0
            return frames[self.frame], profile.ease_ms
        self.frame = (self.frame + 1) % len(frames)
        self._next = now + profile.frame_ms / 1000.0
        return frames[self.frame], profile.frame_ms

    def stop(self):
        """Back to the standing pose: (positions, move_ms)."""
        self.direction = None
        self.frame = 0
        return list(self.home), self.profile.ease_ms
