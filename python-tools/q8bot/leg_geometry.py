"""One description of a leg's drawable shape, shared by every view.

Both the gait illustrations and the calibration diagram ask for the same thing:
where every pivot sits at a given joint pair, and which pivots are joined by
which kind of link. Keeping that in one place means a new leg design becomes
drawable everywhere at once, and no picture can disagree with the solver that
commands the robot.

Coordinates are leg-local, matching the solvers: x to the right, y positive
downwards (away from the body), motor axes at (0, 0) and (d, 0).
"""

import math

# Link roles, so each view can style them consistently.
FRAME = 'frame'        # The span between the two motor axes.
CRANK = 'crank'        # Driven directly by a servo.
PUSHROD = 'pushrod'    # Transmits crank motion further down the leg.
BONE = 'bone'          # Structural limb segment carrying the foot.


def pivots(solver, q1, q2):
    """Every drawable pivot for this solver at the given joint pair, in degrees."""
    if hasattr(solver, 'pivots'):
        return solver.pivots(q1, q2)
    # Five-bar: two upper links from the motor axes meeting at a shared foot.
    a1, a2 = math.radians(q1), math.radians(q2)
    knee1 = (solver.d + solver.l1 * math.cos(a1), solver.l1 * math.sin(a1))
    knee2 = (solver.l1p * math.cos(a2), solver.l1p * math.sin(a2))
    foot = solver.fk_solve(q1, q2)
    return {'A': (0.0, 0.0), 'B': (float(solver.d), 0.0),
            'K1': knee1, 'K2': knee2, 'F': (float(foot[0]), float(foot[1]))}


def links(pivot_set):
    """(start, end, role) for every link, ordered far-to-near for drawing."""
    if 'E' in pivot_set:      # Crank-and-pushrod leg, e.g. the biomimetic one.
        return [('A', 'B', FRAME),
                ('A', 'P1', CRANK), ('P1', 'P2', PUSHROD),
                ('B', 'Q1', CRANK), ('Q1', 'Q2', PUSHROD),
                ('B', 'E', BONE), ('E', 'F', BONE)]
    return [('A', 'B', FRAME),
            ('B', 'K1', CRANK), ('A', 'K2', CRANK),
            ('K1', 'F', BONE), ('K2', 'F', BONE)]


def motor_pivots():
    """The two pivots that are servo output axes, for labelling."""
    return ('A', 'B')


def foot_pivot():
    return 'F'


def bounds(pivot_set):
    """(min_x, min_y, max_x, max_y) over every pivot."""
    xs = [p[0] for p in pivot_set.values()]
    ys = [p[1] for p in pivot_set.values()]
    return min(xs), min(ys), max(xs), max(ys)
