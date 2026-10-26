"""The leg designs the robot can be built with, and their mounting references.

Each entry carries the solver for that linkage and the joint pair the servos
are driven to while the linkages are attached. The mounting pose is what makes
a design swappable: it defines the keying between a servo horn and its crank,
so every later commanded angle means the same thing.

Joint ordering matches ``q8_espnow.move_all``: ``q1`` is the motor at (d, 0)
and ``q2`` the motor at (0, 0), repeated for FL, FR, BL, BR.
"""

from gait_manager import GAITS as Q8_GAITS
from kinematics_solver import k_solver
from routine_generator import G1, G2, G3
from squro_solver import HIND, squro_solver

# Dynamixel ticks per degree; the robot firmware uses 0 deg = 4096 ticks.
TICKS_PER_DEG = 4096.0 / 360.0
TICK_ZERO = 4096


def deg_to_ticks(deg):
    return int(deg * TICKS_PER_DEG + 0.5) + TICK_ZERO


# --- motion profiles ------------------------------------------------------
# A gait table is [stacktype, x0, y0, xrange, yrange, yrange2, s1, s2], where
# x0/y0 are the stance centre in foot space and xrange/yrange the stride and
# lift. Stride, lift and step counts describe locomotion, so they carry across
# designs unchanged; the stance centre does not, because it is where that
# particular linkage naturally hangs.
#
# The biomimetic table is the Q8 table with:
#   x0 -> the design's natural stance x (33.935, from the CAD pose), and
#   y0 -> y0 * (45.113 / 43.36), the ratio of the two natural stance heights,
# then stride and lift trimmed 5% at a time only where the joints would
# otherwise swing more than 40 deg from the mounting pose. Driving this leg
# with the Q8 stance centre instead reaches 219 deg on q2, about 90 deg past
# its as-designed pose, which is why the re-centring is needed at all.
BIOMIMETIC_GAITS = {
    'TROT':      ['trot', 33.935, 45.11, 40.0, 20.0, 0.0, 15, 30],
    'TROT_HIGH': ['trot', 33.935, 62.43, 20.0, 10.0, 0.0, 15, 30],
    'TROT_LOW':  ['trot', 33.935, 26.01, 19.0, 9.5, 0.0, 15, 30],
    'TROT_FAST': ['trot', 33.935, 45.11, 50.0, 20.0, 0.0, 12, 24],
    'WALK':      ['walk', 33.935, 45.11, 30.0, 20.0, 0.0, 20, 140],
    'CRAWL':     ['crawl', 33.935, 41.62, 10.0, 20.0, 0.0, 50, 25],
    'BOUND':     ['bound', 33.935, 34.71, 38.0, 0.0, 19.0, 50, 10],
    'PRONK':     ['pronk', 33.935, 34.71, 38.0, 0.0, 19.0, 60, 10],
}

# Greeting/rearing waypoints are stored as joint angles, not foot positions, so
# they are remapped per design: Q8 joints -> foot xy through the five-bar FK ->
# the new design's joints through its IK. That preserves the foot path, which
# is the part that means anything. Computed once from G1/G2/G3.
BIOMIMETIC_G1 = [92.0, 138.75] * 4
BIOMIMETIC_G2 = [34.24, 104.06] * 4
BIOMIMETIC_G3 = [-0.23, 86.98] * 2 + [73.25, 124.83] * 2


class LegDesign:
    """One buildable leg design."""

    def __init__(self, key, title, subtitle, solver_factory, mount_pose,
                 available=True, source=None, steps=(), notes=(),
                 natural_stance=None, gaits=None, sitting=None, rearing=None,
                 hind_solver_factory=None, hind_mount_pose=None, supports_gaits=True):
        self.key = key
        self.title = title
        self.subtitle = subtitle
        self._solver_factory = solver_factory
        # Front-leg mounting pose; hind legs use their own when the design has one.
        self.mount_pose = mount_pose
        self._hind_solver_factory = hind_solver_factory
        self.hind_mount_pose = hind_mount_pose or mount_pose
        # Gait tables and the sit/rear/greet poses are single-leg-model data.
        # A design whose hind legs differ from its front legs cannot use them
        # until they are rebuilt per leg; the GUI must not offer them.
        self.supports_gaits = supports_gaits
        self.available = available
        self.source = source
        self.steps = tuple(steps)
        self.notes = tuple(notes)
        self.natural_stance = natural_stance
        self.gaits = gaits
        # Foot targets for the static sitting pose: (foot_x, front, rear).
        self.sitting = sitting
        # (G1, G2, G3, rearward_offset) for the greeting and rearing poses.
        self.rearing = rearing

    @property
    def asset_dir(self):
        """Folder holding this design's gait and pose pictures."""
        return self.key.lower()

    def solver(self):
        """The front-leg solver (the only one, for designs without a hind model)."""
        if not self.available or self._solver_factory is None:
            raise ValueError(f'{self.title} has no kinematic model yet')
        return self._solver_factory()

    def hind_solver(self):
        if self._hind_solver_factory is None:
            return self.solver()
        if not self.available:
            raise ValueError(f'{self.title} has no kinematic model yet')
        return self._hind_solver_factory()

    @property
    def per_leg(self):
        """True when the hind legs are a different linkage pose from the front."""
        return self._hind_solver_factory is not None

    def leg_solvers(self):
        """One solver per leg, in wire order FL, FR, BL, BR."""
        front, hind = self.solver(), self.hind_solver()
        return [front, self.solver(), hind, self.hind_solver()]

    def mount_positions(self):
        """Mounting poses for all four legs, FL FR BL BR, for move_all."""
        front = [float(a) for a in self.mount_pose]
        hind = [float(a) for a in self.hind_mount_pose]
        return front * 2 + hind * 2

    def mount_ticks(self):
        return tuple(deg_to_ticks(a) for a in self.mount_pose)

    def hind_mount_ticks(self):
        return tuple(deg_to_ticks(a) for a in self.hind_mount_pose)


# The original five-bar leg. Its mounting pose is the one the motor-config
# firmware already drives to at the end of setup (4618 / 5622 ticks).
Q8 = LegDesign(
    key='Q8',
    title='Q8',
    subtitle='Original five-bar parallel leg.',
    solver_factory=k_solver,
    mount_pose=(45.879, 134.121),
    source='Q8bot_rev2_5_FDM',
    natural_stance=(9.75, 43.36),
    gaits=Q8_GAITS,
    sitting=(9.75, 60.0, 20.0),
    rearing=(list(G1), list(G2), list(G3), 13.0),
    steps=(
        'Detach all four leg linkages before starting.',
        'Both joints of every leg are driven to the mounting pose and held.',
        'Fit each linkage so the upper links point along the held crank angle.',
        'Check that all four legs are mirrored the same way before releasing.',
    ),
)

# The MiNIQ leg. Declared so it appears in the picker, but it has no measured
# geometry yet, so it cannot be selected.
FULL_RANGE = LegDesign(
    key='FULL_RANGE',
    title='Full range',
    subtitle='MiNIQ leg. Geometry not measured yet.',
    solver_factory=None,
    mount_pose=None,
    available=False,
    source='Q8bot_MiNIQ_Leg',
    notes=('Pivot geometry still has to be read out of the MiNIQ Fusion model.',),
)

# The biomimetic SQuRo leg, measured from Q8bot_rev2_5_SQuRo_Leg.
BIOMIMETIC = LegDesign(
    key='BIOMIMETIC',
    title='Biomimetic',
    subtitle='SQuRo rat leg. Crank and pushrod, 2R equivalent.',
    solver_factory=squro_solver.front,
    mount_pose=squro_solver.MOUNT_POSE,
    # Same linkage, but the hind femur slopes forward and carries a longer,
    # pointed foot: its own solver and mounting pose (3627 / 4745 ticks).
    hind_solver_factory=squro_solver.hind,
    hind_mount_pose=HIND.mount_pose,
    supports_gaits=False,
    source='Q8bot_rev2_5_SQuRo_Leg',
    natural_stance=(33.935, 45.113),
    gaits=BIOMIMETIC_GAITS,
    sitting=(33.935, 62.43, 20.81),
    rearing=(BIOMIMETIC_G1, BIOMIMETIC_G2, BIOMIMETIC_G3, 13.0),
    steps=(
        'Detach all four leg linkages. Support the body; the feet leave the ground.',
        'Both cranks of every leg are driven to the mounting pose and held.',
        'Fit each shoulder crank pointing along the humerus line, then the elbow crank.',
        'Fit the hip and knee pushrods last; they set the two parallelograms.',
        'Confirm the head servo is daisy-chained and centred before releasing.',
    ),
    notes=(
        'Hind legs: femur slopes forward, pointed SQ4_UlnaFootHind, own mounting pose.',
        'Walking, sitting and rearing were derived from the front leg only; not enabled.',
    ),
)

LEG_DESIGNS = {design.key: design for design in (Q8, FULL_RANGE, BIOMIMETIC)}
DEFAULT_DESIGN = 'Q8'

# The head servo is part of the biomimetic build. It is configured and held at
# its centre, but it is not a ninth joint on the wire: the ESP-NOW packet still
# carries eight joint angles, so existing robot firmware keeps working.
HEAD_SERVO_ID = 19
HEAD_SERVO_CENTRE_TICKS = TICK_ZERO
HEAD_DESIGNS = ('BIOMIMETIC',)


def design_for(key):
    return LEG_DESIGNS[key]


def selectable():
    return [design for design in LEG_DESIGNS.values() if design.available]
