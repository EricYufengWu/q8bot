"""Kinematics for the biomimetic (SQuRo) legs, front and hind.

Geometry measured from the Fusion model ``Q8bot_rev2_5_SQuRo_Leg``: the pivot
axes of SQ3_ShoulderCrank, SQ3_ElbowCrank, SQ3_Humerus, SQ3_HipPushrod,
SQ3_KneePushrod, SQ3_UlnaFoot (front) and SQ4_UlnaFootHind (hind), placed with
their assembly occurrence transforms.

Leg frame, shared by front and hind and matching kinematics_solver.k_solver:
+x points toward the nose, +y points down (away from the body), and the origin
is the shoulder-crank axis, so the two motor axes sit at (0, 0) and (d, 0).

    A  = (0, 0)   shoulder-crank axis; the servo of the pair nearer the tail
    B  = (d, 0)   elbow-crank axis and humerus/femur pivot; nearer the nose
    P1 = shoulder crank pin      P2 = hip pushrod -> humerus
    Q1 = elbow crank pin         Q2 = knee pushrod -> ulna
    E  = humerus -> ulna pivot (elbow on the forelimb, knee on the hindlimb)
    F  = foot reference: round pad centre (front), toe-tip round centre (hind)

Both legs use the same cranks, pushrods and humerus, so both are the same
*double parallelogram*:

    |A-P1| = |B-P2| = 15.920      |P1-P2| = |A-B| = 19.500
    |B-Q1| = |E-Q2| = 15.390      |Q1-Q2| = |B-E| = 34.870

with B, P2, E collinear. Kinematically each is a plain 2R serial arm with both
actuators left on the body: the humerus angle equals the shoulder-crank angle
and the ulna angle equals the elbow-crank angle, so

    E = B + L1 * u(theta1)
    F = E + L2 * u(theta2 + foot_offset)

What differs between front and hind is the foot part and the pose. The front
humerus slopes back from B with the elbow behind; the hind femur slopes
forward with the knee in front. That puts the two legs on opposite branches of
the inverse kinematics, and gives the hind foot a much longer reach (59.4 mm to
the toe against 40.6 mm to the front pad centre). Both feet meet the ground at
the same depth below the servo axes in the CAD stance: 64.213 mm.
"""

import math

import numpy as np

# Front-left pivots as measured, in the leg frame (mm).
MEASURED_PIVOTS = {
    'A': (0.000, 0.000), 'B': (19.500, 0.000),
    'P1': (-10.023, 12.369), 'P2': (9.477, 12.369),
    'Q1': (28.835, 12.236), 'Q2': (6.881, 39.327),
    'E': (-2.454, 27.091), 'F': (33.935, 45.113),
}

# Hind-left pivots, from the crank pins in
# results/2026-10-01/cad-crank-references.json and the SQ4_UlnaFootHind_P:3
# occurrence (translation 40.25 mm, rotated 180 deg). E follows from the knee
# parallelogram; F is the centre of the 1.1 mm round at the toe tip.
MEASURED_HIND_PIVOTS = {
    'A': (0.000, 0.000), 'B': (19.500, 0.000),
    'P1': (8.6707, 13.3516), 'P2': (28.1707, 13.3516),
    'Q1': (31.0644, -10.1546), 'Q2': (50.0560, 19.0900),
    'E': (38.4916, 29.2446), 'F': (-10.3340, 63.1130),
}


class LegSpec:
    """The parameters that distinguish one SQuRo leg type from another."""

    def __init__(self, name, l2, foot_offset, branch, mount_pose, contact_radius, stance):
        self.name = name
        self.l2 = l2                      # E to foot reference (mm)
        self.foot_offset = foot_offset    # elbow-crank direction to the foot line (deg)
        self.branch = branch              # +1 elbow behind (front), -1 knee forward (hind)
        self.mount_pose = mount_pose      # (q1 elbow crank, q2 shoulder crank) in deg
        self.contact_radius = contact_radius  # foot reference to ground contact (mm)
        self.stance = stance              # foot reference in the CAD stance (mm)


FRONT = LegSpec('front', l2=40.607, foot_offset=-26.312, branch=+1,
                mount_pose=(52.660, 129.021), contact_radius=19.1,
                stance=(33.935, 45.113))
HIND = LegSpec('hind', l2=59.4223, foot_offset=186.5385, branch=-1,
               mount_pose=(-41.286, 57.000), contact_radius=1.1,
               stance=(-10.334, 63.113))


class squro_solver:
    """Biomimetic leg kinematics, interface-compatible with ``k_solver``.

    ``ik_solve``/``fk_solve`` use the same joint ordering as the five-bar
    solver: ``q1`` drives the motor at (d, 0) and ``q2`` the motor at (0, 0),
    which is the order ``q8_espnow.move_all`` puts on the wire. Construct with
    no arguments (or ``squro_solver.front()``) for a front leg, and with
    ``squro_solver.hind()`` for a hind leg.
    """

    # Humerus B->E; shared by both legs.
    L1 = 34.870
    # Front-leg defaults, kept as class attributes for existing callers.
    L2 = FRONT.l2
    FOOT_OFFSET = FRONT.foot_offset
    MOUNT_POSE = FRONT.mount_pose
    # Link lengths kept for the assembly drawing and the parts list.
    SHOULDER_CRANK = 15.920
    ELBOW_CRANK = 15.390
    HIP_PUSHROD = 19.500
    KNEE_PUSHROD = 34.870
    # Ground contact depth below the servo axes in the CAD stance (both legs).
    STANCE_CONTACT_DEPTH = 64.213

    def __init__(self, spec=FRONT, d=19.5, l1=L1):
        self.spec = spec
        self.d = d
        self.l1 = l1
        self.l2 = spec.l2
        self.foot_offset = spec.foot_offset
        self.branch = spec.branch
        self.mount_pose = spec.mount_pose
        self.contact_radius = spec.contact_radius
        self.prev_ik = list(spec.mount_pose)

    @classmethod
    def front(cls):
        return cls(FRONT)

    @classmethod
    def hind(cls):
        return cls(HIND)

    def ik_check(self, x, y):
        """True when the foot lies inside the annulus the 2R arm can reach."""
        reach = math.hypot(x - self.d, y)
        return abs(self.l1 - self.l2) <= reach <= self.l1 + self.l2 and reach > 0

    def ik_solve(self, x, y, deg=True, rounding=3):
        """Closed-form 2R inverse kinematics. Returns (q1, q2, success)."""
        vx, vy = x - self.d, y
        reach = math.hypot(vx, vy)
        if not self.ik_check(x, y):
            return self.prev_ik[0], self.prev_ik[1], False
        cos_beta = (reach**2 + self.l1**2 - self.l2**2) / (2 * reach * self.l1)
        # Guard the arccos against reach values sitting exactly on a limit.
        beta = math.acos(max(-1.0, min(1.0, cos_beta)))
        # The branch is the one the CAD build uses: elbow behind on the front
        # leg, knee forward on the hind leg. The other branch is the mirrored
        # assembly and would drive the linkage through its hard stops.
        theta1 = math.atan2(vy, vx) + self.branch * beta
        ex = self.d + self.l1 * math.cos(theta1)
        ey = self.l1 * math.sin(theta1)
        theta2 = math.atan2(y - ey, x - ex) - math.radians(self.foot_offset)
        q1 = _near(math.degrees(theta2), self.mount_pose[0])
        q2 = _near(math.degrees(theta1), self.mount_pose[1])
        if not deg:
            q1, q2 = math.radians(q1), math.radians(q2)
        self.prev_ik = [q1, q2]
        return np.round(q1, rounding), np.round(q2, rounding), True

    def fk_check(self):
        return True

    def fk_solve(self, q1, q2, deg=True, rounding=3):
        """Foot reference position for the commanded joint pair. Returns (x, y)."""
        theta2, theta1 = (q1, q2) if deg else (math.degrees(q1), math.degrees(q2))
        x, y = self.pivots(theta2, theta1)['F']
        return round(x, rounding), round(y, rounding)

    def contact_depth(self, q1, q2):
        """Depth of the lowest foot point below the servo axes (mm)."""
        return self.pivots(q1, q2)['F'][1] + self.contact_radius

    def pivots(self, q1, q2):
        """Every linkage pivot at the given joint pair, in degrees.

        Used by the drawings, so a picture and the commanded pose cannot drift
        apart.
        """
        theta2, theta1 = math.radians(q1), math.radians(q2)
        foot_dir = theta2 + math.radians(self.foot_offset)
        a = (0.0, 0.0)
        b = (self.d, 0.0)
        e = (b[0] + self.l1 * math.cos(theta1), b[1] + self.l1 * math.sin(theta1))
        p1 = (a[0] + self.SHOULDER_CRANK * math.cos(theta1),
              a[1] + self.SHOULDER_CRANK * math.sin(theta1))
        p2 = (p1[0] + self.HIP_PUSHROD, p1[1])
        q1p = (b[0] + self.ELBOW_CRANK * math.cos(theta2),
               b[1] + self.ELBOW_CRANK * math.sin(theta2))
        q2p = (e[0] + self.ELBOW_CRANK * math.cos(theta2),
               e[1] + self.ELBOW_CRANK * math.sin(theta2))
        f = (e[0] + self.l2 * math.cos(foot_dir), e[1] + self.l2 * math.sin(foot_dir))
        return {'A': a, 'B': b, 'P1': p1, 'P2': p2, 'Q1': q1p, 'Q2': q2p, 'E': e, 'F': f}


def _near(angle, reference):
    """The 360-degree equivalent of ``angle`` closest to ``reference``.

    atan2 answers in (-180, 180], but a servo commanded 360 degrees away from
    where it is would turn a full revolution. Keeping every answer next to the
    leg's own mounting angle keeps the commands on the same turn as the servo.
    """
    return angle + 360.0 * round((reference - angle) / 360.0)
