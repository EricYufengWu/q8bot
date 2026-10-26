"""Nonblocking greeting-style entry, static rear, and return to the selected gait."""
from collections import deque
import math
import time

from routine_generator import G1, G2, G3


class RearingPose:
    # Tune this starting offset on the physical robot for the payload.
    # Reducing both rear angles moves the support feet forward relative to
    # the rear hips (so the body is farther back relative to those feet).
    REARWARD_ANGLE_OFFSET = 13.0
    REAR_TRANSITION_MS = 1500
    FRONT_SUPPORT_X = 9.75
    FRONT_SUPPORT_HEIGHT = 60.0
    FRONT_DOWN_MS = 2000
    REAR_DOWN_MS = 2500
    SETTLE_MS = 1500
    SUPPORT_PAUSE_S = 0.2
    POSE_NAME = 'REARING'

    def __init__(self, leg, robot, clock=time.monotonic, waypoints=None):
        self.leg = leg
        self.robot = robot
        self.clock = clock
        # (G1, G2, G3, rearward_offset) for the fitted design. The greeting is
        # stored as joint angles, so each design carries its own remap.
        self.g1, self.g2, self.g3, self.rearward_offset = waypoints or (
            list(G1), list(G2), list(G3), self.REARWARD_ANGLE_OFFSET)
        self.active = False
        self.transition_until = 0.0
        self._steps = deque()
        self.failed = False
        self._last_positions = None

    @classmethod
    def target_positions(cls, waypoints=None):
        # G3 is the greeting support stance; omit its G4/G5 waving motions.
        g3, offset = ((waypoints[2], waypoints[3]) if waypoints
                      else (G3, cls.REARWARD_ANGLE_OFFSET))
        positions = list(g3)
        positions[4:] = [angle - offset for angle in g3[4:]]
        return positions

    def _targets(self):
        return self.target_positions((self.g1, self.g2, self.g3, self.rearward_offset))

    def toggle(self, gait_x, gait_y):
        """Start entry/exit without blocking input. Returns False on send failure."""
        if self.active:
            steps = self.lowering_steps(gait_x, gait_y)
        else:
            steps = [(list(self.g1), 1000, 1.1), (list(self.g2), 1000, 1.1),
                     (self._targets(), self.REAR_TRANSITION_MS, self.REAR_TRANSITION_MS / 1000)]
        self._validate_steps(steps)
        positions, duration, wait = steps[0]
        if not self.robot.move_all(positions, duration, False):
            return False
        self._last_positions = list(positions)
        self._steps = deque(steps[1:])
        self.active = not self.active
        self.failed = False
        self.transition_until = self.clock() + wait
        return True

    def lowering_steps(self, gait_x, gait_y):
        """Front support first, rear adjustment second, final gait height last."""
        def pair(x, y):
            q1, q2, valid = self.leg.ik_solve(x, y, True, 3)
            if not valid:
                raise ValueError('Cannot lower from rearing: unreachable foot position')
            return [float(q1), float(q2)]
        rest = pair(gait_x, gait_y)
        front_support = pair(self.FRONT_SUPPORT_X, self.FRONT_SUPPORT_HEIGHT)
        # Keep the most recent rear targets, including when entry was interrupted
        # or a failed descent is retried. No servo position feedback is available.
        rear_hold = (self._last_positions or self._targets())[4:]
        return [
            (front_support * 2 + rear_hold, self.FRONT_DOWN_MS,
             self.FRONT_DOWN_MS / 1000 + self.SUPPORT_PAUSE_S),
            (front_support * 2 + rest * 2, self.REAR_DOWN_MS,
             self.REAR_DOWN_MS / 1000 + self.SUPPORT_PAUSE_S),
            (rest * 4, self.SETTLE_MS, self.SETTLE_MS / 1000),
        ]

    def _validate_steps(self, steps):
        # Five-bar closure: the two knee circles must intersect. Unlike the
        # legacy FK solver this also handles the greeting's folded entry pose.
        for positions, _, _ in steps:
            if len(positions) != 8 or not all(math.isfinite(p) for p in positions):
                raise ValueError('Invalid rearing joint positions')
            if not hasattr(self.leg, 'l1p'):
                # Closure below is specific to the five-bar; other leg designs
                # carry their own reach limits inside ik_solve.
                continue
            for index in range(0, 8, 2):
                q1, q2 = map(math.radians, positions[index:index+2])
                dx = self.leg.d + self.leg.l1 * math.cos(q1) - self.leg.l1p * math.cos(q2)
                dy = self.leg.l1 * math.sin(q1) - self.leg.l1p * math.sin(q2)
                separation = math.hypot(dx, dy)
                if not abs(self.leg.l2 - self.leg.l2p) < separation < self.leg.l2 + self.leg.l2p:
                    raise ValueError('Rearing linkage cannot reach the requested angles')

    def update(self):
        """Advance at most one waypoint. Failed sends pause until U retries."""
        if self.failed or not self._steps or self.clock() < self.transition_until:
            return None
        positions, duration, wait = self._steps[0]
        if not self.robot.move_all(positions, duration, False):
            self.failed = True
            return 'Rearing transition send failed. Press U to retry / lower.'
        self._steps.popleft()
        self._last_positions = list(positions)
        self.transition_until = self.clock() + wait
        return None

    def retry_lowering(self, gait_x, gait_y):
        """After a failed exit, U retries lowering instead of starting to rear."""
        was_active = self.active
        self.active = True
        try:
            result = self.toggle(gait_x, gait_y)
        except ValueError:
            self.active = was_active
            raise
        if not result:
            self.active = was_active
        return result

    def blocks_movement(self):
        return self.active or self.failed or bool(self._steps) or self.clock() < self.transition_until

    def label(self):
        if self.failed:
            return 'U: Retry lowering  |  Transition paused - movement blocked'
        if self.active:
            action = 'Rising...' if self._steps or self.clock() < self.transition_until else 'REARING - holding pose'
            return f'U: Lower to gait  |  {action}  |  Esc: Release torque'
        phase = {2: 'Extending front legs...', 1: 'Lowering rear legs...',
                 0: 'Settling into gait position...'}[len(self._steps)]
        return f'{phase}  |  Esc: Release torque'
