"""Static sitting pose using the same foot coordinates as the gait generator."""

import time


class SittingPose:
    POSE_NAME = 'SITTING'
    # Extend the front as in TROT_HIGH; crouch the rear below TROT_LOW.
    FRONT_HEIGHT = 60.0
    REAR_HEIGHT = 20.0
    FOOT_X = 9.75
    TRANSITION_MS = 1000

    def __init__(self, leg, robot, clock=time.monotonic, targets=None):
        self.leg = leg
        self.robot = robot
        self.clock = clock
        # (foot_x, front, rear) for the fitted leg design; the class constants
        # remain the five-bar values so existing callers keep working.
        self.foot_x, self.front_height, self.rear_height = targets or (
            self.FOOT_X, self.FRONT_HEIGHT, self.REAR_HEIGHT)
        self.active = False
        self.transition_until = 0.0

    def _pair(self, x, y):
        q1, q2, success = self.leg.ik_solve(x, y, True, 3)
        if not success:
            raise ValueError(f"Unreachable sitting/standing foot position: {x}, {y}")
        return [float(q1), float(q2)]

    def toggle(self, gait_x, gait_y):
        """Send one pose command; commit the toggle only if sending succeeds."""
        if self.active:
            positions = self._pair(gait_x, gait_y) * 4
        else:
            front = self._pair(self.foot_x, self.front_height)
            rear = self._pair(self.foot_x, self.rear_height)
            # Servo order, as in append_pos_list: FL, FR, BL, BR.
            positions = front * 2 + rear * 2

        if not self.robot.move_all(positions, self.TRANSITION_MS, False):
            return False
        self.active = not self.active
        self.transition_until = self.clock() + self.TRANSITION_MS / 1000
        return True

    def blocks_movement(self):
        # Allow the return to standing to finish before starting another gait.
        return self.active or self.clock() < self.transition_until

    def label(self):
        if self.active:
            return "P: Stand up  |  SITTING - movement paused"
        if self.clock() < self.transition_until:
            return "P: Sit down  |  Returning to gait position..."
        return "P: Sit down  |  U: Rear up  |  Esc: Exit and release torque"
