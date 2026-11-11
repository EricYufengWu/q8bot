"""Leg-design picker and the leg-attachment calibration sequence.

Calibration drives every joint to the selected design's mounting pose and
holds it there while the linkages are fitted, which is what fixes the keying
between each servo horn and its crank. It is the host-side equivalent of what
the motor-config firmware does once at the end of first setup, so the pose can
be re-established after a leg swap without reflashing the robot.

Nothing here blocks: ``update`` advances at most one stage per frame, so the
window keeps redrawing and Esc keeps working while the servos are moving.

Once the move has had time, the servos are read back: the pose is held only
if every joint got there, otherwise calibration fails with a diagnosis
(servo_health) - a servo not answering, a servo fault, or a joint short of
its goal.
"""

import time

from leg_designs import DEFAULT_DESIGN, LEG_DESIGNS, design_for
from servo_health import Diagnosis, check, no_answer

PICK, CONFIRM, MOVING, CHECKING, HOLD, FAILED = (
    'PICK', 'CONFIRM', 'MOVING', 'CHECKING', 'HOLD', 'FAILED')


class LegCalibration:
    POSE_NAME = 'CALIBRATION'
    MOVE_MS = 2500
    REPLY_TIMEOUT = 1.5
    # Torque stays on through the hold so the cranks keep their reference.
    ORDER = tuple(LEG_DESIGNS)

    def __init__(self, robot, clock=time.monotonic, design_key=DEFAULT_DESIGN):
        self.robot = robot
        self.clock = clock
        self.stage = None
        self.fitted_key = design_key      # The design currently on the robot.
        self.cursor = self.ORDER.index(design_key)
        self.chosen_key = None            # The design being calibrated.
        self.move_until = 0.0
        self.error = None
        self.diagnosis = None
        self._deadline = 0.0
        self._reports = []

    # --- state -----------------------------------------------------------
    @property
    def active(self):
        return self.stage is not None

    def blocks_movement(self):
        """Gaits and poses stay suspended for the whole sequence."""
        return self.active

    def fitted(self):
        return design_for(self.fitted_key)

    def highlighted(self):
        return design_for(self.ORDER[self.cursor])

    def chosen(self):
        return design_for(self.chosen_key) if self.chosen_key else None

    # --- transitions -----------------------------------------------------
    def open_picker(self):
        self.stage = PICK
        self.cursor = self.ORDER.index(self.fitted_key)
        self.chosen_key = None
        self.error = None

    def move_cursor(self, delta):
        if self.stage == PICK:
            self.cursor = (self.cursor + delta) % len(self.ORDER)

    def choose(self):
        """Accept the highlighted design and show its calibration steps."""
        if self.stage != PICK:
            return False
        design = self.highlighted()
        if not design.available:
            self.error = f'{design.title} has no kinematic model yet.'
            return False
        self.chosen_key = design.key
        self.stage = CONFIRM
        self.error = None
        return True

    def confirm(self):
        """Send the mounting pose. Torque must already be on."""
        if self.stage not in (CONFIRM, FAILED):
            return False
        design = self.chosen()
        if not self.robot.move_all(design.mount_positions(), self.MOVE_MS, False):
            self._failed('Could not send the mounting pose.', Diagnosis(
                'communication', 'the controller did not take the command',
                'Check that the controller dongle is plugged in, then retry.'))
            return False
        self.stage = MOVING
        self.error = None
        self.diagnosis = None
        self.move_until = self.clock() + self.MOVE_MS / 1000
        return True

    def update(self):
        """Once the commanded move has had time, read the servos back."""
        if self.stage == MOVING and self.clock() >= self.move_until:
            self.stage = CHECKING
            self.robot.request_telemetry()
            self._deadline = self.clock() + self.REPLY_TIMEOUT
        elif self.stage == CHECKING and self.clock() > self._deadline:
            self._failed('No reply after the move.', no_answer())
        return None

    def feed(self, reply):
        """HOLD only when every joint reached the mounting pose."""
        if reply['kind'] != 'telemetry' or self.stage != CHECKING:
            return
        problem = check(reply, self.chosen().mount_positions())
        if problem:
            self._failed('Mounting pose not reached.', problem)
        else:
            self.stage = HOLD

    def reports(self):
        reports, self._reports = self._reports, []
        return reports

    def _failed(self, error, diagnosis):
        self.stage, self.error, self.diagnosis = FAILED, error, diagnosis
        self._reports += [f'Calibration stopped: {error}', diagnosis.text()]

    def finish(self):
        """Commit the swap: the chosen design becomes the fitted one."""
        if self.stage != HOLD:
            return False
        self.fitted_key = self.chosen_key
        self.stage = None
        self.chosen_key = None
        return True

    def cancel(self):
        """Leave calibration without changing the fitted design."""
        if not self.active:
            return False
        self.stage = None
        self.chosen_key = None
        self.error = None
        return True

    # --- presentation ----------------------------------------------------
    def label(self):
        if self.stage == PICK:
            note = self.error or 'Up/Down: choose   Enter: calibrate   Esc: cancel'
            return f'CHANGE LEGS  |  {note}'
        design = self.chosen()
        if self.stage == CONFIRM:
            return (f'{design.title}  |  Detach linkages and support the robot  |  '
                    'Enter: drive to mounting pose   Esc: cancel')
        if self.stage == MOVING:
            return f'{design.title}  |  Driving to mounting pose...  |  Esc: cancel'
        if self.stage == CHECKING:
            return f'{design.title}  |  Checking every joint got there...  |  Esc: cancel'
        if self.stage == HOLD:
            return (f'{design.title}  |  HOLDING mounting pose - fit the linkages  |  '
                    'Enter: done   Esc: cancel')
        if self.stage == FAILED:
            reason = self.diagnosis.title if self.diagnosis else self.error
            return f'{design.title}  |  {reason}  |  Enter: retry   Esc: cancel'
        return ''
