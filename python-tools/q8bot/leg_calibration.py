"""Leg-design picker and the leg-attachment calibration sequence.

Calibration drives every joint to the selected design's mounting pose and
holds it there while the linkages are fitted, which is what fixes the keying
between each servo horn and its crank. It is the host-side equivalent of what
the motor-config firmware does once at the end of first setup, so the pose can
be re-established after a leg swap without reflashing the robot.

Nothing here blocks: ``update`` advances at most one stage per frame, so the
window keeps redrawing and Esc keeps working while the servos are moving.
"""

import time

from leg_designs import DEFAULT_DESIGN, LEG_DESIGNS, design_for

PICK, CONFIRM, MOVING, HOLD, FAILED = 'PICK', 'CONFIRM', 'MOVING', 'HOLD', 'FAILED'


class LegCalibration:
    POSE_NAME = 'CALIBRATION'
    MOVE_MS = 2500
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
            self.stage = FAILED
            self.error = 'Could not send the mounting pose.'
            return False
        self.stage = MOVING
        self.error = None
        self.move_until = self.clock() + self.MOVE_MS / 1000
        return True

    def update(self):
        """Advance from MOVING to HOLD once the commanded move has had time."""
        if self.stage == MOVING and self.clock() >= self.move_until:
            self.stage = HOLD
        return None

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
        if self.stage == HOLD:
            return (f'{design.title}  |  HOLDING mounting pose - fit the linkages  |  '
                    'Enter: done   Esc: cancel')
        if self.stage == FAILED:
            return f'{design.title}  |  {self.error}  |  Enter: retry   Esc: cancel'
        return ''
