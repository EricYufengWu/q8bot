"""Head yaw from the GUI, kept inside the head's measured collision limits.

The limits come from the head sweeps at the end of leg detection: each sweep
runs the head gently to its first contact either side, and the limit sits
HEAD_MARGIN inside that contact (leg_detection.head_limits). Until a sweep has
measured them, only +-15 deg is allowed. The robot additionally clamps the
head to +-90 deg and caps its output at about 40 %, so a misjudged limit ends
in a soft bump rather than a forced one.

Which way the head turns for a positive angle depends on how the servo is
mounted, so the left/right sense can be swapped (Y) and is remembered.
"""

import json
import time

from leg_detection import state_dir

HEAD_RATE = 60.0          # deg/s while an arrow key is held
SEND_PERIOD = 0.05        # s; at most 20 head packets a second
MOVE_MS = 80              # each step's move time, a little longer than the period
DEFAULT_LIMITS = (-15.0, 15.0)


def _path():
    return state_dir() / 'head.json'


def load_settings():
    try:
        data = json.loads(_path().read_text())
        low, high = data['limits']
        if not low <= 0.0 <= high:
            raise ValueError
        return {'limits': (float(low), float(high)), 'measured': bool(data.get('measured')),
                'reversed': bool(data.get('reversed'))}
    except (OSError, ValueError, KeyError, TypeError):
        return {'limits': DEFAULT_LIMITS, 'measured': False, 'reversed': False}


def save_settings(settings):
    path = _path()
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps({'limits': list(settings['limits']),
                                'measured': settings['measured'],
                                'reversed': settings['reversed']}))


class HeadControl:
    def __init__(self, robot, clock=None):
        self.robot = robot
        self.clock = clock or time.monotonic
        settings = load_settings()
        self.limits = settings['limits']
        self.measured = settings['measured']
        self.reversed = settings['reversed']
        self.angle = 0.0
        # Only a key press moves the head: nothing is sent until one changes the
        # angle, so starting the GUI never torques or nudges the head by itself.
        self._dirty = False
        self._last_send = -1.0
        self._last_update = None

    def set_limits(self, limits):
        """New measured limits from a head sweep; saved for later sessions."""
        self.limits, self.measured = (float(limits[0]), float(limits[1])), True
        self.angle = self._clamp(self.angle)
        self._save()

    def set_present(self, deg):
        """Start from where the head actually is, so the first step is small."""
        self.angle = self._clamp(deg)

    def swap_direction(self):
        self.reversed = not self.reversed
        self._save()

    def update(self, direction, centre=False):
        """direction: -1 left, +1 right, 0 none, as the user sees the head."""
        now = self.clock()
        dt = 0.0 if self._last_update is None else min(now - self._last_update, 0.1)
        self._last_update = now
        target = self.angle
        if centre:
            target = self._clamp(0.0)
        elif direction:
            # Left is a positive servo angle unless the sense has been swapped.
            sign = 1.0 if self.reversed else -1.0
            target = self._clamp(self.angle + sign * direction * HEAD_RATE * dt)
        if centre or target != self.angle:
            self.angle = target
            self._dirty = True
        if self._dirty and now - self._last_send >= SEND_PERIOD:
            self.robot.move_head(self.angle, MOVE_MS)
            self._dirty = False
            self._last_send = now

    def label(self):
        low, high = self.limits
        note = '' if self.measured else ', not measured yet'
        return f'Head {self.angle:+.0f} deg   limits {low:+.0f} to {high:+.0f}{note}'

    def _clamp(self, angle):
        low, high = self.limits
        return max(low, min(high, angle))

    def _save(self):
        save_settings({'limits': self.limits, 'measured': self.measured, 'reversed': self.reversed})
