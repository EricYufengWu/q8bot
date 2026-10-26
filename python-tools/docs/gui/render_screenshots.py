"""Render GUI screenshots headless, with no ESP32 attached.

Draws the same frames operate.py's main loop draws, via a dummy SDL video
driver, and writes them next to this file.

    venv/bin/python docs/gui/render_screenshots.py
"""
import os
import sys
import time
from pathlib import Path

os.environ.setdefault('SDL_VIDEODRIVER', 'dummy')
os.environ.setdefault('SDL_AUDIODRIVER', 'dummy')

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent.parent
sys.path.insert(0, str(ROOT / 'q8bot'))

import pygame
import eth_theme as eth
from control_panel import ControlPanel, WINDOW_SIZE
from helpers import Q8Logger
from leg_calibration import CONFIRM, HOLD, LegCalibration
from rearing_pose import RearingPose
from sitting_pose import SittingPose

pygame.init()
window = pygame.display.set_mode(WINDOW_SIZE)
pygame.display.set_caption('Q8bot - Gaits and poses')
Q8Logger.set_pygame_surface(window)
panel = ControlPanel(ROOT / 'docs' / 'poses')


def frame(gait, pose, messages, name, calibration=None):
    Q8Logger._message_log = list(messages)
    window.fill(eth.BACKGROUND)
    panel.draw(window, gait, pose, calibration)
    Q8Logger.render_pygame_messages()
    pygame.display.flip()
    pygame.image.save(window, str(HERE / name))
    print(f'wrote {name} ({panel.active_preview})')


stamp = time.strftime('%H:%M:%S')
ready = (stamp, 'INFO', 'Controls ready. G: switch gait. P: sit / stand. U: rear / lower.')
idle = SittingPose(None, None)
sitting = SittingPose(None, None)
sitting.active = True
rearing = RearingPose(None, None)
rearing.active = True

frame('TROT', idle, [ready], 'gui_startup.png')
frame('WALK', idle, [ready, (stamp, 'INFO', 'Switched to WALK'),
                     (stamp, 'INFO', 'Battery: 87%')], 'gui_walk.png')
frame('TROT', sitting, [(stamp, 'INFO', 'Sitting pose')], 'gui_sitting.png')
frame('TROT', rearing, [(stamp, 'INFO', 'Rearing pose')], 'gui_rearing.png')


class _Bench:
    """Stands in for the robot so the calibration screens render offline."""
    def move_all(self, *_args, **_kwargs):
        return True


cal = LegCalibration(_Bench())
cal.open_picker()
cal.cursor = 2
frame('TROT', idle, [(stamp, 'INFO', 'Change legs')], 'gui_legs_picker.png', cal)
cal.choose()
frame('TROT', idle, [(stamp, 'INFO', 'Calibrating Biomimetic legs.')],
      'gui_legs_confirm.png', cal)
cal.confirm()
cal.stage = HOLD
frame('TROT', idle, [(stamp, 'INFO', 'Driving to the mounting pose. Keep hands clear.')],
      'gui_legs_hold.png', cal)


# --- Startup leg detection and the biomimetic stand mode -------------------
import tempfile
os.environ['Q8BOT_HOME'] = tempfile.mkdtemp(prefix='q8bot-shots-')
from leg_designs import BIOMIMETIC
from leg_detection import LegDetector
from stand_up import StandUp
from startup_flow import StartupFlow


class _Legs:
    """Answers like the robot firmware, with stops at fixed joint angles."""
    ANGLES = [20.0, 120.0, 20.0, 120.0, -40.0, 50.0, -40.0, 50.0]
    STOPS = {0: (60, -40), 1: (150, 60), 4: (-10, -80), 5: (110, 10)}

    def __init__(self):
        self.replies = []

    def request_telemetry(self):
        servos = {11 + i: {'responding': True, 'torque': False, 'fault': False, 'deg': a, 'error': 0}
                  for i, a in enumerate(self.ANGLES)}
        self.replies.append({'kind': 'telemetry', 'servos': servos})

    def start_probe(self, excursions, pwm):
        joints = [{'swept': False, 'involved': False, 'reason': 'none', 'start': None, 'stop': None}
                  for _ in range(8)]
        for i, e in enumerate(excursions):
            if not e:
                continue
            start = self.ANGLES[i]
            plus, minus = self.STOPS[i % 2 + (4 if i >= 4 else 0)]
            wall = plus if e > 0 else minus
            hit = min(start, start + e) <= wall <= max(start, start + e)
            joints[i] = {'swept': True, 'involved': True, 'reason': 'contact' if hit else 'bound',
                         'start': start, 'stop': wall if hit else start + e}
        self.replies.append({'kind': 'probe', 'status': 0, 'status_text': 'ok', 'joints': joints})

    def abort_probe(self):
        pass


def startup_frame(flow, name):
    window.fill(eth.BACKGROUND)
    panel.draw(window, 'TROT', idle, startup=flow)
    pygame.display.flip()
    pygame.image.save(window, str(HERE / name))
    print(f'wrote {name} ({panel.active_preview})')


legs = _Legs()
flow = StartupFlow(LegDetector(legs), Q8Logger)
flow.begin()
flow.handle([], legs.replies[:]); legs.replies.clear(); flow.update()
startup_frame(flow, 'gui_startup_lift.png')
flow._enter()
while legs.replies:
    flow.handle([], [legs.replies.pop(0)]); flow.update()
flow.cursor = 1
flow.detector.record('BIOMIMETIC')
startup_frame(flow, 'gui_startup_detected.png')

panel.set_design('BIOMIMETIC')
frame('TROT', StandUp(BIOMIMETIC, legs), [(stamp, 'INFO', 'Biomimetic legs. Set the robot down on '
                                          'its feet, then press T to stand up.')], 'gui_stand_mode.png')
pygame.quit()
