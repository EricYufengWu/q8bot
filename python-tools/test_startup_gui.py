"""The GUI's startup: leg detection before any torque, then the fitted design."""
import os
os.environ.setdefault('SDL_VIDEODRIVER', 'dummy')
os.environ.setdefault('SDL_AUDIODRIVER', 'dummy')
import json
import runpy
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / 'q8bot'))
sys.path.insert(0, str(ROOT))
import pygame
import espnow
import stand_up as su
from test_leg_detection import HANGING, SQURO_WALLS, probe_values, telemetry_values
from input_handler import InputHandler


class FakeLink:
    """Stands in for q8_espnow and the controller: replies arrive as serial lines."""

    def __init__(self, angles, walls, silent=False):
        self.angles, self.walls, self.silent = list(angles), walls, silent
        self.pending = []
        self.calls = []
        self.torque_on = False
        link = self

        class Port:
            @property
            def in_waiting(self):
                return len(link.pending)

            def readline(self):
                return link.pending.pop(0)

        self.serialHandler = Port()

    def _reply(self, values):
        if not self.silent:
            self.pending.append((' '.join(str(v) for v in values) + ' \r\n').encode())

    def request_telemetry(self):
        self.calls.append(('telemetry',))
        self._reply(telemetry_values(self.angles + [0.0]))

    def start_probe(self, excursions, pwm):
        self.calls.append(('probe', list(excursions), pwm))
        joints = {}
        for i, excursion in enumerate(excursions):
            if not excursion:
                continue
            start = self.angles[i]
            wall = self.walls.get((i, 1 if excursion > 0 else -1))
            reach = start + excursion
            hit = wall is not None and min(start, reach) <= wall <= max(start, reach)
            joints[i] = (3, 1 if hit else 3, start, wall if hit else reach)
            joints[i ^ 1] = (1, 4, self.angles[i ^ 1], self.angles[i ^ 1])
        self._reply(probe_values(0, joints))

    def start_head_probe(self, bound, pwm):
        self.calls.append(('head_probe', bound, pwm))
        start, wall = 0.0, self.walls.get((8, 1 if bound > 0 else -1))
        hit = wall is not None and min(start, start + bound) <= wall <= max(start, start + bound)
        self._reply(probe_values(0, {8: (3, 1 if hit else 3, start, wall if hit else start + bound)}))

    def move_head(self, deg, ms):
        self.calls.append(('head', deg, ms))
        return True

    def abort_probe(self):
        self.calls.append(('abort',))

    def set_output_cap(self, pwm):
        self.calls.append(('cap', pwm))
        values = [0] * 100
        values[0], values[1], values[2] = espnow.TAG_CAP, pwm, 0xFF
        self._reply(values)

    def set_turn_offsets(self, present, model):
        self.calls.append(('offsets',))

    def arm(self, positions):
        self.calls.append(('arm', list(positions)))
        self.angles[:8] = list(positions)
        self.torque_on = True
        return True

    def enable_torque(self):
        self.calls.append(('torque', 1))
        self.torque_on = True
        return True

    def disable_torque(self):
        self.calls.append(('torque', 0))
        self.torque_on = False
        return True

    def move_all(self, positions, duration=0, record=True):
        self.calls.append(('move', list(positions), duration))
        self.angles[:8] = list(positions)          # the servos follow
        return True

    def move_mirror(self, pair, duration=0):
        self.calls.append(('move', list(pair) * 4, duration))
        return True

    def __getattr__(self, name):
        # Any other robot command (battery, recording) is recorded, not run.
        return lambda *a, **k: self.calls.append((name,)) or True

    def kinds(self):
        return [c[0] for c in self.calls]


def key(k):
    return [pygame.event.Event(pygame.KEYDOWN, key=k), pygame.event.Event(pygame.KEYUP, key=k)]


class StartupGuiTests(unittest.TestCase):
    def setUp(self):
        pygame.init()
        self.home = tempfile.TemporaryDirectory()
        self.patch_home = patch.dict(os.environ, {'Q8BOT_HOME': self.home.name})
        self.patch_home.start()

    def tearDown(self):
        self.patch_home.stop()
        self.home.cleanup()
        pygame.quit()

    def run_gui(self, link, script, frames=400, argv=(), held=None):
        frame = [0]
        marks = {}

        def events():
            frame[0] += 1
            n = frame[0]
            if n >= frames:
                return [pygame.event.Event(pygame.QUIT)]
            for when, (label, produce) in script.items():
                if n == when:
                    marks[label] = len(link.calls)
                    return produce
            return []

        # Movement and most actions are read from key state, not events.
        held = held or (lambda name, frame: False)
        moving = patch.object(InputHandler, 'is_movement_input',
                              side_effect=lambda: held('move', frame[0]))
        acting = patch.object(InputHandler, 'is_action_pressed',
                              side_effect=lambda name: held(name, frame[0]))
        with moving, acting, patch('espnow.q8_espnow', return_value=link), \
             patch('helpers.XiaoPortFinder.validate', return_value=True), \
             patch.object(pygame.event, 'get', side_effect=events), \
             patch('time.sleep'), \
             patch.object(sys, 'argv', ['operate.py', 'mock-port', *argv]):
            runpy.run_path(str(ROOT / 'q8bot' / 'operate.py'), run_name='__main__')
        return marks

    def test_detect_record_choose_and_stand_up(self):
        link = FakeLink(HANGING, SQURO_WALLS)
        marks = self.run_gui(link, {
            5: ('enter', key(pygame.K_RETURN)),        # lifted: detect
            40: ('down', key(pygame.K_DOWN)),          # highlight Biomimetic
            41: ('arm_record', key(pygame.K_r)),       # first R only asks
            42: ('record', key(pygame.K_r)),           # second R records
            44: ('accept', key(pygame.K_RETURN)),      # use it
            50: ('stand', key(pygame.K_t)),            # stand up
        }, frames=90)
        before = link.calls[:marks['accept']]
        # Nothing was armed or moved while the design was unknown.
        self.assertNotIn('torque', [c[0] for c in before if c[0] == 'torque' and c[1] == 1])
        self.assertNotIn('move', [c[0] for c in before])
        self.assertNotIn('arm', [c[0] for c in before])
        probes = [c for c in before if c[0] == 'probe']
        self.assertEqual(len(probes), 4)
        for _, excursions, pwm in probes:
            self.assertEqual(pwm, 140)
            for leg in range(4):
                self.assertFalse(excursions[leg * 2] and excursions[leg * 2 + 1])
        # Biomimetic starts released: no torque until T.
        between = link.calls[marks['accept']:marks['stand']]
        self.assertNotIn(('torque', 1), between)
        self.assertFalse(any(c[0] == 'move' for c in between))
        # T: measure, cap the output, arm on the measured pose, then waypoints.
        after = link.kinds()[marks['stand']:]
        self.assertIn('arm', after)
        self.assertLess(after.index('cap'), after.index('arm'))
        self.assertLess(after.index('arm'), after.index('move'))
        first_move = next(c for c in link.calls[marks['stand']:] if c[0] == 'move')
        self.assertEqual(first_move[2], su.STEP_MS)
        self.assertEqual(json.loads((Path(self.home.name) / 'state.json').read_text())['design'],
                         'BIOMIMETIC')
        recorded = json.loads((Path(self.home.name) / 'leg_signatures.json').read_text())
        self.assertIn('BIOMIMETIC', recorded)
        self.assertEqual(link.calls[-1], ('torque', 0))

    def test_recording_needs_two_presses_on_the_same_design(self):
        from leg_detection import LegDetector, SignatureStore
        from startup_flow import StartupFlow
        from test_leg_detection import FakeRobot

        class Log:
            def __init__(self):
                self.lines = []

            def info(self, text):
                self.lines.append(text)
            warning = error = info

        robot, log = FakeRobot(HANGING, SQURO_WALLS), Log()
        flow = StartupFlow(LegDetector(robot, store=SignatureStore()), log)
        flow.begin()
        flow.handle([], robot.replies[:]); robot.replies.clear(); flow.update()
        flow.handle(key(pygame.K_RETURN), []); flow.update()
        while robot.replies:
            flow.handle([], [robot.replies.pop(0)]); flow.update()
        self.assertEqual(flow.stage, 'result')
        flow.handle(key(pygame.K_r), [])                 # Q8 highlighted: only asks
        flow.handle(key(pygame.K_DOWN), [])              # moving the highlight cancels
        flow.handle(key(pygame.K_r), [])                 # asks again, for Biomimetic
        self.assertEqual(SignatureStore().data, {})
        self.assertIn('R again', flow.label())
        flow.handle(key(pygame.K_r), [])
        self.assertEqual(list(SignatureStore().data), ['BIOMIMETIC'])

    def test_old_firmware_falls_back_to_a_manual_choice(self):
        link = FakeLink(HANGING, {}, silent=True)
        marks = self.run_gui(link, {
            # Telemetry times out after 1.5 s of real time.
            380: ('manual', key(pygame.K_m)),
            382: ('choose', key(pygame.K_RETURN)),
        }, frames=420)
        before = link.calls[:marks['choose']]
        self.assertFalse(any(c[0] in ('probe', 'move', 'arm') for c in before))
        self.assertNotIn(('torque', 1), before)
        # Q8 chosen: the original startup runs (torque, rest pose).
        self.assertIn(('torque', 1), link.calls[marks['choose']:])

    def test_escape_during_detection_stops_the_sweep_not_the_program(self):
        class Slow(FakeLink):
            def start_probe(self, excursions, pwm):
                self.calls.append(('probe', list(excursions), pwm))   # never answers

        link = Slow(HANGING, SQURO_WALLS)
        marks = self.run_gui(link, {
            5: ('enter', key(pygame.K_RETURN)),
            10: ('escape', key(pygame.K_ESCAPE)),
        }, frames=30)
        self.assertIn(('abort',), link.calls[marks['escape']:])
        # Still running afterwards: telemetry requested again.
        self.assertIn('telemetry', link.kinds()[marks['escape']:])

    def test_walking_keys_do_nothing_on_biomimetic_legs(self):
        link = FakeLink(HANGING, SQURO_WALLS)
        # Walk held for frames 5-12, then switch-gait, jump and greet pressed.
        held = lambda name, n: ((name == 'move' and 5 <= n <= 12) or
                                (name in ('switch_gait', 'jump', 'greet') and 14 <= n <= 16))
        self.run_gui(link, {}, frames=24, argv=('--legs', 'BIOMIMETIC'), held=held)
        self.assertFalse(any(c[0] in ('move', 'arm', 'send_jump') for c in link.calls), link.calls)
        self.assertNotIn(('torque', 1), link.calls)

    def test_q8_legs_still_walk(self):
        link = FakeLink(HANGING, {})
        held = lambda name, n: name == 'move' and 5 <= n <= 12
        with patch.object(InputHandler, 'get_movement_direction', return_value='f'):
            self.run_gui(link, {}, frames=16, argv=('--legs', 'Q8'), held=held)
        self.assertIn(('torque', 1), link.calls)
        gait_moves = [c for c in link.calls if c[0] == 'move' and c[2] == 0]
        self.assertTrue(gait_moves)


if __name__ == '__main__':
    unittest.main()
