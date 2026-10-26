"""Hardware-free rearing sequence, geometry, failure and GUI regressions."""
import os
os.environ.setdefault('SDL_VIDEODRIVER', 'dummy')
# Keep remembered leg designs out of the real home directory.
import tempfile as _tempfile
os.environ.setdefault('Q8BOT_HOME', _tempfile.mkdtemp(prefix='q8bot-test-'))
os.environ.setdefault('SDL_AUDIODRIVER', 'dummy')
import math
from pathlib import Path
import runpy
import sys
import unittest
from unittest.mock import Mock, patch
ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / 'q8bot'))
import pygame
from control_panel import ControlPanel
from gait_manager import GAITS
from input_handler import InputHandler
from kinematics_solver import k_solver
from rearing_pose import RearingPose
from routine_generator import G1, G2, G3


class RearingTests(unittest.TestCase):
    def setUp(self):
        self.now = 0.0
        self.leg = k_solver()
        self.robot = Mock()
        self.robot.move_all.return_value = True
        self.pose = RearingPose(self.leg, self.robot, lambda: self.now)

    def advance(self):
        self.now = self.pose.transition_until
        return self.pose.update()

    def test_static_hold_and_return_to_every_gait(self):
        for name, params in GAITS.items():
            self.robot.reset_mock()
            self.assertTrue(self.pose.toggle(params[1], params[2]))
            self.assertEqual(self.robot.move_all.call_args.args[0], G1)
            self.advance()
            self.assertEqual(self.robot.move_all.call_args.args[0], G2)
            self.advance()
            self.assertEqual(self.robot.move_all.call_args.args[0], [-45, 45, -45, 45, 37, 62, 37, 62])
            self.now += 30
            self.pose.update()
            self.assertEqual(self.robot.move_all.call_count, 3)  # Hold; no waving or repeated sends.
            self.assertTrue(self.pose.blocks_movement())
            self.assertTrue(self.pose.toggle(params[1], params[2]))
            front_down = self.robot.move_all.call_args.args[0]
            self.assertEqual(front_down[4:], self.pose.target_positions()[4:])
            front_x, front_y = self.leg.fk_solve(*front_down[:2])
            self.assertAlmostEqual(front_x, 9.75, places=2)
            self.assertAlmostEqual(front_y, 60, places=2)
            self.advance()
            expected = list(self.leg.ik_solve(params[1], params[2])[:2]) * 4
            rear_down = self.robot.move_all.call_args.args[0]
            self.assertEqual(rear_down[:4], front_down[:4])
            self.assertEqual(rear_down[4:], expected[4:])
            self.advance()
            self.assertEqual(self.robot.move_all.call_args.args[0], expected, name)
            self.assertTrue(self.pose.blocks_movement())
            self.advance()
            self.assertFalse(self.pose.blocks_movement())

    def test_rearward_offset_and_interpolated_linkage_closure(self):
        target = self.pose.target_positions()
        self.assertEqual(target[:4], G3[:4])
        self.assertEqual(target[4:6], target[6:])
        old_x, _ = self.leg.fk_solve(*G3[4:6])
        new_x, _ = self.leg.fk_solve(*target[4:6])
        self.assertGreater(new_x, old_x + 5)
        for params in GAITS.values():
            rest = list(self.leg.ik_solve(params[1], params[2])[:2]) * 4
            stages = [rest, G1, G2, target] + [p for p, _, _ in self.pose.lowering_steps(params[1], params[2])]
            for start, end in zip(stages, stages[1:]):
                for step in range(101):
                    positions = [float(a + (b-a) * step / 100) for a, b in zip(start, end)]
                    self.pose._validate_steps([(positions, 1000, 1)])

    def test_failed_initial_send_and_unreachable_return_preserve_state(self):
        self.robot.move_all.return_value = False
        self.assertFalse(self.pose.toggle(9.75, 43.36))
        self.assertFalse(self.pose.active)
        self.robot.move_all.return_value = True
        self.pose.toggle(9.75, 43.36)
        self.robot.reset_mock()
        with self.assertRaises(ValueError):
            self.pose.toggle(9.75, 1000)
        self.assertTrue(self.pose.active)
        self.robot.move_all.assert_not_called()

    def test_lowering_waits_for_front_support_before_moving_rear(self):
        self.pose.toggle(9.75, 43.36)
        self.advance()
        self.advance()
        self.advance()
        self.robot.reset_mock()
        started = self.now
        self.pose.toggle(9.75, 43.36)
        self.assertIn('Extending front', self.pose.label())
        self.assertEqual(self.robot.move_all.call_args.args[1], 2000)
        self.now += 2.19
        self.pose.update()
        self.assertEqual(self.robot.move_all.call_count, 1)
        self.advance()
        self.assertIn('Lowering rear', self.pose.label())
        self.assertEqual(self.robot.move_all.call_args.args[1], 2500)
        self.now += 2.69
        self.pose.update()
        self.assertEqual(self.robot.move_all.call_count, 2)
        self.advance()
        self.assertIn('Settling', self.pose.label())
        self.assertEqual(self.robot.move_all.call_args.args[1], 1500)
        self.advance()
        self.assertAlmostEqual(self.now - started, 6.4)
        self.assertFalse(self.pose.blocks_movement())

    def test_retry_does_not_restore_old_rearing_targets(self):
        self.pose.toggle(9.75, 43.36)
        self.advance()
        self.advance()
        self.pose.toggle(9.75, 43.36)
        self.advance()  # Rear legs have been commanded toward the gait position.
        current_rear = self.robot.move_all.call_args.args[0][4:]
        self.robot.move_all.return_value = False
        self.advance()  # Final settling command fails.
        self.robot.move_all.return_value = True
        self.assertTrue(self.pose.retry_lowering(9.75, 43.36))
        self.assertEqual(self.robot.move_all.call_args.args[0][4:], current_rear)

    def test_failed_waypoint_pauses_then_retries_lowering(self):
        self.pose.toggle(9.75, 43.36)
        self.robot.move_all.return_value = False
        self.assertIn('failed', self.advance())
        self.assertTrue(self.pose.failed)
        count = self.robot.move_all.call_count
        self.now += 20
        self.pose.update()
        self.assertEqual(self.robot.move_all.call_count, count)
        self.assertTrue(self.pose.blocks_movement())
        self.robot.move_all.return_value = True
        self.assertTrue(self.pose.retry_lowering(9.75, 43.36))
        self.assertEqual(self.robot.move_all.call_args.args[0][4:], G1[4:])
        self.advance()
        self.advance()
        self.advance()
        self.assertFalse(self.pose.blocks_movement())

    def test_interrupt_entry_and_recover_failed_exit(self):
        self.pose.toggle(9.75, 43.36)
        self.pose.toggle(9.75, 43.36)
        self.robot.move_all.return_value = False
        self.advance()
        self.assertFalse(self.pose.active)
        self.assertTrue(self.pose.failed)
        self.assertFalse(self.pose.retry_lowering(9.75, 43.36))
        self.assertFalse(self.pose.active)
        self.robot.move_all.return_value = True
        self.assertTrue(self.pose.retry_lowering(9.75, 43.36))
        self.advance()
        self.advance()
        self.advance()
        self.assertFalse(self.pose.blocks_movement())

    def test_u_held_and_joystick_keyboard_toggle(self):
        handler = InputHandler(use_joystick=True)
        down = pygame.event.Event(pygame.KEYDOWN, key=pygame.K_u)
        up = pygame.event.Event(pygame.KEYUP, key=pygame.K_u)
        self.assertTrue(handler.keyboard_action_pressed_once('rear', [down]))
        self.assertFalse(handler.keyboard_action_pressed_once('rear', [down]))
        self.assertFalse(handler.keyboard_action_pressed_once('rear', [up]))
        self.assertTrue(handler.keyboard_action_pressed_once('rear', [down]))

    def test_escape_during_entry_sends_no_later_waypoints(self):
        self.robot.serialHandler.in_waiting = 0
        events = [[pygame.event.Event(pygame.KEYDOWN, key=pygame.K_u)],
                  [pygame.event.Event(pygame.KEYDOWN, key=pygame.K_ESCAPE)]]
        with patch('espnow.q8_espnow', return_value=self.robot), \
             patch('helpers.XiaoPortFinder.validate', return_value=True), \
             patch.object(pygame.event, 'get', side_effect=events), \
             patch.object(InputHandler, 'is_action_pressed', return_value=False), \
             patch('time.sleep'), \
             patch.object(sys, 'argv', ['operate.py', 'mock-port', '--legs', 'Q8']):
            runpy.run_path(str(ROOT / 'q8bot' / 'operate.py'), run_name='__main__')
        self.assertEqual(self.robot.move_all.call_count, 1)
        self.robot.disable_torque.assert_called_once()

    def test_gui_u_sequence_blocks_p_and_walking_but_allows_battery(self):
        self.robot.serialHandler.in_waiting = 0
        frame, pictures, counts = [0], [], {}
        original_draw = ControlPanel.draw
        def draw(panel, window, gait, pose, calibration=None, **extra):
            original_draw(panel, window, gait, pose, calibration, **extra)
            pictures.append(panel.active_preview)
            if frame[0] == 8:
                pygame.image.save(window, '/tmp/q8-rearing-gui.png')
        def events():
            frame[0] += 1
            n = frame[0]
            self.now = n * .6
            counts[n] = self.robot.move_all.call_count
            if n in (1, 2, 12):
                return [pygame.event.Event(pygame.KEYDOWN, key=pygame.K_u)]
            if n in (3, 13):
                return [pygame.event.Event(pygame.KEYUP, key=pygame.K_u)]
            if n == 9:
                return [pygame.event.Event(pygame.KEYDOWN, key=pygame.K_p)]
            if n == 10:
                return [pygame.event.Event(pygame.KEYUP, key=pygame.K_p)]
            if n == 24:
                return [pygame.event.Event(pygame.KEYDOWN, key=pygame.K_ESCAPE)]
            return []
        def action(name):
            return name == 'battery' and frame[0] == 10
        with patch('espnow.q8_espnow', return_value=self.robot), \
             patch('helpers.XiaoPortFinder.validate', return_value=True), \
             patch('rearing_pose.RearingPose', return_value=self.pose), \
             patch.object(ControlPanel, 'draw', draw), \
             patch.object(pygame.event, 'get', side_effect=events), \
             patch.object(InputHandler, 'is_movement_input', return_value=True), \
             patch.object(InputHandler, 'is_action_pressed', side_effect=action), \
             patch('time.sleep'), \
             patch.object(sys, 'argv', ['operate.py', 'mock-port', '--legs', 'Q8']):
            runpy.run_path(str(ROOT / 'q8bot' / 'operate.py'), run_name='__main__')
        self.assertEqual(counts[8], 3)
        self.assertEqual(counts[12], 3)
        self.assertEqual(self.robot.move_all.call_count, 6)
        self.assertEqual(pictures[:11], ['REARING'] * 11)
        self.assertEqual(pictures[11:], ['TROT'] * 12)
        self.robot.check_battery.assert_called_once()
        self.robot.disable_torque.assert_called_once()


if __name__ == '__main__':
    unittest.main()
