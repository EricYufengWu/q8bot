"""Run: venv/bin/python -m unittest test_sitting_pose (from python-tools)."""
import os
os.environ.setdefault('SDL_VIDEODRIVER', 'dummy')
# Keep remembered leg designs out of the real home directory.
import tempfile as _tempfile
os.environ.setdefault('Q8BOT_HOME', _tempfile.mkdtemp(prefix='q8bot-test-'))
os.environ.setdefault('SDL_AUDIODRIVER', 'dummy')
import runpy
import sys
import unittest
from pathlib import Path
from unittest.mock import Mock, patch
ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / 'q8bot'))
import pygame
from gait_manager import GAITS
from input_handler import InputHandler
from kinematics_solver import k_solver
from sitting_pose import SittingPose

class SittingPoseTests(unittest.TestCase):
    def setUp(self):
        self.now = 0.0
        self.leg = k_solver()
        self.robot = Mock()
        self.robot.move_all.return_value = True
        self.pose = SittingPose(self.leg, self.robot, lambda: self.now)

    def test_geometry_and_return_to_every_gait(self):
        for params in GAITS.values():
            self.assertTrue(self.pose.toggle(params[1], params[2]))
            positions, duration, recording = self.robot.move_all.call_args.args
            self.assertEqual(duration, 1000)
            self.assertFalse(recording)
            for i, height in ((0, 60), (2, 60), (4, 20), (6, 20)):
                x, y = self.leg.fk_solve(*positions[i:i+2])
                self.assertAlmostEqual(x, 9.75, places=2)
                self.assertAlmostEqual(y, height, places=2)
            self.now += 10
            self.assertTrue(self.pose.blocks_movement())
            self.assertTrue(self.pose.toggle(params[1], params[2]))
            positions = self.robot.move_all.call_args.args[0]
            for i in range(0, 8, 2):
                x, y = self.leg.fk_solve(*positions[i:i+2])
                self.assertAlmostEqual(x, params[1], places=2)
                self.assertAlmostEqual(y, params[2], places=2)
            self.assertTrue(self.pose.blocks_movement())
            self.now += 1
            self.assertFalse(self.pose.blocks_movement())

    def test_failed_send_or_ik_preserves_state(self):
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

    def test_one_toggle_per_press_even_with_joystick(self):
        handler = InputHandler(use_joystick=True)
        down = pygame.event.Event(pygame.KEYDOWN, key=pygame.K_p)
        up = pygame.event.Event(pygame.KEYUP, key=pygame.K_p)
        repeat = pygame.event.Event(pygame.KEYDOWN, key=pygame.K_p, repeat=True)
        self.assertTrue(handler.keyboard_action_pressed_once('sit', [down]))
        for events in ([], [down], [repeat], [up]):
            self.assertFalse(handler.keyboard_action_pressed_once('sit', events))
        self.assertTrue(handler.keyboard_action_pressed_once('sit', [down]))

    def test_gui_holds_pose_and_waits_for_standing(self):
        robot = Mock()
        robot.move_all.return_value = True
        robot.serialHandler.in_waiting = 0
        now, frame, calls = [0.0], [0], {}
        pose = SittingPose(self.leg, robot, lambda: now[0])
        def events():
            frame[0] += 1
            n = frame[0]
            now[0] = n * .5
            calls[n] = robot.move_all.call_count
            if n == 3:
                pygame.image.save(pygame.display.get_surface(), '/tmp/q8-sitting-gui.png')
            if n in (1, 4):
                return [pygame.event.Event(pygame.KEYDOWN, key=pygame.K_p)]
            if n in (3, 5):
                return [pygame.event.Event(pygame.KEYUP, key=pygame.K_p)]
            if n == 9:
                return [pygame.event.Event(pygame.QUIT)]
            return []
        with patch('espnow.q8_espnow', return_value=robot), \
             patch('helpers.XiaoPortFinder.validate', return_value=True), \
             patch('sitting_pose.SittingPose', return_value=pose), \
             patch.object(pygame.event, 'get', side_effect=events), \
             patch.object(InputHandler, 'is_movement_input', return_value=True), \
             patch.object(InputHandler, 'get_movement_direction', return_value='f'), \
             patch.object(InputHandler, 'is_action_pressed', return_value=False), \
             patch('time.sleep'), \
             patch.object(sys, 'argv', ['operate.py', 'mock-port', '--legs', 'Q8']):
            runpy.run_path(str(ROOT / 'q8bot' / 'operate.py'), run_name='__main__')
        self.assertEqual(calls[2], 1)
        self.assertEqual(calls[4], 1)
        self.assertEqual(calls[5], 2)
        self.assertEqual(calls[7], 2)
        self.assertGreater(calls[9], 2)
        robot.disable_torque.assert_called_once()

if __name__ == '__main__':
    unittest.main()
