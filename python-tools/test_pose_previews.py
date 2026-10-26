"""Hardware-free checks for all gait assets and GUI selection changes."""
import os
os.environ.setdefault('SDL_VIDEODRIVER', 'dummy')
# Keep remembered leg designs out of the real home directory.
import tempfile as _tempfile
os.environ.setdefault('Q8BOT_HOME', _tempfile.mkdtemp(prefix='q8bot-test-'))
os.environ.setdefault('SDL_AUDIODRIVER', 'dummy')
import hashlib
import math
import runpy
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / 'q8bot'))
import pygame
import eth_theme as eth
from control_panel import ControlPanel, WINDOW_SIZE
from gait_manager import GAITS, GaitManager
from input_handler import InputHandler
from kinematics_solver import k_solver
from leg_designs import selectable
from pose_preview import DESCRIPTIONS, PREVIEW_SIZE, generate_previews
from sitting_pose import SittingPose


class PreviewTests(unittest.TestCase):
    def setUp(self):
        pygame.init()

    def tearDown(self):
        pygame.quit()

    def test_every_gait_has_finite_eight_joint_trajectories(self):
        manager = GaitManager(k_solver())
        for name in GAITS:
            self.assertTrue(manager.load_gait(name), name)
            trajectories = manager.current_trajectories[name]
            self.assertIn('f', trajectories)
            for direction, frames in trajectories.items():
                self.assertTrue(frames, (name, direction))
                for frame in frames:
                    self.assertEqual(len(frame), 8)
                    self.assertTrue(all(math.isfinite(float(q)) for q in frame))

    def test_assets_complete_unique_and_reproducible(self):
        self.assertEqual(set(DESCRIPTIONS), set(GAITS) | {'SITTING', 'REARING'})
        designs = selectable()
        self.assertTrue(designs)
        hashes = set()
        for design in designs:
            with tempfile.TemporaryDirectory() as directory:
                generate_previews(directory, design)
                for name in DESCRIPTIONS:
                    path = ROOT / 'docs' / 'poses' / design.asset_dir / f'{name.lower()}.png'
                    actual = pygame.image.load(str(path))
                    generated = pygame.image.load(str(Path(directory) / path.name))
                    self.assertEqual(actual.get_size(), PREVIEW_SIZE, (design.key, name))
                    pixels = pygame.image.tobytes(actual, 'RGB')
                    self.assertEqual(pixels, pygame.image.tobytes(generated, 'RGB'),
                                     (design.key, name))
                    hashes.add(hashlib.sha256(pixels).hexdigest())
        # Every picture of every design must be distinct, so no two states can
        # be confused and no design silently reuses another's artwork.
        self.assertEqual(len(hashes), 10 * len(designs))

    def test_missing_or_corrupt_pictures_fall_back_to_rendering(self):
        with tempfile.TemporaryDirectory() as directory:
            (Path(directory) / 'sitting.png').write_bytes(b'broken picture')
            panel = ControlPanel(directory)
            self.assertEqual(set(panel.previews), set(DESCRIPTIONS))
            for picture in panel.previews.values():
                self.assertEqual(picture.get_size(), PREVIEW_SIZE)

    def test_all_previews_render_in_keyboard_and_joystick_layouts(self):
        pose = SittingPose(k_solver(), Mock())
        for joystick in (False, True):
            panel = ControlPanel(ROOT / 'docs' / 'poses', joystick, {'actions': {'battery': 3, 'reset': 2}})
            window = pygame.Surface(WINDOW_SIZE)
            for name in GAITS:
                pose.active = False
                window.fill(eth.BACKGROUND)
                panel.draw(window, name, pose)
                self.assertEqual(panel.active_preview, name)
                pose.active = True
                window.fill(eth.BACKGROUND)
                panel.draw(window, name, pose)
                self.assertEqual(panel.active_preview, 'SITTING')
            pygame.image.save(window, f'/tmp/q8-panel-{joystick}.png')

    def test_gui_cycles_all_gaits_and_changes_picture_on_p(self):
        robot = Mock()
        robot.move_all.return_value = True
        robot.serialHandler.in_waiting = 0
        frame = [0]
        seen = []
        pose = SittingPose(k_solver(), robot, lambda: frame[0] * .5)
        original_draw = ControlPanel.draw
        def draw(panel, window, gait, sitting, calibration=None, **extra):
            original_draw(panel, window, gait, sitting, calibration, **extra)
            if not seen or seen[-1] != panel.active_preview:
                seen.append(panel.active_preview)
        def events():
            frame[0] += 1
            n = frame[0]
            if n in (27, 29):
                return [pygame.event.Event(pygame.KEYDOWN, key=pygame.K_p)]
            if n in (28, 30):
                return [pygame.event.Event(pygame.KEYUP, key=pygame.K_p)]
            if n == 32:
                return [pygame.event.Event(pygame.QUIT)]
            return []
        def action(name):
            return name == 'switch_gait' and frame[0] in range(3, 25, 3)
        with patch('espnow.q8_espnow', return_value=robot), \
             patch('helpers.XiaoPortFinder.validate', return_value=True), \
             patch('sitting_pose.SittingPose', return_value=pose), \
             patch.object(ControlPanel, 'draw', draw), \
             patch.object(pygame.event, 'get', side_effect=events), \
             patch.object(InputHandler, 'is_movement_input', return_value=False), \
             patch.object(InputHandler, 'is_action_pressed', side_effect=action), \
             patch('time.sleep'), \
             patch.object(sys, 'argv', ['operate.py', 'mock-port', '--legs', 'Q8']):
            runpy.run_path(str(ROOT / 'q8bot' / 'operate.py'), run_name='__main__')
        self.assertEqual(seen, list(GAITS) + ['TROT', 'SITTING', 'TROT'])
        robot.disable_torque.assert_called_once()

    def test_battery_and_escape_work_while_sitting(self):
        robot = Mock()
        robot.move_all.return_value = True
        robot.serialHandler.in_waiting = 0
        frame = [0]
        def events():
            frame[0] += 1
            if frame[0] == 1:
                return [pygame.event.Event(pygame.KEYDOWN, key=pygame.K_p)]
            if frame[0] == 2:
                return [pygame.event.Event(pygame.KEYUP, key=pygame.K_p)]
            return [pygame.event.Event(pygame.KEYDOWN, key=pygame.K_ESCAPE)]
        def action(name):
            return name == 'battery' and frame[0] == 2
        with patch('espnow.q8_espnow', return_value=robot), \
             patch('helpers.XiaoPortFinder.validate', return_value=True), \
             patch.object(pygame.event, 'get', side_effect=events), \
             patch.object(InputHandler, 'is_movement_input', return_value=True), \
             patch.object(InputHandler, 'is_action_pressed', side_effect=action), \
             patch('time.sleep'), \
             patch.object(sys, 'argv', ['operate.py', 'mock-port', '--legs', 'Q8']):
            runpy.run_path(str(ROOT / 'q8bot' / 'operate.py'), run_name='__main__')
        self.assertEqual(robot.move_all.call_count, 1)  # Sit only; no walking.
        robot.check_battery.assert_called_once()
        robot.disable_torque.assert_called_once()


if __name__ == '__main__':
    unittest.main()
