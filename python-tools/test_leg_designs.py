"""Hardware-free checks for the leg designs, their solvers and calibration."""
import os
os.environ.setdefault('SDL_VIDEODRIVER', 'dummy')
# Keep remembered leg designs out of the real home directory.
import tempfile as _tempfile
os.environ.setdefault('Q8BOT_HOME', _tempfile.mkdtemp(prefix='q8bot-test-'))
os.environ.setdefault('SDL_AUDIODRIVER', 'dummy')
import math
import sys
import unittest
from pathlib import Path
from unittest.mock import Mock

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / 'q8bot'))
import pygame
from calibration_panel import CalibrationPanel
from control_panel import ControlPanel, WINDOW_SIZE
import espnow
from leg_calibration import CHECKING, CONFIRM, FAILED, HOLD, MOVING, PICK, LegCalibration
from leg_designs import BIOMIMETIC, FULL_RANGE, LEG_DESIGNS, Q8, deg_to_ticks
from sitting_pose import SittingPose
from squro_solver import MEASURED_PIVOTS, squro_solver


class SquroSolverTests(unittest.TestCase):
    """The solver must agree with the Fusion model it was derived from."""

    def setUp(self):
        self.solver = squro_solver()

    def test_mounting_pose_reproduces_the_measured_pivots(self):
        pivots = self.solver.pivots(*squro_solver.MOUNT_POSE)
        self.assertEqual(set(pivots), set(MEASURED_PIVOTS))
        for name, (mx, my) in MEASURED_PIVOTS.items():
            x, y = pivots[name]
            self.assertLess(math.hypot(x - mx, y - my), 0.005, name)

    def test_double_parallelogram_holds_across_the_workspace(self):
        # Both loops are what make the linkage equivalent to a 2R arm; if they
        # stop closing, the closed-form IK below is no longer valid.
        for q1 in range(30, 130, 10):
            for q2 in range(90, 190, 10):
                p = self.solver.pivots(q1, q2)
                d = lambda a, b: math.hypot(p[b][0] - p[a][0], p[b][1] - p[a][1])
                self.assertAlmostEqual(d('A', 'P1'), d('B', 'P2'), places=6)
                self.assertAlmostEqual(d('P1', 'P2'), d('A', 'B'), places=6)
                self.assertAlmostEqual(d('B', 'Q1'), d('E', 'Q2'), places=6)
                self.assertAlmostEqual(d('Q1', 'Q2'), d('B', 'E'), places=6)

    def test_humerus_pivots_stay_collinear(self):
        for q2 in range(90, 190, 10):
            p = self.solver.pivots(60, q2)
            a = math.atan2(p['P2'][1] - p['B'][1], p['P2'][0] - p['B'][0])
            b = math.atan2(p['E'][1] - p['B'][1], p['E'][0] - p['B'][0])
            self.assertLess(abs(math.degrees(a - b)), 1e-6)

    def test_ik_fk_round_trip(self):
        for x, y in [(33.935, 45.113), (9.75, 45.0), (9.75, 60.0), (0.0, 55.0), (20.0, 35.0)]:
            q1, q2, ok = self.solver.ik_solve(x, y)
            self.assertTrue(ok, (x, y))
            fx, fy = self.solver.fk_solve(q1, q2)
            self.assertAlmostEqual(fx, x, places=3)
            self.assertAlmostEqual(fy, y, places=3)

    def test_unreachable_targets_report_failure_and_hold_the_last_pose(self):
        good = self.solver.ik_solve(9.75, 45.0)[:2]
        for x, y in [(19.5, 2.0), (19.5, 200.0), (300.0, 300.0)]:
            q1, q2, ok = self.solver.ik_solve(x, y)
            self.assertFalse(ok, (x, y))
            # A refused target must not move the commanded angles. The cached
            # pose is unrounded, so compare at the solver's rounding tolerance.
            self.assertAlmostEqual(q1, good[0], places=3)
            self.assertAlmostEqual(q2, good[1], places=3)

    def test_reach_annulus_matches_the_link_lengths(self):
        self.assertTrue(self.solver.ik_check(19.5, abs(squro_solver.L1 - squro_solver.L2) + 0.1))
        self.assertFalse(self.solver.ik_check(19.5, abs(squro_solver.L1 - squro_solver.L2) - 0.1))
        self.assertTrue(self.solver.ik_check(19.5, squro_solver.L1 + squro_solver.L2 - 0.1))
        self.assertFalse(self.solver.ik_check(19.5, squro_solver.L1 + squro_solver.L2 + 0.1))


class LegDesignTests(unittest.TestCase):
    def test_q8_mounting_pose_matches_the_motor_config_firmware(self):
        # firmware/q8bot_motor_config drives to these ticks before leg install.
        self.assertEqual(Q8.mount_ticks(), (4618, 5622))

    def test_biomimetic_mounting_pose_is_the_cad_pose(self):
        self.assertEqual(BIOMIMETIC.mount_pose, squro_solver.MOUNT_POSE)
        self.assertEqual(BIOMIMETIC.mount_ticks(), (deg_to_ticks(52.660), deg_to_ticks(129.021)))

    def test_mount_positions_cover_four_legs(self):
        for design in (Q8, BIOMIMETIC):
            positions = design.mount_positions()
            self.assertEqual(len(positions), 8)
            # Left and right of each pair share a pose; the servos mirror it.
            self.assertEqual(positions[0:2], positions[2:4])
            self.assertEqual(positions[4:6], positions[6:8])
        # Q8 legs are one linkage; SQuRo hind legs are mounted at their own pose.
        self.assertEqual(Q8.mount_positions()[:2], Q8.mount_positions()[4:6])
        self.assertEqual(BIOMIMETIC.mount_positions()[4:6], [-41.286, 57.0])
        self.assertEqual(BIOMIMETIC.hind_mount_ticks(), (3627, 4745))

    def test_unavailable_design_has_no_solver(self):
        self.assertFalse(FULL_RANGE.available)
        with self.assertRaises(ValueError):
            FULL_RANGE.solver()

    def test_available_designs_build_a_solver_that_reaches_their_mount_pose(self):
        for design in (Q8, BIOMIMETIC):
            solver = design.solver()
            x, y = solver.fk_solve(*design.mount_pose)
            q1, q2, ok = solver.ik_solve(x, y)
            self.assertTrue(ok, design.key)
            self.assertAlmostEqual(float(q1), design.mount_pose[0], places=2)
            self.assertAlmostEqual(float(q2), design.mount_pose[1], places=2)


class MotionProfileTests(unittest.TestCase):
    """Each design's gait table must suit that design's own workspace."""

    # The five-bar's own gaits swing up to 74.2 deg from its mounting pose, so
    # that is the established ceiling any design must stay inside. The
    # biomimetic table was derived to a tighter 40 deg band.
    ESTABLISHED_CEILING = 75.0
    BIOMIMETIC_BAND = 40.0

    def test_every_design_generates_every_gait(self):
        from gait_manager import GaitManager
        for design in (Q8, BIOMIMETIC):
            solver = design.solver()
            manager = GaitManager(solver, design.gaits)
            for name in design.gaits:
                self.assertTrue(manager.load_gait(name), (design.key, name))
                for direction, frames in manager.current_trajectories[name].items():
                    self.assertTrue(frames, (design.key, name, direction))
                    for frame in frames:
                        self.assertEqual(len(frame), 8)
                        self.assertTrue(all(math.isfinite(float(q)) for q in frame))

    def test_no_gait_asks_for_an_unreachable_foot_position(self):
        import gait_generator as gg
        for design in (Q8, BIOMIMETIC):
            solver = design.solver()
            real = solver.ik_solve
            misses = []

            def traced(x, y, deg=True, rounding=3, _real=real, _m=misses):
                q1, q2, ok = _real(x, y, deg, rounding)
                if not ok:
                    _m.append((x, y))
                return q1, q2, ok

            solver.ik_solve = traced
            from gait_manager import GaitManager
            manager = GaitManager(solver, design.gaits)
            for name in design.gaits:
                manager.load_gait(name)
            self.assertEqual(misses, [], f'{design.key} left its workspace')

    def _deviation(self, design, name):
        import gait_generator as gg
        solver, mount = design.solver(), design.mount_pose
        _, x0, y0, xr, yr, yr2, s1, s2 = design.gaits[name]
        traj = gg._generate_base_trajectories(solver, x0, y0, xr, yr, yr2, s1, s2)
        self.assertIsNotNone(traj, (design.key, name))
        return max(max(abs(t[0] - mount[0]), abs(t[1] - mount[1])) for t in traj)

    def test_joints_stay_inside_the_established_ceiling(self):
        for design in (Q8, BIOMIMETIC):
            for name in design.gaits:
                dev = self._deviation(design, name)
                self.assertLessEqual(dev, self.ESTABLISHED_CEILING, (design.key, name, dev))

    def test_biomimetic_table_holds_its_tighter_band(self):
        for name in BIOMIMETIC.gaits:
            dev = self._deviation(BIOMIMETIC, name)
            self.assertLessEqual(dev, self.BIOMIMETIC_BAND, (name, dev))

    def test_biomimetic_is_better_centred_than_the_five_bar_baseline(self):
        worst = {d.key: max(self._deviation(d, n) for n in d.gaits) for d in (Q8, BIOMIMETIC)}
        self.assertLess(worst['BIOMIMETIC'], worst['Q8'], worst)

    def test_biomimetic_is_recentred_on_its_own_stance(self):
        # The five-bar stance centre would swing this leg far past its pose;
        # every biomimetic gait must sit on the design's natural stance.
        stance_x = BIOMIMETIC.natural_stance[0]
        for name, p in BIOMIMETIC.gaits.items():
            self.assertAlmostEqual(p[1], stance_x, places=3, msg=name)
        self.assertNotAlmostEqual(stance_x, Q8.natural_stance[0], places=1)

    def test_static_poses_are_reachable_for_every_design(self):
        for design in (Q8, BIOMIMETIC):
            solver = design.solver()
            foot_x, front, rear = design.sitting
            for height in (front, rear):
                self.assertTrue(solver.ik_solve(foot_x, height)[2], (design.key, height))
            for waypoint in design.rearing[:3]:
                self.assertEqual(len(waypoint), 8, design.key)
                self.assertTrue(all(math.isfinite(a) for a in waypoint), design.key)


class PerDesignPictureTests(unittest.TestCase):
    def setUp(self):
        pygame.init()

    def tearDown(self):
        pygame.quit()

    def test_designs_get_visibly_different_pictures(self):
        from pose_preview import DESCRIPTIONS, make_preview
        for name in DESCRIPTIONS:
            a = pygame.image.tobytes(make_preview(name, Q8), 'RGB')
            b = pygame.image.tobytes(make_preview(name, BIOMIMETIC), 'RGB')
            self.assertNotEqual(a, b, name)

    def test_control_panel_swaps_its_picture_set(self):
        panel = ControlPanel(ROOT / 'docs' / 'poses')
        self.assertEqual(panel.design_key, 'Q8')
        before = pygame.image.tobytes(panel.previews['TROT'], 'RGB')
        panel.set_design('BIOMIMETIC')
        self.assertEqual(panel.design_key, 'BIOMIMETIC')
        after = pygame.image.tobytes(panel.previews['TROT'], 'RGB')
        self.assertNotEqual(before, after)
        panel.set_design('Q8')
        self.assertEqual(pygame.image.tobytes(panel.previews['TROT'], 'RGB'), before)


class CalibrationTests(unittest.TestCase):
    def setUp(self):
        self.clock = [0.0]
        self.robot = Mock()
        self.robot.move_all.return_value = True
        self.cal = LegCalibration(self.robot, clock=lambda: self.clock[0])

    def test_full_sequence_commits_the_new_design(self):
        self.assertFalse(self.cal.active)
        self.cal.open_picker()
        self.assertEqual(self.cal.stage, PICK)
        self.cal.cursor = self.cal.ORDER.index('BIOMIMETIC')
        self.assertTrue(self.cal.choose())
        self.assertEqual(self.cal.stage, CONFIRM)
        self.assertTrue(self.cal.confirm())
        self.assertEqual(self.cal.stage, MOVING)
        self.robot.move_all.assert_called_once_with(BIOMIMETIC.mount_positions(),
                                                    LegCalibration.MOVE_MS, False)
        self.cal.update()
        self.assertEqual(self.cal.stage, MOVING)  # Still inside the move window.
        self.clock[0] += LegCalibration.MOVE_MS / 1000
        self.cal.update()
        self.assertEqual(self.cal.stage, CHECKING)
        self.robot.request_telemetry.assert_called_once()
        self.assertFalse(self.cal.finish())       # not before every joint is confirmed
        self.cal.feed(self.telemetry(BIOMIMETIC.mount_positions()))
        self.assertEqual(self.cal.stage, HOLD)
        self.assertTrue(self.cal.finish())
        self.assertFalse(self.cal.active)
        self.assertEqual(self.cal.fitted_key, 'BIOMIMETIC')

    def telemetry(self, angles, **flags):
        from test_leg_detection import telemetry_values
        return espnow.decode_reply(telemetry_values(list(angles) + [0.0], **flags))

    def moved_and_checked(self, reply):
        self.cal.open_picker()
        self.cal.cursor = self.cal.ORDER.index('BIOMIMETIC')
        self.cal.choose()
        self.cal.confirm()
        self.clock[0] += LegCalibration.MOVE_MS / 1000
        self.cal.update()
        if reply is not None:
            self.cal.feed(reply)

    def test_a_joint_short_of_the_mounting_pose_fails_with_its_name(self):
        short = BIOMIMETIC.mount_positions()
        short[6] -= 30.0                          # BR elbow stuck
        self.moved_and_checked(self.telemetry(short))
        self.assertEqual(self.cal.stage, FAILED)
        self.assertEqual(self.cal.diagnosis.category, 'torque')
        self.assertIn('BR elbow (ID 17) is 30 deg short', self.cal.diagnosis.detail)
        self.assertIn('Not enough torque', self.cal.label())
        self.assertEqual(self.cal.reports()[0], 'Calibration stopped: Mounting pose not reached.')
        self.assertTrue(self.cal.confirm())       # retry from FAILED

    def test_a_silent_servo_or_robot_fails_as_communication(self):
        self.moved_and_checked(self.telemetry(BIOMIMETIC.mount_positions(), missing={11}))
        self.assertEqual(self.cal.diagnosis.category, 'communication')
        self.assertIn('FL elbow (ID 11)', self.cal.diagnosis.detail)
        self.cal.cancel()
        self.moved_and_checked(None)
        self.clock[0] += LegCalibration.REPLY_TIMEOUT + 0.1
        self.cal.update()
        self.assertEqual(self.cal.stage, FAILED)
        self.assertIn('did not answer', self.cal.diagnosis.detail)

    def test_unavailable_design_is_refused_and_keeps_the_picker_open(self):
        self.cal.open_picker()
        self.cal.cursor = self.cal.ORDER.index('FULL_RANGE')
        self.assertFalse(self.cal.choose())
        self.assertEqual(self.cal.stage, PICK)
        self.assertIn('Full range', self.cal.error)
        self.robot.move_all.assert_not_called()

    def test_failed_send_stays_recoverable(self):
        self.robot.move_all.return_value = False
        self.cal.open_picker()
        self.cal.cursor = self.cal.ORDER.index('BIOMIMETIC')
        self.cal.choose()
        self.assertFalse(self.cal.confirm())
        self.assertEqual(self.cal.stage, FAILED)
        self.robot.move_all.return_value = True
        self.assertTrue(self.cal.confirm())
        self.assertEqual(self.cal.stage, MOVING)

    def test_cancel_leaves_the_fitted_design_untouched(self):
        self.cal.open_picker()
        self.cal.cursor = self.cal.ORDER.index('BIOMIMETIC')
        self.cal.choose()
        self.cal.confirm()
        self.assertTrue(self.cal.cancel())
        self.assertFalse(self.cal.active)
        self.assertEqual(self.cal.fitted_key, 'Q8')

    def test_cursor_wraps_and_blocks_movement_only_while_open(self):
        self.assertFalse(self.cal.blocks_movement())
        self.cal.open_picker()
        self.assertTrue(self.cal.blocks_movement())
        self.cal.cursor = len(self.cal.ORDER) - 1
        self.cal.move_cursor(1)
        self.assertEqual(self.cal.cursor, 0)
        self.cal.move_cursor(-1)
        self.assertEqual(self.cal.cursor, len(self.cal.ORDER) - 1)

    def test_finish_only_from_hold(self):
        self.cal.open_picker()
        self.assertFalse(self.cal.finish())
        self.assertEqual(self.cal.fitted_key, 'Q8')


class CalibrationRenderTests(unittest.TestCase):
    def setUp(self):
        pygame.init()

    def tearDown(self):
        pygame.quit()

    def test_every_stage_renders_and_labels(self):
        robot = Mock()
        robot.move_all.return_value = True
        cal = LegCalibration(robot)
        view = CalibrationPanel()
        cal.open_picker()
        stages = []
        for step in ('pick', 'choose', 'confirm', 'hold'):
            if step == 'choose':
                cal.cursor = cal.ORDER.index('BIOMIMETIC')
                cal.choose()
            elif step == 'confirm':
                cal.confirm()
            elif step == 'hold':
                cal.stage = HOLD
            surface = view.render(cal)
            self.assertEqual(surface.get_size(), (800, 480))
            self.assertTrue(cal.label())
            stages.append(cal.stage)
        self.assertEqual(stages, [PICK, CONFIRM, MOVING, HOLD])

    def test_picker_renders_for_every_design_under_the_cursor(self):
        view = CalibrationPanel()
        cal = LegCalibration(Mock())
        cal.open_picker()
        for index in range(len(LEG_DESIGNS)):
            cal.cursor = index
            self.assertEqual(view.render(cal).get_size(), (800, 480))

    def test_control_panel_switches_to_calibration_and_back(self):
        panel = ControlPanel(ROOT / 'docs' / 'poses')
        window = pygame.Surface(WINDOW_SIZE)
        pose = SittingPose(Mock(), Mock())
        cal = LegCalibration(Mock())
        panel.draw(window, 'TROT', pose, cal)
        self.assertEqual(panel.active_preview, 'TROT')  # Inactive: normal view.
        cal.open_picker()
        panel.draw(window, 'TROT', pose, cal)
        self.assertEqual(panel.active_preview, 'CALIBRATION')
        cal.cancel()
        panel.draw(window, 'TROT', pose, cal)
        self.assertEqual(panel.active_preview, 'TROT')


class OperateIntegrationTests(unittest.TestCase):
    """The L key and Esc handling, exercised through operate.py itself."""

    def setUp(self):
        pygame.init()

    def tearDown(self):
        pygame.quit()

    def test_l_opens_picker_and_escape_cancels_without_quitting(self):
        import runpy
        from unittest.mock import patch
        from control_panel import ControlPanel
        from input_handler import InputHandler

        robot = Mock()
        robot.move_all.return_value = True
        robot.serialHandler.in_waiting = 0
        frame = [0]
        seen = []
        original_draw = ControlPanel.draw

        def draw(panel, window, gait, pose, calibration=None, **extra):
            original_draw(panel, window, gait, pose, calibration, **extra)
            seen.append((frame[0], calibration.stage if calibration else None))

        def events():
            frame[0] += 1
            n = frame[0]
            if n == 2:
                return [pygame.event.Event(pygame.KEYDOWN, key=pygame.K_l)]
            if n == 3:
                return [pygame.event.Event(pygame.KEYUP, key=pygame.K_l)]
            if n in (5, 7):
                return [pygame.event.Event(pygame.KEYDOWN, key=pygame.K_ESCAPE)]
            return []

        with patch('espnow.q8_espnow', return_value=robot), \
             patch('helpers.XiaoPortFinder.validate', return_value=True), \
             patch.object(ControlPanel, 'draw', draw), \
             patch.object(pygame.event, 'get', side_effect=events), \
             patch.object(InputHandler, 'is_movement_input', return_value=False), \
             patch.object(InputHandler, 'is_action_pressed', return_value=False), \
             patch('time.sleep'), \
             patch.object(sys, 'argv', ['operate.py', 'mock-port', '--legs', 'Q8']):
            runpy.run_path(str(ROOT / 'q8bot' / 'operate.py'), run_name='__main__')

        stages = dict(seen)
        self.assertEqual(stages.get(2), PICK, 'L should open the picker')
        self.assertEqual(stages.get(4), PICK, 'picker should stay open')
        # The first Esc cancels calibration; it must not quit the program.
        self.assertIsNone(stages.get(5), 'Esc should close the picker')
        self.assertIn(6, stages, 'the loop must survive the cancelling Esc')
        # The second Esc, with no calibration open, exits.
        self.assertNotIn(8, stages, 'the second Esc should quit')
        robot.disable_torque.assert_called_once()


if __name__ == '__main__':
    unittest.main()
