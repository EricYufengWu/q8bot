"""Head yaw control and the biomimetic test trot, without hardware."""
import os
os.environ.setdefault('SDL_VIDEODRIVER', 'dummy')
os.environ.setdefault('SDL_AUDIODRIVER', 'dummy')
import json
import math
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
import head_control as hc
import squro_gait as sg
from input_handler import InputHandler
from leg_designs import BIOMIMETIC
from leg_detection import near_change_point
from test_leg_detection import HANGING, SQURO_WALLS
from test_startup_gui import FakeLink, key
from test_stand_up import RESTING


def peak_lift(profile):
    """The lift at the highest swing frame: an odd frame count straddles the sine's peak."""
    return profile.lift * max(math.sin((i + 1) * math.pi / profile.swing) for i in range(profile.swing))


class Clock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now


class Recorder:
    def __init__(self):
        self.heads = []

    def move_head(self, deg, ms):
        self.heads.append((deg, ms))


class Home(unittest.TestCase):
    def setUp(self):
        self.home = tempfile.TemporaryDirectory()
        self.patch_home = patch.dict(os.environ, {'Q8BOT_HOME': self.home.name})
        self.patch_home.start()

    def tearDown(self):
        self.patch_home.stop()
        self.home.cleanup()


class HeadControlTests(Home):
    def test_unmeasured_limits_are_narrow_and_clamp(self):
        clock, robot = Clock(), Recorder()
        head = hc.HeadControl(robot, clock=clock)
        self.assertEqual(head.limits, hc.DEFAULT_LIMITS)
        self.assertFalse(head.measured)
        for _ in range(100):                   # 5 s of holding left
            clock.now += 0.05
            head.update(-1)
        self.assertEqual(head.angle, hc.DEFAULT_LIMITS[1])
        self.assertTrue(all(hc.DEFAULT_LIMITS[0] <= d <= hc.DEFAULT_LIMITS[1] for d, _ in robot.heads))
        self.assertIn('not measured', head.label())

    def test_commands_are_rate_limited_and_timed(self):
        clock, robot = Clock(), Recorder()
        head = hc.HeadControl(robot, clock=clock)
        for _ in range(200):                   # 1 s of frames at 200 Hz
            clock.now += 0.005
            head.update(+1)
        self.assertLessEqual(len(robot.heads), 21)
        self.assertTrue(all(ms == hc.MOVE_MS for _, ms in robot.heads))
        # Nothing is resent while the head is still.
        sent = len(robot.heads)
        for _ in range(50):
            clock.now += 0.05
            head.update(0)
        self.assertLessEqual(len(robot.heads), sent + 1)

    def test_measured_limits_centre_and_swap_persist(self):
        clock, robot = Clock(), Recorder()
        head = hc.HeadControl(robot, clock=clock)
        head.set_limits((-24.0, 33.0))
        head.swap_direction()
        again = hc.HeadControl(Recorder(), clock=clock)
        self.assertEqual(again.limits, (-24.0, 33.0))
        self.assertTrue(again.measured and again.reversed)
        again.set_present(80.0)
        self.assertEqual(again.angle, 33.0)          # a reading outside is pulled in
        clock.now += 1.0
        again.update(0, centre=True)
        self.assertEqual(again.angle, 0.0)

    def test_swapped_sense_turns_the_other_way(self):
        clock, robot = Clock(), Recorder()
        head = hc.HeadControl(robot, clock=clock)
        clock.now = 1.0
        head.update(-1)
        clock.now = 1.1
        head.update(-1)
        left = head.angle
        head.swap_direction()
        head.set_present(0.0)
        clock.now = 2.0
        head.update(-1)
        clock.now = 2.1
        head.update(-1)
        self.assertGreater(left, 0)
        self.assertLess(head.angle, 0)

    def test_nothing_is_sent_without_a_key(self):
        clock, robot = Clock(), Recorder()
        head = hc.HeadControl(robot, clock=clock)
        head.set_present(-1.3)
        head.set_limits((-5.0, 30.0))           # even one that moves the target inward
        head.set_present(-9.0)
        for _ in range(40):
            clock.now += 0.05
            head.update(0)
        self.assertEqual(robot.heads, [])
        head.update(0, centre=True)
        self.assertEqual(robot.heads, [(0.0, hc.MOVE_MS)])


class TrotTests(unittest.TestCase):
    def test_every_frame_of_every_gait_keeps_every_margin(self):
        solvers = BIOMIMETIC.leg_solvers()
        for profile in sg.PROFILES:
            trajectories = sg.build_trot(BIOMIMETIC, profile)
            self.assertEqual(set(trajectories), set(sg.DIRECTIONS))
            for direction, frames in trajectories.items():
                self.assertEqual(len(frames), profile.swing + profile.stance)
                m = sg.margins(BIOMIMETIC, frames, profile.frame_ms, profile.depth)
                label = (profile.key, direction)
                self.assertGreaterEqual(m['gap'], sg.MIN_LEG_GAP, label)
                self.assertGreaterEqual(m['change'], sg.CHANGE_POINT_MARGIN, label)
                self.assertGreaterEqual(m['fold'], sg.MIN_FOLD, label)
                self.assertGreaterEqual(m['clearance'], sg.MIN_CLEARANCE, label)
                self.assertLessEqual(m['speed'], sg.MAX_JOINT_SPEED, label)
                # Independently of margins(): stricter than detection's 15 deg.
                for frame in frames:
                    for leg, solver in enumerate(solvers):
                        q1, q2 = frame[leg * 2: leg * 2 + 2]
                        self.assertFalse(near_change_point(solver, q1, q2), label)

    def test_the_fast_gait_is_much_longer_and_faster(self):
        self.assertGreaterEqual(sg.FAST.stride, 2 * sg.SLOW.stride)
        self.assertGreaterEqual(sg.FAST.speed, 5 * sg.SLOW.speed)
        self.assertLess(sg.FAST.cycle_s, sg.SLOW.cycle_s / 2)

    def test_both_gaits_walk_where_the_robot_stands(self):
        standing = sg.standing_positions(BIOMIMETIC)
        for profile in sg.PROFILES:
            self.assertEqual(profile.depth, sg.STAND_DEPTH)          # never lower than standing
            for leg, solver in enumerate(BIOMIMETIC.leg_solvers()):
                foot = solver.pivots(*standing[leg * 2: leg * 2 + 2])['F']
                centre = sg.walking_stance(solver, leg, profile)
                self.assertAlmostEqual(foot[0], centre[0], delta=0.05)   # no slide into the trot
                self.assertAlmostEqual(foot[1], centre[1], delta=0.05)

    def test_the_fast_gait_lifts_high_and_walks_tall(self):
        self.assertGreaterEqual(sg.FAST.lift, 20.0)
        self.assertGreaterEqual(sg.FAST.speed, 140.0)
        self.assertGreaterEqual(sg.FAST.depth, sg.STANCE_DEPTH)   # no lower than the CAD stance
        self.assertGreaterEqual(sg.MIN_CLEARANCE, 20.0)           # elbows never carry the robot
        front = BIOMIMETIC.solver()
        path = sg.foot_path(sg.walking_stance(front, 0, sg.FAST), +1, sg.FAST)
        # The highest foot point clears the ground by the full lift.
        lowest_ground = sg.FAST.depth - front.contact_radius
        self.assertAlmostEqual(lowest_ground - min(y for _, y in path), peak_lift(sg.FAST), delta=0.01)

    def test_unsafe_gaits_are_refused_with_a_reason(self):
        crowded = sg.GaitProfile('X', 'crowded', stride=56, lift=12, swing=12, stance=24,
                                 frame_ms=25, ease_ms=500, reach_front=0, reach_hind=0)
        with self.assertRaisesRegex(ValueError, 'front and hind legs come within'):
            sg.build_trot(BIOMIMETIC, crowded)
        overreach = sg.GaitProfile('Y', 'overreach', stride=38, lift=12, swing=12, stance=24,
                                   frame_ms=20, ease_ms=500, reach_front=34, reach_hind=14)
        with self.assertRaisesRegex(ValueError, 'change point|reach'):
            sg.build_trot(BIOMIMETIC, overreach)
        hasty = sg.GaitProfile('Z', 'hasty', stride=38, lift=20, swing=4, stance=8,
                               frame_ms=20, ease_ms=500, reach_front=16, reach_hind=14)
        with self.assertRaisesRegex(ValueError, 'deg/s'):
            sg.build_trot(BIOMIMETIC, hasty)

    def test_closest_front_and_hind_parts_have_room_even_standing(self):
        solvers = BIOMIMETIC.leg_solvers()
        for home in (sg.standing_positions(BIOMIMETIC), BIOMIMETIC.mount_positions()):
            pivots = [s.pivots(*home[i * 2: i * 2 + 2]) for i, s in enumerate(solvers)]
            self.assertGreater(sg.leg_gap(pivots[0], pivots[2], solvers[0].contact_radius), sg.MIN_LEG_GAP)
            m = sg.margins(BIOMIMETIC, [home], 1000, sg.STAND_DEPTH)
            self.assertGreaterEqual(m['change'], sg.CHANGE_POINT_MARGIN)
            self.assertGreaterEqual(m['fold'], sg.MIN_FOLD)

    def test_direction_changes_wait_for_the_feet_to_meet(self):
        for profile in sg.PROFILES:
            trajectories = sg.build_trot(BIOMIMETIC, profile)
            n = len(trajectories['f'])
            for pair, frames in sg.direction_switches(trajectories).items():
                ordered = sorted(frames)
                waits = [(b - a) % n or n for a, b in zip(ordered, ordered[1:] + ordered[:1])]
                self.assertLessEqual(max(waits), n // 2, (profile.key, pair))   # at most half a cycle

    def test_turning_while_walking_never_jumps(self):
        clock = Clock()
        trot = sg.SquroTrot(BIOMIMETIC, clock=clock)
        for profile in sg.PROFILES:
            frames = trot.trajectories[profile.key]
            n = len(frames['f'])
            walking_step = max(max(abs(a - b) for a, b in zip(t[i], t[(i + 1) % n]))
                               for t in frames.values() for i in range(n))
            for old in sg.DIRECTIONS:
                for new in sg.DIRECTIONS:
                    if old == new:
                        continue
                    for start in range(n):
                        trot.start_movement(old)
                        trot.tick()                                  # ease in
                        sent = []
                        for _ in range(start + 1):
                            clock.now += 1.0
                            sent.append(trot.tick()[0])
                        trot.start_movement(new)
                        self.assertEqual(trot.direction, old)       # not straight away
                        for _ in range(n // 2 + 1):
                            clock.now += 1.0
                            sent.append(trot.tick()[0])
                        self.assertEqual(trot.direction, new, (profile.key, old, new, start))
                        self.assertIsNone(trot.pending)
                        for a, b in zip(sent, sent[1:]):
                            self.assertLessEqual(max(abs(x - y) for x, y in zip(a, b)),
                                                 walking_step + 1e-9, (profile.key, old, new, start))
                        trot.stop()
            trot.switch_gait()

    def test_a_turn_taken_back_before_the_handover_is_dropped(self):
        clock = Clock()
        trot = sg.SquroTrot(BIOMIMETIC, clock=clock)
        trot.switch_gait()                                          # fast trot
        trot.start_movement('f')
        trot.tick()
        clock.now += 1.0
        trot.tick()                                                 # frame 1, switches at 4 and 17
        trot.start_movement('b')
        self.assertEqual(trot.pending, 'b')
        trot.start_movement('f')
        self.assertIsNone(trot.pending)
        for _ in range(30):
            clock.now += 1.0
            trot.tick()
        self.assertEqual(trot.direction, 'f')
        trot.start_movement('l')
        trot.stop()
        self.assertIsNone(trot.pending)

    def test_diagonal_pairs_alternate_and_one_pair_is_always_down(self):
        solvers = BIOMIMETIC.leg_solvers()
        for profile in sg.PROFILES:
            frames = sg.build_trot(BIOMIMETIC, profile)['f']
            ground = [sg.walking_stance(s, i, profile)[1] for i, s in enumerate(solvers)]

            def lifted(leg, frame):
                return solvers[leg].fk_solve(*frame[leg * 2: leg * 2 + 2])[1] < ground[leg] - 0.2

            for frame in frames:
                up = [lifted(leg, frame) for leg in range(4)]
                self.assertEqual(up[0], up[3])        # FL with BR
                self.assertEqual(up[1], up[2])        # FR with BL
                self.assertFalse(up[0] and up[1])     # never all four in the air

    def test_feet_travel_the_stride_around_the_reached_stance(self):
        for profile in sg.PROFILES:
            for leg, solver in enumerate(BIOMIMETIC.leg_solvers()):
                centre = sg.walking_stance(solver, leg, profile)
                shift = centre[0] - solver.spec.stance[0]
                self.assertAlmostEqual(shift, profile.reach_front if leg < 2 else -profile.reach_hind)
                path = sg.foot_path(centre, +1, profile)
                xs = [x for x, _ in path]
                self.assertAlmostEqual(max(xs) - min(xs), profile.stride, delta=profile.stride / profile.swing + 0.1)
                self.assertAlmostEqual(min(y for _, y in path), centre[1] - peak_lift(profile), delta=0.01)

    def test_eases_in_paces_frames_switches_only_when_stopped(self):
        clock = Clock()
        trot = sg.SquroTrot(BIOMIMETIC, clock=clock)
        self.assertEqual(trot.current_gait, 'TROT')
        self.assertTrue(trot.start_movement('fl'))        # diagonal falls back to forward
        self.assertEqual(trot.direction, 'f')
        self.assertFalse(trot.switch_gait())               # not while walking
        positions, ms = trot.tick()
        self.assertEqual(ms, sg.SLOW.ease_ms)
        self.assertIsNone(trot.tick())                     # waits out the ease
        clock.now += sg.SLOW.ease_ms / 1000
        _, ms = trot.tick()
        self.assertEqual(ms, sg.SLOW.frame_ms)
        frame = trot.frame
        trot.start_movement('b')                           # phase carries over
        self.assertEqual(trot.frame, frame)
        home, ms = trot.stop()
        self.assertEqual((home, ms), (sg.standing_positions(BIOMIMETIC), sg.SLOW.ease_ms))
        self.assertTrue(trot.switch_gait())
        self.assertEqual(trot.current_gait, 'TROT_FAST')
        trot.start_movement('f')
        _, ms = trot.tick()
        self.assertEqual(ms, sg.FAST.ease_ms)
        clock.now += sg.FAST.ease_ms / 1000
        _, ms = trot.tick()
        self.assertEqual(ms, sg.FAST.frame_ms)
        self.assertFalse(trot.start_movement('x'))


class GuiTests(Home):
    def run_gui(self, link, frames, held, script=None, argv=('--legs', 'BIOMIMETIC'),
                direction='f'):
        frame = [0]
        script = script or {}

        def events():
            frame[0] += 1
            if frame[0] >= frames:
                return [pygame.event.Event(pygame.QUIT)]
            return script.get(frame[0], [])

        pygame.init()
        with patch.object(InputHandler, 'is_movement_input', side_effect=lambda: held('move', frame[0])), \
             patch.object(InputHandler, 'is_action_pressed', side_effect=lambda n: held(n, frame[0])), \
             patch.object(InputHandler, 'head_direction', side_effect=lambda: held('head', frame[0]) or 0), \
             patch.object(InputHandler, 'get_movement_direction', return_value=direction), \
             patch('espnow.q8_espnow', return_value=link), \
             patch('helpers.XiaoPortFinder.validate', return_value=True), \
             patch.object(pygame.event, 'get', side_effect=events), \
             patch('time.sleep'), \
             patch.object(sys, 'argv', ['operate.py', 'mock-port', *argv]):
            runpy.run_path(str(ROOT / 'q8bot' / 'operate.py'), run_name='__main__')
        pygame.quit()

    def test_trot_only_after_standing_then_back_to_the_stance(self):
        link = FakeLink(RESTING, SQURO_WALLS)
        standing_at = []
        held = lambda name, n: (name == 'move' and (5 <= n <= 9 or 700 <= n <= 950))
        self.run_gui(link, 1000, held, script={12: key(pygame.K_t), 760: key(pygame.K_t)})
        moves = [c for c in link.calls if c[0] == 'move']
        # Before T nothing moved, even with W held.
        first_arm = link.kinds().index('arm')
        self.assertFalse(any(c[0] == 'move' for c in link.calls[:first_arm]))
        trot = [c for c in moves if c[2] in (sg.SLOW.frame_ms, sg.SLOW.ease_ms)]
        self.assertTrue(any(c[2] == sg.SLOW.frame_ms for c in trot), 'no trot frames were sent')
        ease = [i for i, c in enumerate(moves) if c[2] == sg.SLOW.ease_ms]
        self.assertGreaterEqual(len(ease), 2)                         # ease in, ease out
        self.assertEqual(moves[ease[-1]][1], sg.standing_positions(BIOMIMETIC))
        # T while walking did not start lying down; no head key, no head command.
        self.assertEqual(link.calls[-1], ('torque', 0))
        self.assertNotIn('head', link.kinds())

    def test_g_switches_to_the_fast_trot_while_standing(self):
        link = FakeLink(RESTING, SQURO_WALLS)
        held = lambda name, n: name == 'move' and 700 <= n <= 900
        self.run_gui(link, 950, held, script={12: key(pygame.K_t), 650: key(pygame.K_g)})
        durations = [c[2] for c in link.calls if c[0] == 'move']
        self.assertIn(sg.FAST.frame_ms, durations, 'no fast trot frames were sent')
        self.assertNotIn(sg.SLOW.frame_ms, durations)
        self.assertEqual(durations[-1], sg.FAST.ease_ms)          # eased back to standing

    def test_head_arrows_stay_inside_the_limits(self):
        link = FakeLink(RESTING, SQURO_WALLS)
        held = lambda name, n: (1 if name == 'head' and 5 <= n <= 400 else 0)
        self.run_gui(link, 420, held)
        heads = [c for c in link.calls if c[0] == 'head']
        self.assertTrue(heads)
        low, high = hc.DEFAULT_LIMITS
        self.assertTrue(all(low <= deg <= high for _, deg, _ in heads))
        self.assertAlmostEqual(abs(heads[-1][1]), 15.0)
        self.assertFalse(any(c[0] in ('move', 'arm') for c in link.calls))
        self.assertNotIn(('torque', 1), link.calls)

    def test_detection_measures_head_limits_for_the_session(self):
        link = FakeLink(HANGING, SQURO_WALLS)
        self.run_gui(link, 60, lambda name, n: 0, argv=(),
                     script={5: key(pygame.K_RETURN), 40: key(pygame.K_DOWN),
                             42: key(pygame.K_RETURN)})
        saved = json.loads((Path(self.home.name) / 'head.json').read_text())
        self.assertTrue(saved['measured'])
        self.assertAlmostEqual(saved['limits'][0], -32 + 8, delta=0.2)
        self.assertAlmostEqual(saved['limits'][1], 41 - 8, delta=0.2)


if __name__ == '__main__':
    unittest.main()
