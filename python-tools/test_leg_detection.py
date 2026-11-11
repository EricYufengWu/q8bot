"""Hardware-free checks for leg detection: protocol, safety bounds, matching."""
import os
import math
import random
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / 'q8bot'))
import espnow
import leg_detection as ld
from leg_designs import BIOMIMETIC, Q8
from squro_solver import squro_solver


def deg_to_ticks(deg):
    return int(round(deg * 4096 / 360)) + 4096


def telemetry_values(angles, torque=(), faults=(), missing=()):
    """A telemetry line as the robot firmware builds it."""
    values = [0] * 100
    values[0], values[1] = espnow.TAG_TELEMETRY, 9
    for i in range(9):
        servo_id = 11 + i
        if servo_id in missing:
            continue
        angle = angles[i] if i < len(angles) else 0.0
        flags = 1 | (2 if servo_id in torque else 0) | (4 if servo_id in faults else 0)
        values[2 + i * 4: 6 + i * 4] = [flags, deg_to_ticks(angle) + 32768, 5 + 32768,
                                        32 if servo_id in faults else 0]
    return values


def probe_values(status, joints):
    """joints: {index: (flags, reason, start_deg, stop_deg)}."""
    values = [0] * 100
    values[0], values[1] = espnow.TAG_PROBE, status
    for i, (flags, reason, start, stop) in joints.items():
        values[2 + i * 6: 8 + i * 6] = [flags, reason, deg_to_ticks(start) + 32768,
                                        deg_to_ticks(stop) + 32768, 40, 0]
    return values


class ProtocolTests(unittest.TestCase):
    def test_reply_lines_parse_and_text_lines_do_not(self):
        self.assertEqual(espnow.parse_values(b'61001 9 1 2\r\n'), [61001, 9, 1, 2])
        self.assertIsNone(espnow.parse_values('[PAIRING] Paired successfully'))
        self.assertIsNone(espnow.parse_values(''))
        self.assertIsNone(espnow.decode_reply([87] + [0] * 99))   # a battery reading

    def test_telemetry_decodes_every_servo(self):
        angles = [52.66, 129.02, 52.66, 129.02, -41.29, 57.0, -41.29, 57.0, -1.3]
        reply = espnow.decode_reply(telemetry_values(angles, torque={13}, faults={16}, missing={18}))
        self.assertEqual(reply['kind'], 'telemetry')
        self.assertAlmostEqual(reply['servos'][11]['deg'], 52.66, delta=0.1)
        self.assertAlmostEqual(reply['servos'][19]['deg'], -1.3, delta=0.1)
        self.assertTrue(reply['servos'][13]['torque'])
        self.assertTrue(reply['servos'][16]['fault'])
        self.assertFalse(reply['servos'][18]['responding'])
        self.assertEqual(len(ld.telemetry_problems(reply)), 3)

    def test_probe_reply_decodes_reasons_and_angles(self):
        reply = espnow.decode_reply(probe_values(0, {0: (3, 1, 10.0, 47.5), 1: (1, 4, 120.0, 120.0)}))
        self.assertEqual(reply['status_text'], 'ok')
        self.assertEqual(reply['joints'][0]['reason'], 'contact')
        self.assertAlmostEqual(reply['joints'][0]['stop'], 47.5, delta=0.1)
        self.assertEqual(reply['joints'][1]['reason'], 'held')
        self.assertIsNone(reply['joints'][2]['start'])

    def test_turn_offsets_keep_commands_on_the_servos_turn(self):
        with patch('serial.Serial') as port:
            link = espnow.q8_espnow('mock')
            # ID16 read -5 deg past its edge at power-up: the reading says 355.
            offsets = link.set_turn_offsets([52.7, 129.0, 52.7, 129.0, -41.3, 355.0, -41.3, 57.0],
                                            [52.7, 129.0, 52.7, 129.0, -41.3, -5.0, -41.3, 57.0])
            self.assertEqual(offsets, [0, 0, 0, 0, 0, 1, 0, 0])
            link.move_all([0, 0, 0, 0, 0, 57.0, 0, 0], 100, False)
            sent = port.return_value.write.call_args[0][0].decode()
            self.assertEqual(float(sent.split(',')[5]), 417.0)
            # Arming parks the goals with torque off, then turns torque on.
            port.return_value.write.reset_mock()
            link.arm([0.0] * 8)
            first, second = [c[0][0].decode() for c in port.return_value.write.call_args_list]
            self.assertTrue(first.endswith(',0;') and second.endswith(',1;'))


class SafetyBoundTests(unittest.TestCase):
    def test_change_points_are_flagged(self):
        front, five_bar = squro_solver.front(), Q8.solver()
        self.assertFalse(ld.near_change_point(front, *front.mount_pose))
        self.assertTrue(ld.near_change_point(front, 52.0, 175.0))       # hip: crank on the base line
        self.assertTrue(ld.near_change_point(front, 120.0, 129.0))      # knee: crank on the humerus
        self.assertTrue(ld.near_change_point(front, -50.0, 129.0))      # knee: folded the other way
        self.assertTrue(ld.near_change_point(five_bar, 112.95, 67.05))  # knee pins meet
        self.assertFalse(ld.near_change_point(five_bar, 45.879, 134.121))

    def test_no_planned_sweep_reaches_any_candidate_change_point(self):
        designs = ld.candidates()
        solvers = [d.leg_solvers() for d in designs]
        rng = random.Random(7)
        for _ in range(200):
            angles = [rng.uniform(-180, 180) for _ in range(8)]
            for part, direction in ld.ROUNDS:
                excursions, _ = ld.plan_round(angles, designs, part, direction)
                for joint, excursion in enumerate(excursions):
                    if not excursion:
                        continue
                    self.assertLessEqual(abs(excursion), ld.MAX_SWEEP)
                    self.assertGreaterEqual(abs(excursion), ld.MIN_SWEEP)
                    leg = joint // 2
                    # Every degree swept, for every candidate, stays clear.
                    for step in range(1, int(abs(excursion)) + 1):
                        pose = [angles[leg * 2], angles[leg * 2 + 1]]
                        pose[joint % 2] += math.copysign(step, excursion)
                        for s in solvers:
                            self.assertFalse(ld.near_change_point(s[leg], *pose))

    def test_a_round_sweeps_one_joint_per_leg(self):
        excursions, _ = ld.plan_round(list(BIOMIMETIC.mount_positions()), ld.candidates(), 'q2', -1)
        for leg in range(4):
            self.assertEqual(excursions[leg * 2], 0.0)


class FakeRobot:
    """Answers like the firmware: walls at fixed absolute angles per joint."""

    def __init__(self, angles, walls):
        self.angles, self.walls = list(angles), walls
        self.replies, self.probes, self.aborted = [], [], False

    def request_telemetry(self):
        self.replies.append(espnow.decode_reply(telemetry_values(self.angles + [0.0])))

    def start_probe(self, excursions, pwm):
        self.probes.append(list(excursions))
        joints = {}
        for i, excursion in enumerate(excursions):
            if not excursion:
                continue
            start = self.angles[i]
            wall = self.walls.get((i, 1 if excursion > 0 else -1))
            reach = start + excursion
            if wall is not None and min(start, reach) <= wall <= max(start, reach):
                joints[i] = (3, 1, start, wall)
            else:
                joints[i] = (3, 3, start, reach)
            joints[i ^ 1] = (1, 4, self.angles[i ^ 1], self.angles[i ^ 1])
        self.replies.append(espnow.decode_reply(probe_values(0, joints)))

    def start_head_probe(self, bound, pwm):
        self.probes.append(('head', bound))
        start = self.angles[8] if len(self.angles) > 8 else 0.0
        wall = self.walls.get((8, 1 if bound > 0 else -1))
        reach = start + bound
        hit = wall is not None and min(start, reach) <= wall <= max(start, reach)
        self.replies.append(espnow.decode_reply(
            probe_values(0, {8: (3, 1 if hit else 3, start, wall if hit else reach)})))

    def abort_probe(self):
        self.aborted = True


class Flaky(FakeRobot):
    """Fails the first sweeps with the given statuses; can then lose or fault a servo."""

    def __init__(self, angles, walls, failures=(), silent_after=None, fault_after=None):
        super().__init__(angles, walls)
        self.failures = list(failures)
        self.silent_after, self.fault_after = silent_after, fault_after
        self.failed = 0
        self.mute = False

    def request_telemetry(self):
        if self.mute:
            return
        missing = {self.silent_after} if self.failed and self.silent_after else set()
        faults = {self.fault_after} if self.failed and self.fault_after else set()
        self.replies.append(espnow.decode_reply(
            telemetry_values(self.angles + [0.0], faults=faults, missing=missing)))

    def start_probe(self, excursions, pwm):
        if self.failures:
            self.probes.append(list(excursions))
            self.failed += 1
            self.replies.append(espnow.decode_reply(probe_values(self.failures.pop(0), {})))
            return
        super().start_probe(excursions, pwm)


SQURO_WALLS = {(i, d): w for i, d, w in [
    (0, 1, 60), (0, -1, -40), (1, 1, 150), (1, -1, 60), (2, 1, 60), (2, -1, -40),
    (3, 1, 150), (3, -1, 60), (4, 1, -10), (4, -1, -80), (5, 1, 110), (5, -1, 10),
    (6, 1, -10), (6, -1, -80), (7, 1, 110), (7, -1, 10), (8, 1, 41), (8, -1, -32)]}
HANGING = [20.0, 120.0, 20.0, 120.0, -40.0, 50.0, -40.0, 50.0]


class DetectionTests(unittest.TestCase):
    def setUp(self):
        self.home = tempfile.TemporaryDirectory()
        self.previous_home = os.environ.get('Q8BOT_HOME')
        os.environ['Q8BOT_HOME'] = self.home.name

    def tearDown(self):
        # Restore rather than remove: other modules rely on an isolated home.
        if self.previous_home is None:
            os.environ.pop('Q8BOT_HOME', None)
        else:
            os.environ['Q8BOT_HOME'] = self.previous_home
        self.home.cleanup()

    def run_detection(self, robot, store=None):
        clock = [0.0]
        detector = ld.LegDetector(robot, store=store or ld.SignatureStore(), clock=lambda: clock[0])
        detector.request_telemetry()
        for _ in range(40):
            while robot.replies:
                detector.feed(robot.replies.pop(0))
            if detector.stage == 'ready':
                self.assertTrue(detector.start())
            if detector.stage in ('done', 'failed'):
                break
            detector.update()
        return detector

    def test_first_run_has_nothing_to_match_then_learning_makes_it_recognisable(self):
        detector = self.run_detection(FakeRobot(HANGING, SQURO_WALLS))
        self.assertEqual(detector.stage, 'done')
        self.assertIsNone(detector.result['design'])
        self.assertIn('no design has a recording', detector.result['reason'])
        detector.record('BIOMIMETIC')
        self.assertEqual(detector.result['design'], 'BIOMIMETIC')
        # A later startup, legs hanging a little differently, readings wrapped
        # by a turn on one joint: still recognised.
        store = ld.SignatureStore()
        later = [a + 3.0 for a in HANGING]
        walls = {k: w + 1.5 for k, w in SQURO_WALLS.items()}
        later[5] += 360.0                                   # ID16 wrapped at power-up
        walls.update({(5, d): w + 360.0 for (j, d), w in walls.items() if j == 5})
        again = self.run_detection(FakeRobot(later, walls), store)
        self.assertEqual(again.result['design'], 'BIOMIMETIC', again.result)

    def test_different_stops_are_not_recognised(self):
        store = ld.SignatureStore()
        first = self.run_detection(FakeRobot(HANGING, SQURO_WALLS), store)
        first.record('BIOMIMETIC')
        other = {k: w + 25.0 for k, w in SQURO_WALLS.items()}
        detector = self.run_detection(FakeRobot(HANGING, other), store)
        self.assertIsNone(detector.result['design'])
        self.assertGreater(detector.result['scores']['BIOMIMETIC'][1], 0)

    def test_no_stops_at_all_is_reported_not_guessed(self):
        detector = self.run_detection(FakeRobot(HANGING, {}))
        self.assertIsNone(detector.result['design'])
        self.assertIn('too few', detector.result['reason'])

    def test_silence_from_old_firmware_is_reported(self):
        clock = [0.0]

        class Silent(FakeRobot):
            def request_telemetry(self):
                pass

        detector = ld.LegDetector(Silent(HANGING, {}), store=ld.SignatureStore(), clock=lambda: clock[0])
        detector.request_telemetry()
        clock[0] = 5.0
        detector.update()
        self.assertEqual(detector.stage, 'no_reply')

    def test_a_sweep_failing_every_time_stops_detection_with_a_diagnosis(self):
        robot = Flaky(HANGING, SQURO_WALLS, failures=[8] * (ld.MAX_RETRIES + 1))
        detector = self.run_detection(robot)
        self.assertEqual(detector.stage, 'failed')
        self.assertIn('pushed out of place', detector.error)
        self.assertEqual(detector.diagnosis.category, 'disturbed')
        self.assertEqual(len(detector.take_notes()), ld.MAX_RETRIES)   # each retry logged

    def test_a_servo_dropping_out_once_is_retried_and_detection_completes(self):
        store = ld.SignatureStore()
        self.run_detection(FakeRobot(HANGING, SQURO_WALLS), store).record('BIOMIMETIC')
        robot = Flaky(HANGING, SQURO_WALLS, failures=[7, 5])
        detector = self.run_detection(robot, store)
        self.assertEqual(detector.stage, 'done', detector.error)
        self.assertEqual(detector.result['design'], 'BIOMIMETIC')
        notes = detector.take_notes()
        self.assertEqual(len(notes), 2)
        self.assertIn('stopped answering', notes[0])
        # The failed sweep was sent again, and every round completed.
        self.assertEqual(len([p for p in robot.probes if p[0] != 'head']), len(ld.ROUNDS) + 2)

    def test_a_servo_still_silent_after_a_failure_is_named(self):
        robot = Flaky(HANGING, SQURO_WALLS, failures=[7], silent_after=16)
        detector = self.run_detection(robot)
        self.assertEqual(detector.stage, 'failed')
        self.assertEqual(detector.diagnosis.category, 'communication')
        self.assertIn('BL shoulder (ID 16)', detector.diagnosis.detail)
        self.assertEqual(len(robot.probes), 1)                       # not retried blindly

    def test_a_fault_after_a_sweep_is_diagnosed_not_retried(self):
        robot = Flaky(HANGING, SQURO_WALLS, failures=[6], fault_after=12)
        detector = self.run_detection(robot)
        self.assertEqual(detector.stage, 'failed')
        self.assertEqual(detector.diagnosis.category, 'torque')
        self.assertIn('FL shoulder (ID 12) reports overload', detector.diagnosis.detail)

    def test_no_reply_to_the_check_is_a_communication_problem(self):
        clock = [0.0]
        robot = Flaky(HANGING, SQURO_WALLS, failures=[7])
        detector = ld.LegDetector(robot, store=ld.SignatureStore(), clock=lambda: clock[0])
        detector.request_telemetry()
        detector.feed(robot.replies.pop())
        detector.start()
        robot.mute = True
        detector.feed(robot.replies.pop())
        self.assertEqual(detector.stage, 'rechecking')
        self.assertIn('Checking every servo', detector.progress())
        clock[0] = 5.0
        detector.update()
        self.assertEqual(detector.stage, 'failed')
        self.assertEqual(detector.diagnosis.text(), ld.no_answer().text())

    def test_slightly_changed_legs_still_match_and_changed_ones_are_offered(self):
        store = ld.SignatureStore()
        self.run_detection(FakeRobot(HANGING, SQURO_WALLS), store).record('BIOMIMETIC')
        # One stop moved by a part adjustment: still the same legs.
        nudged = dict(SQURO_WALLS)
        nudged[(0, 1)] += 20.0
        detector = self.run_detection(FakeRobot(HANGING, nudged), store)
        self.assertEqual(detector.result['design'], 'BIOMIMETIC', detector.result)
        self.assertIn('1 disagree', detector.result['reason'])
        # Half the stops moved: not named, but offered as the closest.
        moved = {k: (w + 25.0 if k[0] in (0, 2, 4) else w) for k, w in SQURO_WALLS.items()}
        detector = self.run_detection(FakeRobot(HANGING, moved), store)
        self.assertIsNone(detector.result['design'])
        self.assertEqual(detector.result['closest'], 'BIOMIMETIC')
        self.assertIn('closest is Biomimetic', detector.result['reason'])

    def test_cancel_aborts_the_running_sweep(self):
        robot = FakeRobot(HANGING, SQURO_WALLS)
        detector = ld.LegDetector(robot, store=ld.SignatureStore())
        detector.request_telemetry()
        detector.feed(robot.replies.pop())
        detector.start()
        detector.abort()
        self.assertTrue(robot.aborted)
        self.assertEqual(detector.stage, 'failed')

    def test_the_head_is_swept_after_the_legs_and_never_classified(self):
        robot = FakeRobot(HANGING, SQURO_WALLS)
        detector = self.run_detection(robot)
        self.assertEqual(detector.stage, 'done')
        heads = [p for p in robot.probes if p[0] == 'head']
        self.assertEqual([round(b) for _, b in heads], [90, -90])
        self.assertEqual(robot.probes.index(heads[0]), len(robot.probes) - 2)   # legs first
        self.assertEqual(detector.head[+1]['kind'], 'contact')
        low, high = ld.head_limits(detector.head)
        self.assertAlmostEqual(low, -32 + ld.HEAD_MARGIN, delta=0.1)    # ticks quantise angles
        self.assertAlmostEqual(high, 41 - ld.HEAD_MARGIN, delta=0.1)
        self.assertFalse(any(key.startswith('HEAD') for key in detector.measurement))

    def test_head_sweeps_stay_inside_their_range_and_skip_an_unready_head(self):
        self.assertEqual(ld.head_bound(10.0, +1), 80.0)
        self.assertEqual(ld.head_bound(10.0, -1), -100.0)
        self.assertEqual(ld.head_bound(95.0, +1), 0.0)
        torqued = espnow.decode_reply(telemetry_values(HANGING + [0.0], torque={19}))
        missing = espnow.decode_reply(telemetry_values(HANGING + [0.0], missing={19}))
        self.assertFalse(ld.head_ready(torqued))
        self.assertFalse(ld.head_ready(missing))
        # A contact on the wrong side of centre leaves no usable limits.
        self.assertIsNone(ld.head_limits({+1: {'kind': 'contact', 'stop': 5.0},
                                          -1: {'kind': 'contact', 'stop': -30.0}}))

    def test_recordings_persist_and_are_capped(self):
        store = ld.SignatureStore()
        for _ in range(ld.MAX_RUNS + 2):
            store.record('BIOMIMETIC', {'FL.q1+': {'kind': 'free'}})
        self.assertEqual(len(ld.SignatureStore().runs('BIOMIMETIC')), ld.MAX_RUNS)
        ld.save_last_design('BIOMIMETIC')
        self.assertEqual(ld.load_last_design(), 'BIOMIMETIC')


if __name__ == '__main__':
    unittest.main()
