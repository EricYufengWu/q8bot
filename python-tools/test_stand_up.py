"""Hardware-free checks for the SQuRo stand-up and lie-down."""
import sys
import unittest
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / 'q8bot'))
import espnow
import squro_gait as sg
import stand_up as su
from leg_designs import BIOMIMETIC
from leg_detection import near_change_point


def pose_at(depth):
    """All four feet at their stance x with the floor ``depth`` below the axes."""
    positions = []
    for solver in BIOMIMETIC.leg_solvers():
        q1, q2, ok = solver.ik_solve(solver.spec.stance[0], depth - solver.contact_radius)
        assert ok
        positions += [float(q1), float(q2)]
    return positions


RESTING = pose_at(29.0)   # feet and front elbows on the floor, unpowered
STAND = (su.STAND_REACH_FRONT, su.STAND_REACH_HIND)


class PlanTests(unittest.TestCase):
    def test_rise_from_rest_ends_in_the_standing_pose(self):
        waypoints, error = su.plan_motion(BIOMIMETIC, RESTING, su.STAND_DEPTH, STAND)
        self.assertIsNone(error)
        self.assertEqual(len(waypoints), su.STEPS)
        # The pose the trot starts from and returns to.
        for got, want in zip(waypoints[-1], sg.standing_positions(BIOMIMETIC)):
            self.assertAlmostEqual(got, want, delta=0.05)

    def test_rise_keeps_front_and_hind_legs_as_far_apart_as_resting(self):
        solvers = BIOMIMETIC.leg_solvers()

        def gap(positions):
            pivots = [s.pivots(*positions[i * 2: i * 2 + 2]) for i, s in enumerate(solvers)]
            return sg.leg_gap(pivots[0], pivots[2], solvers[0].contact_radius)

        waypoints, _ = su.plan_motion(BIOMIMETIC, RESTING, su.STAND_DEPTH, STAND)
        self.assertGreaterEqual(min(gap(w) for w in waypoints), gap(RESTING) - 0.1)
        self.assertGreater(gap(waypoints[-1]), sg.MIN_LEG_GAP)

    def test_every_waypoint_is_safe(self):
        for start, depth, reach in ((RESTING, su.STAND_DEPTH, STAND),
                                    (sg.standing_positions(BIOMIMETIC), su.CROUCH_DEPTH, (0.0, 0.0))):
            waypoints, error = su.plan_motion(BIOMIMETIC, start, depth, reach)
            self.assertIsNone(error)
            previous = start
            for positions in waypoints:
                for leg, solver in enumerate(BIOMIMETIC.leg_solvers()):
                    q1, q2 = positions[leg * 2: leg * 2 + 2]
                    self.assertFalse(near_change_point(solver, q1, q2))
                    self.assertLessEqual(abs(q1 - previous[leg * 2]), su.MAX_STEP_DEG)
                    self.assertLessEqual(abs(q2 - previous[leg * 2 + 1]), su.MAX_STEP_DEG)
                previous = positions

    def test_crouch_keeps_elbows_knees_and_belly_off_the_floor(self):
        waypoints, _ = su.plan_motion(BIOMIMETIC, sg.standing_positions(BIOMIMETIC), su.CROUCH_DEPTH)
        last = waypoints[-1]
        for leg, solver in enumerate(BIOMIMETIC.leg_solvers()):
            self.assertGreaterEqual(su._clearance(solver, *last[leg * 2: leg * 2 + 2]), 5.0)
            self.assertAlmostEqual(solver.contact_depth(*last[leg * 2: leg * 2 + 2]),
                                   su.CROUCH_DEPTH, delta=0.05)
        self.assertGreater(su.CROUCH_DEPTH, 26.1 + 5)   # belly plate

    def test_a_leg_folded_the_other_way_is_refused(self):
        solver = BIOMIMETIC.solver()
        q1, q2 = RESTING[0:2]
        foot = solver.pivots(q1, q2)['F']
        # Same foot position on the other elbow branch.
        other = solver.__class__(solver.spec)
        other.branch = -solver.branch
        b1, b2, ok = other.ik_solve(*foot)
        self.assertTrue(ok)
        start = list(RESTING)
        start[0:2] = [float(b1), float(b2)]
        waypoints, error = su.plan_motion(BIOMIMETIC, start, su.STAND_DEPTH, STAND)
        self.assertIsNone(waypoints)
        self.assertIn('FL is folded the other way', error)

    def test_wrapped_readings_are_brought_back_to_the_model(self):
        present = list(RESTING)
        present[5] += 360.0                                   # ID16 read past its edge
        self.assertAlmostEqual(su.model_angles(BIOMIMETIC, present)[5], RESTING[5])


class FakeRobot:
    def __init__(self, angles, confirm=0xFF):
        self.angles, self.confirm = angles, confirm
        self.replies, self.moves, self.caps, self.armed = [], [], [], None
        self.offsets_from = None

    def request_telemetry(self):
        servos = {11 + i: {'responding': True, 'deg': a} for i, a in enumerate(self.angles)}
        self.replies.append({'kind': 'telemetry', 'servos': servos})

    def set_output_cap(self, pwm):
        self.caps.append(pwm)
        self.replies.append({'kind': 'cap', 'pwm': pwm, 'confirmed': self.confirm})

    def set_turn_offsets(self, present, model):
        self.offsets_from = (present, model)

    def arm(self, positions):
        self.armed = list(positions)
        self.angles = list(positions)
        return True

    def move_all(self, positions, duration, record):
        self.moves.append((list(positions), duration))
        self.angles = list(positions)          # the servos follow
        return True


class SequenceTests(unittest.TestCase):
    def drive(self, sequence, robot, clock, frames=200):
        for _ in range(frames):
            while robot.replies:
                sequence.feed(robot.replies.pop(0))
            sequence.update()
            clock[0] += 0.05

    def test_stand_up_then_lie_down(self):
        clock = [0.0]
        present = list(RESTING)
        present[5] += 360.0
        robot = FakeRobot(present)
        sequence = su.StandUp(BIOMIMETIC, robot, clock=lambda: clock[0])
        self.assertFalse(sequence.blocks_movement())
        self.assertTrue(sequence.toggle())
        self.drive(sequence, robot, clock)
        self.assertEqual(sequence.stage, 'standing', sequence.error)
        self.assertTrue(sequence.active)
        # Full output confirmed before torque came on, and kept.
        self.assertEqual(robot.caps, [su.FULL_PWM])
        # Armed on the measured pose, so nothing jumped at torque-on.
        for got, want in zip(robot.armed, RESTING):
            self.assertAlmostEqual(got, want, delta=1e-6)
        self.assertEqual(robot.offsets_from[0], present)
        self.assertEqual(len(robot.moves), su.STEPS)
        self.assertTrue(all(duration == su.STEP_MS for _, duration in robot.moves))

        robot.moves.clear()
        self.assertTrue(sequence.toggle())
        self.drive(sequence, robot, clock)
        self.assertEqual(sequence.stage, 'crouched', sequence.error)
        self.assertFalse(sequence.active)
        self.assertEqual(robot.caps, [su.FULL_PWM, su.FULL_PWM])     # never below full
        self.assertEqual(len(robot.moves), su.STEPS)

    def test_unconfirmed_cap_stops_before_torque(self):
        clock = [0.0]
        robot = FakeRobot(list(RESTING), confirm=0x7F)
        sequence = su.StandUp(BIOMIMETIC, robot, clock=lambda: clock[0])
        sequence.toggle()
        self.drive(sequence, robot, clock)
        self.assertEqual(sequence.stage, 'failed')
        self.assertIn('servo ID 18 did not confirm', sequence.error)   # bit 7 missing
        self.assertIsNone(robot.armed)
        self.assertEqual(robot.moves, [])

    def test_a_silent_servo_is_named_and_reported(self):
        clock = [0.0]

        class OneSilent(FakeRobot):
            def request_telemetry(self):
                super().request_telemetry()
                self.replies[-1]['servos'][16]['responding'] = False

        robot = OneSilent(list(RESTING))
        sequence = su.StandUp(BIOMIMETIC, robot, clock=lambda: clock[0])
        sequence.toggle()
        failure = sequence.feed(robot.replies.pop(0))
        self.assertEqual(failure, 'no answer from BL shoulder (ID 16)')
        self.assertEqual(sequence.stage, 'failed')
        self.assertEqual(sequence.diagnosis.category, 'communication')
        self.assertEqual(sequence.diagnosis.servos, (16,))
        self.assertIsNone(robot.armed)
        reports = sequence.reports()
        self.assertEqual(reports[0], 'Stand-up stopped: no answer from BL shoulder (ID 16).')
        self.assertTrue(reports[1].startswith('Communication problem'))
        self.assertEqual(sequence.reports(), [])                      # each line once

    def test_a_joint_left_short_of_the_stance_is_a_torque_problem(self):
        clock = [0.0]

        class Weak(FakeRobot):
            def move_all(self, positions, duration, record):
                kept = self.angles[3]                                # FR shoulder cannot lift
                sent = super().move_all(positions, duration, record)
                self.angles[3] = kept
                return sent

        robot = Weak(list(RESTING))
        sequence = su.StandUp(BIOMIMETIC, robot, clock=lambda: clock[0])
        sequence.toggle()
        self.drive(sequence, robot, clock)
        self.assertEqual(sequence.stage, 'failed')
        self.assertFalse(sequence.active)                            # no trot from here
        self.assertIn('did not reach its standing pose', sequence.error)
        self.assertEqual(sequence.diagnosis.category, 'torque')
        self.assertIn('FR shoulder (ID 14)', sequence.diagnosis.detail)

    def test_a_latched_overload_is_named_before_torque_comes_on(self):
        clock = [0.0]

        class Overloaded(FakeRobot):
            def request_telemetry(self):
                super().request_telemetry()
                self.replies[-1]['servos'][17].update(fault=True, error=32)

        robot = Overloaded(list(RESTING))
        sequence = su.StandUp(BIOMIMETIC, robot, clock=lambda: clock[0])
        sequence.toggle()
        self.drive(sequence, robot, clock)
        self.assertEqual(sequence.stage, 'failed')
        self.assertIsNone(robot.armed)
        self.assertEqual(sequence.diagnosis.category, 'torque')
        self.assertIn('BR elbow (ID 17) reports overload', sequence.diagnosis.detail)
        self.assertIn('battery off and on', sequence.diagnosis.action)

    def test_an_unconfirmed_cap_is_diagnosed_from_a_read_back(self):
        clock = [0.0]
        robot = FakeRobot(list(RESTING), confirm=0xFF & ~(1 << 2))
        sequence = su.StandUp(BIOMIMETIC, robot, clock=lambda: clock[0])
        sequence.toggle()
        self.drive(sequence, robot, clock)
        self.assertIn('servo ID 13 did not confirm', sequence.error)
        # Every servo answered the read-back: an intermittent link.
        self.assertEqual(sequence.diagnosis.category, 'communication')
        self.assertIn('now and then', sequence.diagnosis.action)
        self.assertIn('T: Retry', sequence.label())

    def test_an_unplannable_start_is_a_pose_problem(self):
        clock = [0.0]
        solver = BIOMIMETIC.solver()
        foot = solver.pivots(*RESTING[0:2])['F']
        other = solver.__class__(solver.spec)
        other.branch = -solver.branch
        b1, b2, _ = other.ik_solve(*foot)
        start = list(RESTING)
        start[0:2] = [float(b1), float(b2)]
        robot = FakeRobot(start)
        sequence = su.StandUp(BIOMIMETIC, robot, clock=lambda: clock[0])
        sequence.toggle()
        self.drive(sequence, robot, clock)
        self.assertEqual(sequence.diagnosis.category, 'pose')
        self.assertIn('FL is folded the other way', sequence.diagnosis.detail)

    def test_silence_fails_cleanly(self):
        clock = [0.0]

        class Silent(FakeRobot):
            def request_telemetry(self):
                pass

        robot = Silent(list(RESTING))
        sequence = su.StandUp(BIOMIMETIC, robot, clock=lambda: clock[0])
        sequence.toggle()
        self.drive(sequence, robot, clock)
        self.assertEqual(sequence.stage, 'failed')
        self.assertIn('no reply', sequence.error)
        self.assertIsNone(robot.armed)

    def test_hold_after_calibration_restores_full_output(self):
        clock = [0.0]
        robot = FakeRobot(list(RESTING))
        sequence = su.StandUp(BIOMIMETIC, robot, clock=lambda: clock[0])
        sequence.hold(BIOMIMETIC.mount_positions())
        # Not standing (so no trot) until every servo confirms full output.
        self.assertNotEqual(sequence.stage, 'standing')
        self.assertEqual(robot.caps, [su.FULL_PWM])
        self.drive(sequence, robot, clock)
        self.assertEqual(sequence.stage, 'standing', sequence.error)
        self.assertTrue(sequence.active)
        self.assertEqual(robot.moves, [])           # holds the pose, nothing moves

    def test_hold_refuses_to_stand_on_an_unconfirmed_cap(self):
        clock = [0.0]
        robot = FakeRobot(list(RESTING), confirm=0x7F)
        sequence = su.StandUp(BIOMIMETIC, robot, clock=lambda: clock[0])
        sequence.hold(BIOMIMETIC.mount_positions())
        self.drive(sequence, robot, clock)
        self.assertEqual(sequence.stage, 'failed')
        self.assertIn('output cap', sequence.error)

    def test_hold_fails_cleanly_without_a_reply(self):
        clock = [0.0]

        class Silent(FakeRobot):
            def set_output_cap(self, pwm):
                self.caps.append(pwm)

        robot = Silent(list(RESTING))
        sequence = su.StandUp(BIOMIMETIC, robot, clock=lambda: clock[0])
        sequence.hold(BIOMIMETIC.mount_positions())
        self.drive(sequence, robot, clock)
        self.assertEqual(sequence.stage, 'failed')
        self.assertIn('no reply', sequence.error)


class ArmTests(unittest.TestCase):
    def test_arming_while_holding_never_drops_torque(self):
        with patch('serial.Serial') as port:
            link = espnow.q8_espnow('mock')
            link.arm([0.0] * 8)
            port.return_value.write.reset_mock()
            link.arm([1.0] * 8)
            sent = [c[0][0].decode() for c in port.return_value.write.call_args_list]
            self.assertEqual(len(sent), 1)
            self.assertTrue(sent[0].endswith(',1;'))


if __name__ == '__main__':
    unittest.main()
