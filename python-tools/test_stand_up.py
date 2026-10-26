"""Hardware-free checks for the SQuRo stand-up and lie-down."""
import sys
import unittest
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / 'q8bot'))
import espnow
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


class PlanTests(unittest.TestCase):
    def test_rise_from_rest_ends_in_the_cad_stance(self):
        waypoints, error = su.plan_motion(BIOMIMETIC, RESTING, su.STANCE_DEPTH)
        self.assertIsNone(error)
        self.assertEqual(len(waypoints), su.STEPS)
        # The CAD stance is the mounting pose, front and hind.
        for got, want in zip(waypoints[-1], BIOMIMETIC.mount_positions()):
            self.assertAlmostEqual(got, want, delta=0.05)

    def test_every_waypoint_is_safe(self):
        for start, depth in ((RESTING, su.STANCE_DEPTH), (pose_at(su.STANCE_DEPTH), su.CROUCH_DEPTH)):
            waypoints, error = su.plan_motion(BIOMIMETIC, start, depth)
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
        waypoints, _ = su.plan_motion(BIOMIMETIC, pose_at(su.STANCE_DEPTH), su.CROUCH_DEPTH)
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
        waypoints, error = su.plan_motion(BIOMIMETIC, start, su.STANCE_DEPTH)
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
        return True

    def move_all(self, positions, duration, record):
        self.moves.append((list(positions), duration))
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
        # Output capped before torque came on, full again once standing.
        self.assertEqual(robot.caps, [su.STAND_PWM, su.FULL_PWM])
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
        self.assertEqual(robot.caps[-1], su.STAND_PWM)
        self.assertEqual(len(robot.moves), su.STEPS)

    def test_unconfirmed_cap_stops_before_torque(self):
        clock = [0.0]
        robot = FakeRobot(list(RESTING), confirm=0x7F)
        sequence = su.StandUp(BIOMIMETIC, robot, clock=lambda: clock[0])
        sequence.toggle()
        self.drive(sequence, robot, clock)
        self.assertEqual(sequence.stage, 'failed')
        self.assertIsNone(robot.armed)
        self.assertEqual(robot.moves, [])

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
