import math
import unittest

from roam_e.motor_control import (
    feedforward_command,
    pid_command,
    twist_to_wheel_targets,
    velocity_command,
)


class TwistConversionTest(unittest.TestCase):
    def setUp(self):
        self.metres_per_tick = 2.0 * math.pi * 0.033975 / 748
        self.arguments = {
            'wheel_separation': 0.175150,
            'metres_per_tick': self.metres_per_tick,
            'max_linear_velocity': 0.15,
            'max_angular_velocity': 1.5,
            'max_wheel_ticks_per_second': 1200.0,
        }

    def convert(self, linear, angular):
        return twist_to_wheel_targets(
            linear_velocity=linear,
            angular_velocity=angular,
            **self.arguments,
        )

    def test_straight_motion_has_equal_wheel_targets(self):
        targets = self.convert(0.1, 0.0)

        expected = round(0.1 / self.metres_per_tick)
        self.assertEqual(targets.left_ticks_per_second, expected)
        self.assertEqual(targets.right_ticks_per_second, expected)
        self.assertFalse(targets.limited)

    def test_positive_yaw_uses_opposite_wheel_targets(self):
        targets = self.convert(0.0, 1.0)

        self.assertLess(targets.left_ticks_per_second, 0)
        self.assertGreater(targets.right_ticks_per_second, 0)
        self.assertEqual(
            targets.left_ticks_per_second,
            -targets.right_ticks_per_second,
        )

    def test_body_and_wheel_limits_are_enforced(self):
        targets = self.convert(10.0, 10.0)

        self.assertTrue(targets.limited)
        self.assertLessEqual(
            abs(targets.left_ticks_per_second), 1200
        )
        self.assertLessEqual(
            abs(targets.right_ticks_per_second), 1200
        )

    def test_nonfinite_command_is_rejected(self):
        with self.assertRaises(ValueError):
            self.convert(math.nan, 0.0)


class SerialCommandTest(unittest.TestCase):
    def test_velocity_command_is_integer_and_newline_terminated(self):
        self.assertEqual(velocity_command(123, -456), b'VEL,123,-456\n')

    def test_pid_command_uses_fixed_point_milli_units(self):
        self.assertEqual(
            pid_command(0.08, 0.2, 0.1, 45),
            b'PID,80,200,100,45\n',
        )

    def test_out_of_range_pid_is_rejected(self):
        with self.assertRaises(ValueError):
            pid_command(1.1, 0.2, 0.1, 45)

    def test_feedforward_command_has_four_directional_values(self):
        self.assertEqual(
            feedforward_command(45, 36, 40, 39),
            b'FF,45,36,40,39\n',
        )

    def test_out_of_range_feedforward_is_rejected(self):
        with self.assertRaises(ValueError):
            feedforward_command(45, -1, 40, 39)


if __name__ == '__main__':
    unittest.main()
