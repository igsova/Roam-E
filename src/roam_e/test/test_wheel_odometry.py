import math
import unittest

from roam_e.wheel_odometry import (
    DifferentialDriveTracker,
    INT32_MAX,
    INT32_MIN,
    MotorTelemetry,
    parse_motor_telemetry,
    signed_counter_delta,
    unsigned_elapsed_ms,
)


def telemetry(milliseconds, left_ticks, right_ticks, left_pwm=0, right_pwm=0):
    return MotorTelemetry(
        controller_time_ms=milliseconds,
        left_ticks=left_ticks,
        right_ticks=right_ticks,
        left_pwm=left_pwm,
        right_pwm=right_pwm,
    )


class MotorTelemetryTest(unittest.TestCase):
    def test_valid_telemetry_is_parsed(self):
        parsed = parse_motor_telemetry('T,1234,-10,20,-80,90')

        self.assertEqual(
            parsed,
            telemetry(1234, -10, 20, -80, 90),
        )

    def test_non_telemetry_line_is_ignored(self):
        self.assertIsNone(
            parse_motor_telemetry('READY,ROAM_E_MOTOR_CONTROLLER,1')
        )

    def test_extended_velocity_telemetry_is_parsed(self):
        parsed = parse_motor_telemetry(
            'T,1234,-10,20,-80,90,300,310,295,305'
        )

        self.assertEqual(parsed.left_target_ticks_per_second, 300)
        self.assertEqual(parsed.right_target_ticks_per_second, 310)
        self.assertEqual(parsed.left_measured_ticks_per_second, 295)
        self.assertEqual(parsed.right_measured_ticks_per_second, 305)

    def test_malformed_telemetry_is_rejected(self):
        for line in (
            'T,1,2,3,4',
            'T,one,2,3,4,5',
            'T,1,2,3,999,0',
        ):
            with self.subTest(line=line):
                with self.assertRaises(ValueError):
                    parse_motor_telemetry(line)

    def test_counter_rollovers_are_handled(self):
        self.assertEqual(signed_counter_delta(INT32_MIN, INT32_MAX), 1)
        self.assertEqual(signed_counter_delta(INT32_MAX, INT32_MIN), -1)
        self.assertEqual(unsigned_elapsed_ms(5, 0xFFFFFFFA), 11)


class DifferentialDriveTrackerTest(unittest.TestCase):
    def setUp(self):
        self.tracker = DifferentialDriveTracker(
            ticks_per_revolution=748,
            wheel_radius=0.033975,
            wheel_separation=0.175150,
        )

    def test_one_wheel_revolution_drives_straight_by_circumference(self):
        self.assertIsNone(self.tracker.update(telemetry(1000, 0, 0)))

        update = self.tracker.update(telemetry(2000, 748, 748))

        self.assertAlmostEqual(update.left_position, 2.0 * math.pi)
        self.assertAlmostEqual(update.right_position, 2.0 * math.pi)
        self.assertAlmostEqual(update.x, 2.0 * math.pi * 0.033975)
        self.assertAlmostEqual(update.y, 0.0)
        self.assertAlmostEqual(update.yaw, 0.0)

    def test_opposite_wheel_motion_rotates_in_place(self):
        self.tracker.update(telemetry(1000, 0, 0))

        update = self.tracker.update(telemetry(1100, -20, 20))

        expected_yaw = (
            40
            * self.tracker.metres_per_tick
            / self.tracker.wheel_separation
        )
        self.assertAlmostEqual(update.x, 0.0)
        self.assertAlmostEqual(update.y, 0.0)
        self.assertAlmostEqual(update.yaw, expected_yaw)
        self.assertGreater(update.angular_velocity, 0.0)

    def test_controller_restart_rebases_without_odometry_jump(self):
        self.tracker.update(telemetry(50000, 1000, 1000))
        first = self.tracker.update(telemetry(50100, 1010, 1010))

        self.assertIsNone(self.tracker.update(telemetry(100, 0, 0)))
        after_restart = self.tracker.update(telemetry(200, 10, 10))

        self.assertAlmostEqual(after_restart.x, 2.0 * first.x)
        self.assertEqual(
            self.tracker.last_rebase_reason,
            'controller clock restarted',
        )

    def test_implausible_tick_reset_is_rebased(self):
        self.tracker.update(telemetry(1000, 100000, 100000))

        self.assertIsNone(self.tracker.update(telemetry(1100, 0, 0)))
        self.assertEqual(
            self.tracker.last_rebase_reason,
            'implausible encoder jump',
        )


if __name__ == '__main__':
    unittest.main()
