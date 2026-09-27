import unittest

from roam_e.imu_health import (
    FUSION_RUNNING,
    IMUPLUS_MODE,
    WINDOWS_UNITS,
    fusion_state_is_healthy,
    physical_calibration_is_ready,
)


class FusionStateTest(unittest.TestCase):
    def test_expected_imuplus_state_is_healthy(self):
        self.assertTrue(
            fusion_state_is_healthy(
                IMUPLUS_MODE,
                FUSION_RUNNING,
                0,
                WINDOWS_UNITS,
            )
        )

    def test_ndof_mode_is_not_accepted_by_imuplus_runtime(self):
        self.assertFalse(
            fusion_state_is_healthy(
                0x0C,
                FUSION_RUNNING,
                0,
                WINDOWS_UNITS,
            )
        )


class PhysicalCalibrationTest(unittest.TestCase):
    def test_gyro_and_accelerometer_are_required(self):
        self.assertTrue(physical_calibration_is_ready(3, 3))
        self.assertFalse(physical_calibration_is_ready(2, 3))
        self.assertFalse(physical_calibration_is_ready(3, 2))

    def test_magnetometer_is_not_part_of_imuplus_policy(self):
        # The policy intentionally has no magnetometer argument.
        self.assertTrue(physical_calibration_is_ready(3, 3))


if __name__ == '__main__':
    unittest.main()
