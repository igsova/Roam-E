IMUPLUS_MODE = 0x08
FUSION_RUNNING = 0x05
WINDOWS_UNITS = 0x00


def fusion_state_is_healthy(mode, status, error, units):
    return (
        mode == IMUPLUS_MODE
        and status == FUSION_RUNNING
        and error == 0
        and units == WINDOWS_UNITS
    )


def physical_calibration_is_ready(gyro_cal, accel_cal):
    """IMUPLUS only depends on the gyroscope and accelerometer."""
    return gyro_cal == 3 and accel_cal == 3
