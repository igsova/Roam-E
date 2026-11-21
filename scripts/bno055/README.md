# BNO055 IMU tools

The BNO055 runs in its IMU-only fusion mode (gyro and accelerometer, no
magnetometer) on I²C bus 3 of the Pi. Its calibration is saved once and
reloaded at every boot, because the sensor forgets it when power is removed.

| Script | What it does |
|---|---|
| `calibrate.py` | Interactive calibration. Shows the calibration status while you move the robot, then saves the offsets and axis remap to [`config/bno055_calibration.json`](../../config/bno055_calibration.json) |
| `restore.py` | Validates that file and writes it back to the sensor. Run at boot by the `roam-e-bno055-restore` systemd unit ([installer](../setup/install_bno055_restore_service.sh)) |
| `check_mount_readonly.py` | Reads raw sensor output with the robot held in named poses to confirm how the chip is mounted. Read-only, no ROS |
| `ros_imu_axis_test.py` | Guides you through nose-up, left-up and counter-clockwise moves and prints the orientation change seen on `/imu/data`, to check the ROS axis convention |
| `turn_calibration.py` | Records one full in-place turn driven with teleop and compares the yaw from the integrated gyro, the IMU orientation and the wheel odometry. Used to tune the effective wheel separation |

`test_check_mount_readonly.py` tests the mount check's math without hardware:

```bash
cd scripts/bno055 && python3 -m pytest
```
