#!/usr/bin/env python3

import math
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, MagneticField, Temperature
from smbus2 import SMBus


CHIP_ID = 0x00
MAG_DATA = 0x0E
GYRO_DATA = 0x14
TEMPERATURE = 0x34
CALIB_STAT = 0x35
SYS_STATUS = 0x39
SYS_ERROR = 0x3A
UNIT_SEL = 0x3B
OPR_MODE = 0x3D

EXPECTED_CHIP_ID = 0xA0
NDOF_MODE = 0x0C
FUSION_RUNNING = 0x05
WINDOWS_UNITS = 0x00

DEG_TO_RAD = math.pi / 180.0
UT_TO_T = 1.0e-6
SQRT_HALF = math.sqrt(0.5)

# The BNO055 Windows orientation convention used by restore.py is north-west-up
# (NWU). ROS REP-103 requires east-north-up (ENU). A +90 degree rotation around
# world Z converts an NWU orientation quaternion into ENU.
Q_ENU_FROM_NWU = (0.0, 0.0, SQRT_HALF, SQRT_HALF)

ORIENTATION_COVARIANCE = [
    0.0027, 0.0, 0.0,
    0.0, 0.0027, 0.0,
    0.0, 0.0, 0.0076,
]
ANGULAR_VELOCITY_COVARIANCE = [
    0.0025, 0.0, 0.0,
    0.0, 0.0025, 0.0,
    0.0, 0.0, 0.0025,
]
LINEAR_ACCELERATION_COVARIANCE = [
    0.09, 0.0, 0.0,
    0.0, 0.09, 0.0,
    0.0, 0.0, 0.09,
]
MAGNETIC_FIELD_COVARIANCE = [
    2.5e-11, 0.0, 0.0,
    0.0, 2.5e-11, 0.0,
    0.0, 0.0, 2.5e-11,
]


def signed_16(low, high):
    value = low | (high << 8)
    return value - 0x10000 if value & 0x8000 else value


def signed_8(value):
    return value - 0x100 if value & 0x80 else value


def calibration_values(value):
    return (
        (value >> 6) & 0x03,
        (value >> 4) & 0x03,
        (value >> 2) & 0x03,
        value & 0x03,
    )


def quaternion_multiply(first, second):
    """Hamilton product for quaternions stored as (x, y, z, w)."""
    x1, y1, z1, w1 = first
    x2, y2, z2, w2 = second

    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


def diagnostic_value(key, value):
    item = KeyValue()
    item.key = str(key)
    item.value = str(value)
    return item


class BNO055ImuNode(Node):
    def __init__(self):
        super().__init__('bno055_imu_node')

        self.declare_parameter('i2c_bus', 3)
        self.declare_parameter('i2c_addr', 0x29)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('publish_rate_hz', 20.0)

        self.bus_number = int(self.get_parameter('i2c_bus').value)
        self.address = int(self.get_parameter('i2c_addr').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.publish_rate = float(
            self.get_parameter('publish_rate_hz').value
        )

        if not 0.0 < self.publish_rate <= 20.0:
            raise ValueError('publish_rate_hz must be in (0, 20]')

        self.i2c_errors = 0
        self.consecutive_errors = 0
        self.last_error = 'none'
        self.last_sample_time = None
        self.last_warning_time = 0.0

        self.bus = SMBus(self.bus_number)

        try:
            self._verify_ready()
        except Exception:
            self.bus.close()
            raise

        self.imu_publisher = self.create_publisher(
            Imu, 'imu/data', qos_profile_sensor_data
        )
        self.mag_publisher = self.create_publisher(
            MagneticField, 'imu/mag', qos_profile_sensor_data
        )
        self.temperature_publisher = self.create_publisher(
            Temperature, 'imu/temperature', qos_profile_sensor_data
        )
        self.diagnostic_publisher = self.create_publisher(
            DiagnosticArray, '/diagnostics', 10
        )

        self.sample_timer = self.create_timer(
            1.0 / self.publish_rate, self._publish_sample
        )
        self.diagnostic_timer = self.create_timer(
            1.0, self._publish_diagnostics
        )

        self.get_logger().info(
            f'BNO055 ready: /dev/i2c-{self.bus_number}, '
            f'address=0x{self.address:02X}, frame={self.frame_id}, '
            f'rate={self.publish_rate:.1f} Hz'
        )
        self.get_logger().info(
            'Orientation quaternion converted from BNO055 NWU to ROS ENU. '
            'Axis test requires base_link -> imu_link roll=pi.'
        )

    def _retry(self, operation, description):
        last_exception = None

        for attempt in range(3):
            try:
                result = operation()
                self.consecutive_errors = 0
                return result
            except OSError as error:
                last_exception = error
                self.i2c_errors += 1
                self.last_error = f'{description}: {error}'

                if attempt < 2:
                    time.sleep(0.003)

        self.consecutive_errors += 1
        raise OSError(
            f'{description} failed after 3 attempts: {last_exception}'
        )

    def _read_register(self, register):
        return self._retry(
            lambda: self.bus.read_byte_data(self.address, register),
            f'register 0x{register:02X}',
        )

    def _read_block(self, register, length):
        return self._retry(
            lambda: self.bus.read_i2c_block_data(
                self.address, register, length
            ),
            f'block 0x{register:02X}',
        )

    def _verify_ready(self):
        chip_id = self._read_register(CHIP_ID)
        mode = self._read_register(OPR_MODE)
        status = self._read_register(SYS_STATUS)
        error = self._read_register(SYS_ERROR)
        units = self._read_register(UNIT_SEL)

        if chip_id != EXPECTED_CHIP_ID:
            raise RuntimeError(f'Unexpected chip ID: 0x{chip_id:02X}')

        if (
            mode != NDOF_MODE
            or status != FUSION_RUNNING
            or error != 0
            or units != WINDOWS_UNITS
        ):
            raise RuntimeError(
                'BNO055 is not healthy after restoration: '
                f'mode=0x{mode:02X}, status=0x{status:02X}, '
                f'error=0x{error:02X}, units=0x{units:02X}'
            )

    def _warn_read_error(self, operation, error):
        now = time.monotonic()

        if now - self.last_warning_time >= 5.0:
            self.get_logger().warning(f'{operation}: {error}')
            self.last_warning_time = now

    def _publish_sample(self):
        try:
            # Gyro 0x14, Euler 0x1A, quaternion 0x20 and
            # linear acceleration 0x28 are contiguous (26 bytes).
            data = self._read_block(GYRO_DATA, 26)
            mag = self._read_block(MAG_DATA, 6)
        except OSError as error:
            self._warn_read_error('Sensor sample failed', error)
            return

        stamp = self.get_clock().now().to_msg()
        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = self.frame_id

        imu.angular_velocity.x = (
            signed_16(data[0], data[1]) / 16.0 * DEG_TO_RAD
        )
        imu.angular_velocity.y = (
            signed_16(data[2], data[3]) / 16.0 * DEG_TO_RAD
        )
        imu.angular_velocity.z = (
            signed_16(data[4], data[5]) / 16.0 * DEG_TO_RAD
        )
        imu.angular_velocity_covariance = ANGULAR_VELOCITY_COVARIANCE

        quaternion_w = signed_16(data[12], data[13]) / 16384.0
        quaternion_x = signed_16(data[14], data[15]) / 16384.0
        quaternion_y = signed_16(data[16], data[17]) / 16384.0
        quaternion_z = signed_16(data[18], data[19]) / 16384.0
        quaternion_norm = math.sqrt(
            quaternion_w ** 2
            + quaternion_x ** 2
            + quaternion_y ** 2
            + quaternion_z ** 2
        )

        if 0.5 <= quaternion_norm <= 1.5:
            native_quaternion = (
                quaternion_x / quaternion_norm,
                quaternion_y / quaternion_norm,
                quaternion_z / quaternion_norm,
                quaternion_w / quaternion_norm,
            )
            enu_quaternion = quaternion_multiply(
                Q_ENU_FROM_NWU,
                native_quaternion,
            )
            imu.orientation.x = enu_quaternion[0]
            imu.orientation.y = enu_quaternion[1]
            imu.orientation.z = enu_quaternion[2]
            imu.orientation.w = enu_quaternion[3]
            imu.orientation_covariance = ORIENTATION_COVARIANCE
        else:
            imu.orientation_covariance[0] = -1.0

        # BNO055 linear-acceleration output has gravity removed.
        imu.linear_acceleration.x = signed_16(data[20], data[21]) / 100.0
        imu.linear_acceleration.y = signed_16(data[22], data[23]) / 100.0
        imu.linear_acceleration.z = signed_16(data[24], data[25]) / 100.0
        imu.linear_acceleration_covariance = LINEAR_ACCELERATION_COVARIANCE

        magnetic = MagneticField()
        magnetic.header.stamp = stamp
        magnetic.header.frame_id = self.frame_id
        magnetic.magnetic_field.x = (
            signed_16(mag[0], mag[1]) / 16.0 * UT_TO_T
        )
        magnetic.magnetic_field.y = (
            signed_16(mag[2], mag[3]) / 16.0 * UT_TO_T
        )
        magnetic.magnetic_field.z = (
            signed_16(mag[4], mag[5]) / 16.0 * UT_TO_T
        )
        magnetic.magnetic_field_covariance = MAGNETIC_FIELD_COVARIANCE

        self.imu_publisher.publish(imu)
        self.mag_publisher.publish(magnetic)
        self.last_sample_time = time.monotonic()

    def _publish_diagnostics(self):
        stamp = self.get_clock().now().to_msg()
        diagnostic = DiagnosticStatus()
        diagnostic.name = 'Roam-E BNO055 IMU'
        diagnostic.hardware_id = (
            f'bno055:i2c-{self.bus_number}:0x{self.address:02X}'
        )

        try:
            temperature = signed_8(self._read_register(TEMPERATURE))
            calibration = self._read_register(CALIB_STAT)
            mode = self._read_register(OPR_MODE)
            status = self._read_register(SYS_STATUS)
            error = self._read_register(SYS_ERROR)
            units = self._read_register(UNIT_SEL)
            system_cal, gyro_cal, accel_cal, mag_cal = (
                calibration_values(calibration)
            )

            temperature_message = Temperature()
            temperature_message.header.stamp = stamp
            temperature_message.header.frame_id = self.frame_id
            temperature_message.temperature = float(temperature)
            temperature_message.variance = 0.0
            self.temperature_publisher.publish(temperature_message)

            if (
                mode != NDOF_MODE
                or status != FUSION_RUNNING
                or error != 0
                or units != WINDOWS_UNITS
            ):
                diagnostic.level = DiagnosticStatus.ERROR
                diagnostic.message = 'Fusion state is unhealthy'
            elif gyro_cal < 3 or accel_cal < 3 or mag_cal < 3:
                diagnostic.level = DiagnosticStatus.WARN
                diagnostic.message = 'Physical calibration incomplete'
            else:
                diagnostic.level = DiagnosticStatus.OK
                diagnostic.message = 'Fusion running and sensors calibrated'

            sample_age = (
                time.monotonic() - self.last_sample_time
                if self.last_sample_time is not None
                else float('inf')
            )
            diagnostic.values = [
                diagnostic_value('mode', f'0x{mode:02X}'),
                diagnostic_value('system_status', f'0x{status:02X}'),
                diagnostic_value('system_error', f'0x{error:02X}'),
                diagnostic_value('unit_selection', f'0x{units:02X}'),
                diagnostic_value('orientation_world_frame', 'ENU'),
                diagnostic_value(
                    'orientation_conversion',
                    'BNO055 Windows/NWU -> ROS ENU',
                ),
                diagnostic_value('calibration_system', system_cal),
                diagnostic_value('calibration_gyroscope', gyro_cal),
                diagnostic_value('calibration_accelerometer', accel_cal),
                diagnostic_value('calibration_magnetometer', mag_cal),
                diagnostic_value('temperature_c', temperature),
                diagnostic_value('sample_age_s', f'{sample_age:.3f}'),
                diagnostic_value('i2c_errors', self.i2c_errors),
                diagnostic_value('last_i2c_error', self.last_error),
            ]
        except OSError as error:
            diagnostic.level = DiagnosticStatus.ERROR
            diagnostic.message = 'I2C diagnostic read failed'
            diagnostic.values = [
                diagnostic_value('error', error),
                diagnostic_value('i2c_errors', self.i2c_errors),
            ]
            self._warn_read_error('Diagnostic read failed', error)

        array = DiagnosticArray()
        array.header.stamp = stamp
        array.status = [diagnostic]
        self.diagnostic_publisher.publish(array)

    def destroy_node(self):
        try:
            self.bus.close()
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = BNO055ImuNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
