#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu, MagneticField, Temperature

import board
import busio
import adafruit_bno055

DEG2RAD = math.pi / 180.0
UT_TO_T = 1e-6

def _valid_tuple(t, n):
    return (t is not None) and (len(t) >= n) and all(v is not None for v in t[:n])

class BNO055ImuNode(Node):
    def __init__(self):
        super().__init__('bno055_imu_node')
        self.declare_parameter('i2c_addr', 0x28)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('publish_rate_hz', 50.0)
        i2c_addr = int(self.get_parameter('i2c_addr').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        rate_hz = float(self.get_parameter('publish_rate_hz').value)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        self.pub_imu = self.create_publisher(Imu, 'imu', sensor_qos)
        self.pub_mag = self.create_publisher(MagneticField, 'mag', sensor_qos)
        self.pub_temp = self.create_publisher(Temperature, 'temperature', sensor_qos)

        i2c = busio.I2C(board.SCL, board.SDA)
        self.bno = adafruit_bno055.BNO055_I2C(i2c, address=i2c_addr)

        self.timer = self.create_timer(1.0 / rate_hz, self._on_timer)

    def _on_timer(self):
        now = self.get_clock().now().to_msg()
        msg_imu = Imu()
        msg_imu.header.stamp = now
        msg_imu.header.frame_id = self.frame_id

        try:
            q = self.bno.quaternion          # may contain None values
            gyro = self.bno.gyro             # deg/s, may contain Nones
            lin_acc = self.bno.linear_acceleration  # m/s^2, may contain Nones
        except Exception as e:
            self.get_logger().debug(f'IMU read error: {e}')
            return

        # Orientation
        if _valid_tuple(q, 4):
            msg_imu.orientation.x = float(q[0])
            msg_imu.orientation.y = float(q[1])
            msg_imu.orientation.z = float(q[2])
            msg_imu.orientation.w = float(q[3])
            msg_imu.orientation_covariance[0] = 0.0
        else:
            msg_imu.orientation_covariance[0] = -1.0  # unknown

        # Angular velocity
        if _valid_tuple(gyro, 3):
            msg_imu.angular_velocity.x = float(gyro[0]) * DEG2RAD
            msg_imu.angular_velocity.y = float(gyro[1]) * DEG2RAD
            msg_imu.angular_velocity.z = float(gyro[2]) * DEG2RAD
            msg_imu.angular_velocity_covariance[0] = 0.0
        else:
            msg_imu.angular_velocity_covariance[0] = -1.0

        # Linear acceleration
        if _valid_tuple(lin_acc, 3):
            msg_imu.linear_acceleration.x = float(lin_acc[0])
            msg_imu.linear_acceleration.y = float(lin_acc[1])
            msg_imu.linear_acceleration.z = float(lin_acc[2])
            msg_imu.linear_acceleration_covariance[0] = 0.0
        else:
            msg_imu.linear_acceleration_covariance[0] = -1.0

        self.pub_imu.publish(msg_imu)

        # Optional extras (best-effort)
        try:
            mag = self.bno.magnetic
            if _valid_tuple(mag, 3):
                m = MagneticField()
                m.header.stamp = now
                m.header.frame_id = self.frame_id
                m.magnetic_field.x = float(mag[0]) * UT_TO_T
                m.magnetic_field.y = float(mag[1]) * UT_TO_T
                m.magnetic_field.z = float(mag[2]) * UT_TO_T
                self.pub_mag.publish(m)
        except Exception:
            pass

        try:
            temp_c = self.bno.temperature
            if temp_c is not None:
                t = Temperature()
                t.header.stamp = now
                t.header.frame_id = self.frame_id
                t.temperature = float(temp_c)
                t.variance = 0.0
                self.pub_temp.publish(t)
        except Exception:
            pass

def main():
    rclpy.init()
    node = BNO055ImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

