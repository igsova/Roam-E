#!/usr/bin/env python3

"""Compare wheel-odometry yaw with short-term BNO055 gyro yaw."""

import math
import statistics
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


def normalized_angle(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def quaternion_yaw(quaternion):
    return math.atan2(
        2.0 * (
            quaternion.w * quaternion.z
            + quaternion.x * quaternion.y
        ),
        1.0 - 2.0 * (
            quaternion.y * quaternion.y
            + quaternion.z * quaternion.z
        ),
    )


class UnwrappedYaw:
    def __init__(self):
        self.previous = None
        self.total = 0.0

    def update(self, angle):
        if self.previous is not None:
            self.total += normalized_angle(angle - self.previous)
        self.previous = angle


class TurnCalibrationNode(Node):
    def __init__(self):
        super().__init__('turn_calibration')
        self.started_at = time.monotonic()
        self.ready = False
        self.recording = False
        self.complete = False
        self.gyro_samples = []
        self.gyro_bias = 0.0
        self.latest_base_yaw_rate = 0.0
        self.latest_imu_stamp = None
        self.previous_recorded_imu_stamp = None
        self.gyro_angle = 0.0
        self.last_nonzero_command = None
        self.imu_yaw = UnwrappedYaw()
        self.odom_yaw = UnwrappedYaw()
        self.start_imu_yaw = 0.0
        self.start_odom_yaw = 0.0

        self.create_subscription(
            Imu,
            '/imu/data',
            self._imu_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry,
            '/wheel/odometry',
            self._odom_callback,
            10,
        )
        self.create_subscription(Twist, '/cmd_vel', self._cmd_callback, 10)
        self.create_timer(0.05, self._timer_callback)

    def _imu_callback(self, message):
        # imu_link is mounted with roll=pi, so its Z axis is opposite base Z.
        base_yaw_rate = -message.angular_velocity.z
        stamp = (
            float(message.header.stamp.sec)
            + float(message.header.stamp.nanosec) * 1.0e-9
        )
        self.latest_base_yaw_rate = base_yaw_rate
        self.latest_imu_stamp = stamp
        self.imu_yaw.update(quaternion_yaw(message.orientation))

        if not self.recording:
            if not self.ready:
                self.gyro_samples.append(base_yaw_rate)
            return

        if self.previous_recorded_imu_stamp is not None:
            delta_time = stamp - self.previous_recorded_imu_stamp
            if 0.0 < delta_time <= 0.2:
                self.gyro_angle += (
                    base_yaw_rate - self.gyro_bias
                ) * delta_time
        self.previous_recorded_imu_stamp = stamp

    def _odom_callback(self, message):
        self.odom_yaw.update(quaternion_yaw(message.pose.pose.orientation))

    def _cmd_callback(self, message):
        if not self.ready or abs(message.angular.z) < 0.05:
            return

        now = time.monotonic()
        if not self.recording:
            self.recording = True
            self.gyro_angle = 0.0
            self.previous_recorded_imu_stamp = self.latest_imu_stamp
            self.start_imu_yaw = self.imu_yaw.total
            self.start_odom_yaw = self.odom_yaw.total
            print('TURN DETECTED - recording IMU and wheel yaw...', flush=True)

        self.last_nonzero_command = now

    def _timer_callback(self):
        now = time.monotonic()

        if not self.ready:
            if now - self.started_at > 2.0 and len(self.gyro_samples) >= 20:
                self.gyro_bias = statistics.mean(self.gyro_samples[-40:])
                self.ready = True
                print(
                    'READY - hold Left or Right for one complete turn; '
                    'release when finished.',
                    flush=True,
                )
                print(
                    f'Base yaw-rate bias: {self.gyro_bias:+.6f} rad/s',
                    flush=True,
                )
            elif now - self.started_at > 10.0:
                raise RuntimeError('Timed out waiting for IMU samples')
            return

        if not self.recording or self.last_nonzero_command is None:
            return

        command_stopped = now - self.last_nonzero_command > 0.7
        gyro_stopped = abs(
            self.latest_base_yaw_rate - self.gyro_bias
        ) < 0.05

        if command_stopped and gyro_stopped:
            self._finish()

    def _finish(self):
        self.recording = False
        self.complete = True
        imu_quaternion_angle = self.imu_yaw.total - self.start_imu_yaw
        odom_angle = self.odom_yaw.total - self.start_odom_yaw

        print('TURN COMPLETE', flush=True)
        print(
            'IMU gyro integral: '
            f'{self.gyro_angle:+.6f} rad '
            f'({math.degrees(self.gyro_angle):+.3f} deg)',
            flush=True,
        )
        print(
            'IMU quaternion yaw: '
            f'{imu_quaternion_angle:+.6f} rad '
            f'({math.degrees(imu_quaternion_angle):+.3f} deg)',
            flush=True,
        )
        print(
            'Wheel odometry yaw: '
            f'{odom_angle:+.6f} rad '
            f'({math.degrees(odom_angle):+.3f} deg)',
            flush=True,
        )
        if abs(imu_quaternion_angle) > 0.1:
            print(
                'Wheel/IMU yaw scale: '
                f'{abs(odom_angle / imu_quaternion_angle):.6f}',
                flush=True,
            )


def main():
    rclpy.init()
    node = TurnCalibrationNode()
    try:
        while rclpy.ok() and not node.complete:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        print('Cancelled.')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
