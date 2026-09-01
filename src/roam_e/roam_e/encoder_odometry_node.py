#!/usr/bin/env python3
"""Read Arduino encoder telemetry and publish joint states plus wheel odometry."""

import math
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
import serial
from tf2_ros import TransformBroadcaster

from .wheel_odometry import DifferentialDriveTracker, parse_motor_telemetry


POSE_COVARIANCE = [0.0] * 36
POSE_COVARIANCE[0] = 0.02
POSE_COVARIANCE[7] = 0.02
POSE_COVARIANCE[14] = 1e6
POSE_COVARIANCE[21] = 1e6
POSE_COVARIANCE[28] = 1e6
POSE_COVARIANCE[35] = 0.05

TWIST_COVARIANCE = [0.0] * 36
TWIST_COVARIANCE[0] = 0.03
TWIST_COVARIANCE[7] = 0.03
TWIST_COVARIANCE[14] = 1e6
TWIST_COVARIANCE[21] = 1e6
TWIST_COVARIANCE[28] = 1e6
TWIST_COVARIANCE[35] = 0.10


def diagnostic_value(key, value):
    item = KeyValue()
    item.key = str(key)
    item.value = str(value)
    return item


class EncoderOdometryNode(Node):
    def __init__(self):
        super().__init__('encoder_odometry_node')

        self.declare_parameter(
            'serial_port',
            '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0',
        )
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('ticks_per_revolution', 748)
        self.declare_parameter('wheel_radius', 0.033875)
        self.declare_parameter('wheel_separation', 0.173630)
        self.declare_parameter('max_tick_rate', 10000.0)
        self.declare_parameter('left_joint_name', 'left_wheel_joint')
        self.declare_parameter('right_joint_name', 'right_wheel_joint')
        self.declare_parameter('odom_topic', 'wheel/odometry')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('publish_odom_tf', True)
        self.declare_parameter('telemetry_timeout', 2.0)
        self.declare_parameter('reconnect_interval', 2.0)

        self.serial_port = str(self.get_parameter('serial_port').value)
        self.baud_rate = int(self.get_parameter('baud_rate').value)
        self.left_joint_name = str(
            self.get_parameter('left_joint_name').value
        )
        self.right_joint_name = str(
            self.get_parameter('right_joint_name').value
        )
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.publish_odom_tf = bool(
            self.get_parameter('publish_odom_tf').value
        )
        self.telemetry_timeout = float(
            self.get_parameter('telemetry_timeout').value
        )
        self.reconnect_interval = float(
            self.get_parameter('reconnect_interval').value
        )

        if self.baud_rate <= 0:
            raise ValueError('baud_rate must be positive')
        if self.telemetry_timeout <= 0.0:
            raise ValueError('telemetry_timeout must be positive')
        if self.reconnect_interval <= 0.0:
            raise ValueError('reconnect_interval must be positive')

        self.tracker = DifferentialDriveTracker(
            ticks_per_revolution=int(
                self.get_parameter('ticks_per_revolution').value
            ),
            wheel_radius=float(self.get_parameter('wheel_radius').value),
            wheel_separation=float(
                self.get_parameter('wheel_separation').value
            ),
            max_tick_rate=float(self.get_parameter('max_tick_rate').value),
        )

        odom_topic = str(self.get_parameter('odom_topic').value)
        self.joint_publisher = self.create_publisher(
            JointState, 'joint_states', 10
        )
        self.odom_publisher = self.create_publisher(
            Odometry, odom_topic, 10
        )
        self.diagnostic_publisher = self.create_publisher(
            DiagnosticArray, '/diagnostics', 10
        )
        self.transform_broadcaster = (
            TransformBroadcaster(self) if self.publish_odom_tf else None
        )

        self.connection = None
        self.line_buffer = bytearray()
        self.next_connect_time = 0.0
        self.connected_at = None
        self.last_telemetry_time = None
        self.last_connection_error = 'none'
        self.last_controller_message = 'none'
        self.last_telemetry = None
        self.last_update = None
        self.valid_lines = 0
        self.invalid_lines = 0
        self.reconnects = 0
        self.last_warning_time = 0.0

        self.poll_timer = self.create_timer(0.02, self._poll_serial)
        self.diagnostic_timer = self.create_timer(
            1.0, self._publish_diagnostics
        )

        self.get_logger().info(
            'Encoder odometry is read-only: no motor commands will be sent. '
            f'Waiting for {self.serial_port} at {self.baud_rate} baud.'
        )

    def _warn_throttled(self, message):
        now = time.monotonic()
        if now - self.last_warning_time >= 5.0:
            self.get_logger().warning(message)
            self.last_warning_time = now

    def _connect(self):
        now = time.monotonic()
        if self.connection is not None or now < self.next_connect_time:
            return

        try:
            self.connection = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                timeout=0,
                write_timeout=0,
                exclusive=True,
            )
            self.connection.reset_input_buffer()
            self.line_buffer.clear()
            self.connected_at = now
            self.last_telemetry_time = None
            self.last_connection_error = 'none'
            self.reconnects += 1
            self.get_logger().info(
                f'Connected to motor controller on {self.serial_port}'
            )
        except (OSError, serial.SerialException) as error:
            self.connection = None
            self.last_connection_error = str(error)
            self.next_connect_time = now + self.reconnect_interval
            self._warn_throttled(
                f'Encoder serial connection failed: {error}'
            )

    def _disconnect(self, reason):
        if self.connection is not None:
            try:
                self.connection.close()
            except (OSError, serial.SerialException):
                pass

        self.connection = None
        self.line_buffer.clear()
        self.last_connection_error = reason
        self.next_connect_time = time.monotonic() + self.reconnect_interval
        self._warn_throttled(f'Encoder serial disconnected: {reason}')

    def _poll_serial(self):
        self._connect()
        if self.connection is None:
            return

        try:
            waiting = self.connection.in_waiting
            if waiting:
                self.line_buffer.extend(self.connection.read(waiting))

            while b'\n' in self.line_buffer:
                raw_line, _, remainder = self.line_buffer.partition(b'\n')
                self.line_buffer = bytearray(remainder)
                line = raw_line.rstrip(b'\r').decode('ascii', errors='replace')
                self._handle_line(line)

            if len(self.line_buffer) > 4096:
                self.invalid_lines += 1
                self.line_buffer.clear()

            reference_time = self.last_telemetry_time or self.connected_at
            if (
                reference_time is not None
                and time.monotonic() - reference_time > self.telemetry_timeout
            ):
                self._disconnect('telemetry timeout')
        except (OSError, serial.SerialException) as error:
            self._disconnect(str(error))

    def _handle_line(self, line):
        if not line:
            return

        if not line.startswith('T,'):
            self.last_controller_message = line
            return

        try:
            telemetry = parse_motor_telemetry(line)
        except ValueError as error:
            self.invalid_lines += 1
            self._warn_throttled(f'Invalid encoder telemetry: {error}')
            return

        self.valid_lines += 1
        self.last_telemetry_time = time.monotonic()
        self.last_telemetry = telemetry
        update = self.tracker.update(telemetry)
        if update is None:
            return

        self.last_update = update
        self._publish_state(update)

    def _publish_state(self, update):
        stamp = self.get_clock().now().to_msg()

        joint_state = JointState()
        joint_state.header.stamp = stamp
        joint_state.name = [self.left_joint_name, self.right_joint_name]
        joint_state.position = [
            update.left_position,
            update.right_position,
        ]
        joint_state.velocity = [
            update.left_velocity,
            update.right_velocity,
        ]
        self.joint_publisher.publish(joint_state)

        half_yaw = 0.5 * update.yaw
        quaternion_z = math.sin(half_yaw)
        quaternion_w = math.cos(half_yaw)

        odometry = Odometry()
        odometry.header.stamp = stamp
        odometry.header.frame_id = self.odom_frame
        odometry.child_frame_id = self.base_frame
        odometry.pose.pose.position.x = update.x
        odometry.pose.pose.position.y = update.y
        odometry.pose.pose.orientation.z = quaternion_z
        odometry.pose.pose.orientation.w = quaternion_w
        odometry.pose.covariance = POSE_COVARIANCE
        odometry.twist.twist.linear.x = update.linear_velocity
        odometry.twist.twist.angular.z = update.angular_velocity
        odometry.twist.covariance = TWIST_COVARIANCE
        self.odom_publisher.publish(odometry)

        if self.transform_broadcaster is not None:
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = self.odom_frame
            transform.child_frame_id = self.base_frame
            transform.transform.translation.x = update.x
            transform.transform.translation.y = update.y
            transform.transform.rotation.z = quaternion_z
            transform.transform.rotation.w = quaternion_w
            self.transform_broadcaster.sendTransform(transform)

    def _publish_diagnostics(self):
        stamp = self.get_clock().now().to_msg()
        diagnostic = DiagnosticStatus()
        diagnostic.name = 'Roam-E wheel encoders'
        diagnostic.hardware_id = f'arduino-nano:{self.serial_port}'

        telemetry_age = (
            time.monotonic() - self.last_telemetry_time
            if self.last_telemetry_time is not None
            else float('inf')
        )

        if self.connection is None:
            diagnostic.level = DiagnosticStatus.ERROR
            diagnostic.message = 'Motor-controller serial disconnected'
        elif self.last_telemetry is None:
            diagnostic.level = DiagnosticStatus.WARN
            diagnostic.message = 'Waiting for encoder telemetry'
        elif telemetry_age > self.telemetry_timeout:
            diagnostic.level = DiagnosticStatus.ERROR
            diagnostic.message = 'Encoder telemetry is stale'
        else:
            diagnostic.level = DiagnosticStatus.OK
            diagnostic.message = 'Encoder telemetry active (read-only)'

        telemetry = self.last_telemetry
        update = self.last_update
        diagnostic.values = [
            diagnostic_value('serial_port', self.serial_port),
            diagnostic_value('baud_rate', self.baud_rate),
            diagnostic_value('read_only', 'true'),
            diagnostic_value(
                'ticks_per_revolution', self.tracker.ticks_per_revolution
            ),
            diagnostic_value('wheel_radius_m', self.tracker.wheel_radius),
            diagnostic_value(
                'wheel_separation_m', self.tracker.wheel_separation
            ),
            diagnostic_value('telemetry_age_s', f'{telemetry_age:.3f}'),
            diagnostic_value('valid_lines', self.valid_lines),
            diagnostic_value('invalid_lines', self.invalid_lines),
            diagnostic_value('serial_connections', self.reconnects),
            diagnostic_value(
                'odometry_rebases', self.tracker.rebase_count
            ),
            diagnostic_value(
                'last_rebase_reason', self.tracker.last_rebase_reason
            ),
            diagnostic_value(
                'left_ticks', telemetry.left_ticks if telemetry else 'none'
            ),
            diagnostic_value(
                'right_ticks', telemetry.right_ticks if telemetry else 'none'
            ),
            diagnostic_value(
                'left_pwm', telemetry.left_pwm if telemetry else 'none'
            ),
            diagnostic_value(
                'right_pwm', telemetry.right_pwm if telemetry else 'none'
            ),
            diagnostic_value(
                'sample_period_s', f'{update.dt:.3f}' if update else 'none'
            ),
            diagnostic_value(
                'last_controller_message', self.last_controller_message
            ),
            diagnostic_value('last_serial_error', self.last_connection_error),
        ]

        array = DiagnosticArray()
        array.header.stamp = stamp
        array.status = [diagnostic]
        self.diagnostic_publisher.publish(array)

    def destroy_node(self):
        if self.connection is not None:
            try:
                self.connection.close()
            except (OSError, serial.SerialException):
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = EncoderOdometryNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
