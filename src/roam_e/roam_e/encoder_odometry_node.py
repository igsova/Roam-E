#!/usr/bin/env python3
"""Read Arduino encoder telemetry and publish joint states plus wheel odometry."""

import math
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TransformStamped, Twist, TwistStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
import serial
from tf2_ros import TransformBroadcaster

from .motor_control import (
    feedforward_command,
    pid_command,
    twist_to_wheel_targets,
    velocity_command,
)
from .wheel_odometry import (
    DifferentialDriveTracker,
    parse_motor_telemetry,
    signed_counter_delta,
)


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
        self.declare_parameter('enable_motor_control', False)
        self.declare_parameter('use_stamped_cmd_vel', True)
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('command_timeout', 0.25)
        self.declare_parameter('command_rate', 20.0)
        self.declare_parameter('max_linear_velocity', 0.15)
        self.declare_parameter('max_angular_velocity', 1.5)
        self.declare_parameter('max_wheel_ticks_per_second', 1200.0)
        self.declare_parameter('velocity_pid_kp', 0.080)
        self.declare_parameter('velocity_pid_ki', 0.200)
        self.declare_parameter('velocity_feedforward_kv', 0.100)
        self.declare_parameter('velocity_static_pwm', 45)
        self.declare_parameter('left_forward_static_pwm', 45)
        self.declare_parameter('left_reverse_static_pwm', 36)
        self.declare_parameter('right_forward_static_pwm', 40)
        self.declare_parameter('right_reverse_static_pwm', 39)

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
        self.enable_motor_control = bool(
            self.get_parameter('enable_motor_control').value
        )
        self.use_stamped_cmd_vel = bool(
            self.get_parameter('use_stamped_cmd_vel').value
        )
        self.cmd_vel_topic = str(
            self.get_parameter('cmd_vel_topic').value
        )
        self.command_timeout = float(
            self.get_parameter('command_timeout').value
        )
        self.command_rate = float(
            self.get_parameter('command_rate').value
        )
        self.max_linear_velocity = float(
            self.get_parameter('max_linear_velocity').value
        )
        self.max_angular_velocity = float(
            self.get_parameter('max_angular_velocity').value
        )
        self.max_wheel_ticks_per_second = float(
            self.get_parameter('max_wheel_ticks_per_second').value
        )
        self.velocity_pid_kp = float(
            self.get_parameter('velocity_pid_kp').value
        )
        self.velocity_pid_ki = float(
            self.get_parameter('velocity_pid_ki').value
        )
        self.velocity_feedforward_kv = float(
            self.get_parameter('velocity_feedforward_kv').value
        )
        self.velocity_static_pwm = int(
            self.get_parameter('velocity_static_pwm').value
        )
        self.left_forward_static_pwm = int(
            self.get_parameter('left_forward_static_pwm').value
        )
        self.left_reverse_static_pwm = int(
            self.get_parameter('left_reverse_static_pwm').value
        )
        self.right_forward_static_pwm = int(
            self.get_parameter('right_forward_static_pwm').value
        )
        self.right_reverse_static_pwm = int(
            self.get_parameter('right_reverse_static_pwm').value
        )

        if self.baud_rate <= 0:
            raise ValueError('baud_rate must be positive')
        if self.telemetry_timeout <= 0.0:
            raise ValueError('telemetry_timeout must be positive')
        if self.reconnect_interval <= 0.0:
            raise ValueError('reconnect_interval must be positive')
        if not 0.0 < self.command_timeout < 0.35:
            raise ValueError(
                'command_timeout must be positive and shorter than the '
                '350 ms firmware watchdog'
            )
        if not 0.0 < self.command_rate <= 50.0:
            raise ValueError('command_rate must be in (0, 50]')

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
        self.pid_configuration_command = pid_command(
            self.velocity_pid_kp,
            self.velocity_pid_ki,
            self.velocity_feedforward_kv,
            self.velocity_static_pwm,
        )
        self.feedforward_configuration_command = feedforward_command(
            self.left_forward_static_pwm,
            self.left_reverse_static_pwm,
            self.right_forward_static_pwm,
            self.right_reverse_static_pwm,
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
        message_type = (
            TwistStamped if self.use_stamped_cmd_vel else Twist
        )
        callback = (
            self._stamped_cmd_vel_callback
            if self.use_stamped_cmd_vel
            else self._cmd_vel_callback
        )
        # Keep the standard velocity interface discoverable to tools such as
        # Foxglove even while motor output is disarmed.  The callback and the
        # serial writer both enforce the independent motor-output gate.
        self.cmd_vel_subscription = self.create_subscription(
            message_type,
            self.cmd_vel_topic,
            callback,
            10,
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
        self.firmware_protocol_version = None
        self.firmware_ticks_per_revolution = None
        self.firmware_max_pwm = None
        self.firmware_max_target = None
        self.controller_configured = False
        self.controller_errors = 0
        self.target_left_ticks_per_second = 0
        self.target_right_ticks_per_second = 0
        self.last_cmd_vel_time = None
        self.last_command_send_time = 0.0
        self.stop_command_sent = True
        self.command_start_left_ticks = None
        self.command_start_right_ticks = None
        self.command_started_at = None
        self.motor_commands_sent = 0
        self.limited_commands = 0
        self.rejected_commands = 0

        self.poll_timer = self.create_timer(0.02, self._poll_serial)
        self.diagnostic_timer = self.create_timer(
            1.0, self._publish_diagnostics
        )

        if self.enable_motor_control:
            command_type = (
                'TwistStamped' if self.use_stamped_cmd_vel else 'Twist'
            )
            self.get_logger().warning(
                'Motor control is enabled but will remain stopped until a '
                f'fresh {command_type} arrives on {self.cmd_vel_topic}. '
                f'The ROS timeout is {1000.0 * self.command_timeout:.0f} ms '
                'and firmware watchdog is 350 ms.'
            )
        else:
            self.get_logger().info(
                'Encoder odometry is read-only: no motor commands will be '
                f'sent. Commands on {self.cmd_vel_topic} are ignored.'
            )

        self.get_logger().info(
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
            self.firmware_protocol_version = None
            self.firmware_ticks_per_revolution = None
            self.firmware_max_pwm = None
            self.firmware_max_target = None
            self.controller_configured = False
            self.target_left_ticks_per_second = 0
            self.target_right_ticks_per_second = 0
            self.last_cmd_vel_time = None
            self.last_command_send_time = 0.0
            self.stop_command_sent = True
            self.command_start_left_ticks = None
            self.command_start_right_ticks = None
            self.command_started_at = None
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

    def _disconnect(self, reason, attempt_stop=True):
        if self.connection is not None:
            if self.enable_motor_control and attempt_stop:
                try:
                    self.connection.write(b'STOP\n')
                    self.connection.flush()
                except (OSError, serial.SerialException):
                    pass
            try:
                self.connection.close()
            except (OSError, serial.SerialException):
                pass

        self.connection = None
        self.controller_configured = False
        self.target_left_ticks_per_second = 0
        self.target_right_ticks_per_second = 0
        self.last_cmd_vel_time = None
        self.last_command_send_time = 0.0
        self.stop_command_sent = True
        self.command_start_left_ticks = None
        self.command_start_right_ticks = None
        self.command_started_at = None
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
                return

            self._refresh_motor_command()
        except (OSError, serial.SerialException) as error:
            self._disconnect(str(error), attempt_stop=False)

    def _handle_line(self, line):
        if not line:
            return

        if not line.startswith('T,'):
            self.last_controller_message = line
            self._handle_controller_message(line)
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
        if telemetry.left_target_ticks_per_second is not None:
            self.firmware_protocol_version = 2
            self._ensure_controller_configured()
        update = self.tracker.update(telemetry)
        if update is None:
            return

        self.last_update = update
        self._publish_state(update)

    def _handle_controller_message(self, line):
        if line.startswith('READY,ROAM_E_MOTOR_CONTROLLER,'):
            try:
                self.firmware_protocol_version = int(
                    line.rsplit(',', 1)[1]
                )
            except (IndexError, ValueError):
                self.controller_errors += 1
                return

            self._ensure_controller_configured()
            return

        if line.startswith('INFO,'):
            fields = line.split(',')
            if len(fields) != 5:
                self.controller_errors += 1
                return

            try:
                self.firmware_protocol_version = int(fields[1])
                self.firmware_ticks_per_revolution = int(fields[2])
                self.firmware_max_pwm = int(fields[3])
                self.firmware_max_target = int(fields[4])
            except ValueError:
                self.controller_errors += 1
            return

        if line.startswith('ERR,'):
            self.controller_errors += 1
            self._warn_throttled(f'Motor controller reported {line}')

    def _write_command(self, command):
        if not self.enable_motor_control or self.connection is None:
            return False

        try:
            self.connection.write(command)
            self.connection.flush()
            self.motor_commands_sent += 1
            return True
        except (OSError, serial.SerialException) as error:
            self._disconnect(str(error), attempt_stop=False)
            return False

    def _ensure_controller_configured(self):
        if (
            not self.enable_motor_control
            or self.controller_configured
            or self.firmware_protocol_version is None
        ):
            return

        if self.firmware_protocol_version < 2:
            self._warn_throttled(
                'Motor control requires Nano firmware protocol v2; '
                f'found v{self.firmware_protocol_version}'
            )
            return

        if not self._write_command(b'STOP\n'):
            return
        if not self._write_command(b'INFO\n'):
            return
        if not self._write_command(self.pid_configuration_command):
            return
        if not self._write_command(self.feedforward_configuration_command):
            return

        # Never reuse a velocity received before the firmware was proven
        # compatible. A fresh cmd_vel is required after configuration.
        self.target_left_ticks_per_second = 0
        self.target_right_ticks_per_second = 0
        self.last_cmd_vel_time = None
        self.last_command_send_time = 0.0
        self.controller_configured = True
        self.stop_command_sent = True
        self.get_logger().info(
            'Motor controller protocol v2 configured and stopped; '
            'waiting for fresh cmd_vel input.'
        )

    def _stamped_cmd_vel_callback(self, message):
        self._accept_twist(message.twist)

    def _cmd_vel_callback(self, message):
        self._accept_twist(message)

    def _accept_twist(self, twist):
        if not self.enable_motor_control:
            return

        now = time.monotonic()
        try:
            targets = twist_to_wheel_targets(
                linear_velocity=float(twist.linear.x),
                angular_velocity=float(twist.angular.z),
                wheel_separation=self.tracker.wheel_separation,
                metres_per_tick=self.tracker.metres_per_tick,
                max_linear_velocity=self.max_linear_velocity,
                max_angular_velocity=self.max_angular_velocity,
                max_wheel_ticks_per_second=(
                    self.max_wheel_ticks_per_second
                ),
            )
        except ValueError as error:
            self.rejected_commands += 1
            self.target_left_ticks_per_second = 0
            self.target_right_ticks_per_second = 0
            self.last_cmd_vel_time = None
            self.stop_command_sent = False
            self._warn_throttled(f'Rejected cmd_vel: {error}')
            return

        command_was_active = (
            self.last_cmd_vel_time is not None
            and now - self.last_cmd_vel_time <= self.command_timeout
            and not self.stop_command_sent
        )
        if not command_was_active and self.last_telemetry is not None:
            self.command_start_left_ticks = self.last_telemetry.left_ticks
            self.command_start_right_ticks = self.last_telemetry.right_ticks
            self.command_started_at = now

        self.target_left_ticks_per_second = (
            targets.left_ticks_per_second
        )
        self.target_right_ticks_per_second = (
            targets.right_ticks_per_second
        )
        self.last_cmd_vel_time = now
        self.stop_command_sent = False

        if targets.limited:
            self.limited_commands += 1

    def _refresh_motor_command(self):
        if not self.enable_motor_control or not self.controller_configured:
            return

        now = time.monotonic()
        if now - self.last_command_send_time < 1.0 / self.command_rate:
            return

        command_is_fresh = (
            self.last_cmd_vel_time is not None
            and now - self.last_cmd_vel_time <= self.command_timeout
        )

        if command_is_fresh:
            left_target = self.target_left_ticks_per_second
            right_target = self.target_right_ticks_per_second
        else:
            left_target = 0
            right_target = 0

        if left_target == 0 and right_target == 0:
            if self.stop_command_sent:
                return

            sent = self._write_command(velocity_command(0, 0))
            if sent:
                self.stop_command_sent = True
                self.last_command_send_time = now
                self._log_completed_command()
            return

        sent = self._write_command(
            velocity_command(left_target, right_target)
        )
        if sent:
            self.stop_command_sent = False
            self.last_command_send_time = now

    def _log_completed_command(self):
        telemetry = self.last_telemetry
        if (
            telemetry is None
            or self.command_start_left_ticks is None
            or self.command_start_right_ticks is None
        ):
            return

        left_delta = signed_counter_delta(
            telemetry.left_ticks,
            self.command_start_left_ticks,
        )
        right_delta = signed_counter_delta(
            telemetry.right_ticks,
            self.command_start_right_ticks,
        )
        distance = 0.5 * (left_delta + right_delta) * (
            self.tracker.metres_per_tick
        )
        yaw = (right_delta - left_delta) * (
            self.tracker.metres_per_tick / self.tracker.wheel_separation
        )
        elapsed = (
            time.monotonic() - self.command_started_at
            if self.command_started_at is not None
            else float('nan')
        )
        self.get_logger().info(
            'Motor command stopped: '
            f'elapsed={elapsed:.3f} s, '
            f'left_delta={left_delta} ticks, '
            f'right_delta={right_delta} ticks, '
            f'distance={distance:.4f} m, yaw={yaw:.4f} rad.'
            f' final_target=('
            f'{telemetry.left_target_ticks_per_second},'
            f'{telemetry.right_target_ticks_per_second}) tps,'
            f' final_measured=('
            f'{telemetry.left_measured_ticks_per_second},'
            f'{telemetry.right_measured_ticks_per_second}) tps,'
            f' final_pwm=({telemetry.left_pwm},{telemetry.right_pwm}).'
        )
        self.command_start_left_ticks = None
        self.command_start_right_ticks = None
        self.command_started_at = None

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
        diagnostic.name = 'Roam-E drive base'
        diagnostic.hardware_id = f'arduino-nano:{self.serial_port}'

        telemetry_age = (
            time.monotonic() - self.last_telemetry_time
            if self.last_telemetry_time is not None
            else float('inf')
        )

        firmware_tpr_mismatch = (
            self.firmware_ticks_per_revolution is not None
            and self.firmware_ticks_per_revolution
            != round(self.tracker.ticks_per_revolution)
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
        elif firmware_tpr_mismatch:
            diagnostic.level = DiagnosticStatus.ERROR
            diagnostic.message = 'Firmware and ROS encoder scales disagree'
        elif (
            self.enable_motor_control
            and self.firmware_protocol_version is not None
            and self.firmware_protocol_version < 2
        ):
            diagnostic.level = DiagnosticStatus.ERROR
            diagnostic.message = 'Firmware lacks velocity-control protocol v2'
        elif self.enable_motor_control and not self.controller_configured:
            diagnostic.level = DiagnosticStatus.WARN
            diagnostic.message = 'Waiting to configure motor controller'
        else:
            diagnostic.level = DiagnosticStatus.OK
            diagnostic.message = (
                'Encoder telemetry and motor control ready'
                if self.enable_motor_control
                else 'Encoder telemetry active; motor control disabled'
            )

        telemetry = self.last_telemetry
        update = self.last_update
        command_age = (
            time.monotonic() - self.last_cmd_vel_time
            if self.last_cmd_vel_time is not None
            else float('inf')
        )
        diagnostic.values = [
            diagnostic_value('serial_port', self.serial_port),
            diagnostic_value('baud_rate', self.baud_rate),
            diagnostic_value(
                'read_only', str(not self.enable_motor_control).lower()
            ),
            diagnostic_value(
                'motor_control_enabled',
                str(self.enable_motor_control).lower(),
            ),
            diagnostic_value(
                'controller_configured',
                str(self.controller_configured).lower(),
            ),
            diagnostic_value(
                'firmware_protocol_version',
                self.firmware_protocol_version or 'unknown',
            ),
            diagnostic_value(
                'firmware_ticks_per_revolution',
                self.firmware_ticks_per_revolution or 'unknown',
            ),
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
            diagnostic_value('controller_errors', self.controller_errors),
            diagnostic_value('serial_connections', self.reconnects),
            diagnostic_value('cmd_vel_topic', self.cmd_vel_topic),
            diagnostic_value(
                'cmd_vel_type',
                'TwistStamped' if self.use_stamped_cmd_vel else 'Twist',
            ),
            diagnostic_value('command_age_s', f'{command_age:.3f}'),
            diagnostic_value('command_timeout_s', self.command_timeout),
            diagnostic_value(
                'left_forward_static_pwm', self.left_forward_static_pwm
            ),
            diagnostic_value(
                'left_reverse_static_pwm', self.left_reverse_static_pwm
            ),
            diagnostic_value(
                'right_forward_static_pwm', self.right_forward_static_pwm
            ),
            diagnostic_value(
                'right_reverse_static_pwm', self.right_reverse_static_pwm
            ),
            diagnostic_value(
                'ros_left_target_ticks_per_second',
                self.target_left_ticks_per_second,
            ),
            diagnostic_value(
                'ros_right_target_ticks_per_second',
                self.target_right_ticks_per_second,
            ),
            diagnostic_value('motor_commands_sent', self.motor_commands_sent),
            diagnostic_value('limited_commands', self.limited_commands),
            diagnostic_value('rejected_commands', self.rejected_commands),
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
                'firmware_left_target_ticks_per_second',
                telemetry.left_target_ticks_per_second
                if telemetry
                else 'none',
            ),
            diagnostic_value(
                'firmware_right_target_ticks_per_second',
                telemetry.right_target_ticks_per_second
                if telemetry
                else 'none',
            ),
            diagnostic_value(
                'firmware_left_measured_ticks_per_second',
                telemetry.left_measured_ticks_per_second
                if telemetry
                else 'none',
            ),
            diagnostic_value(
                'firmware_right_measured_ticks_per_second',
                telemetry.right_measured_ticks_per_second
                if telemetry
                else 'none',
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
            if self.enable_motor_control:
                try:
                    self.connection.write(b'STOP\n')
                    self.connection.flush()
                except (OSError, serial.SerialException):
                    pass
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
