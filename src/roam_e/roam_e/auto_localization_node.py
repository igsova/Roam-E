"""Automatic AMCL global localization on a saved map.

Asks AMCL to spread its particles over the whole map, then drives a short,
guarded pattern (forward, a 90-degree turn, forward) on the Nav2 velocity input
until the pose estimate converges. It checks sensor freshness, motion progress
and pose jumps, stops on any failure, clears the costmaps on success and then
exits. Motion is only allowed with allow_motion:=true.
"""

import math
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty

from roam_e.localization_convergence import (
    ConvergenceTracker,
    normalize_angle,
    planar_distance,
    yaw_from_quaternion,
)


class AutoLocalizationNode(Node):
    """Run one guarded AMCL global-localization search and then stop."""

    def __init__(self):
        super().__init__('auto_localization_node')

        self.declare_parameter('allow_motion', False)
        self.declare_parameter('cmd_vel_topic', 'cmd_vel_nav')
        self.declare_parameter('amcl_pose_topic', 'amcl_pose')
        self.declare_parameter('odom_topic', 'odometry/filtered')
        self.declare_parameter('scan_topic', 'scan')
        self.declare_parameter('initial_settle_time', 3.0)
        self.declare_parameter('motion_settle_time', 3.0)
        self.declare_parameter('forward_distance', 0.20)
        self.declare_parameter('forward_speed', 0.04)
        self.declare_parameter('turn_angle', math.pi / 2.0)
        self.declare_parameter('turn_speed', 0.20)
        self.declare_parameter('motion_timeout', 15.0)
        self.declare_parameter('stall_timeout', 2.5)
        self.declare_parameter('sensor_timeout', 1.0)
        self.declare_parameter('max_x_variance', 0.04)
        self.declare_parameter('max_y_variance', 0.04)
        self.declare_parameter('max_yaw_variance', 0.09)
        self.declare_parameter('required_stable_samples', 3)
        self.declare_parameter('max_position_jump', 0.15)
        self.declare_parameter('max_yaw_jump', 0.35)

        self._allow_motion = bool(
            self.get_parameter('allow_motion').value
        )
        self._initial_settle_time = float(
            self.get_parameter('initial_settle_time').value
        )
        self._motion_settle_time = float(
            self.get_parameter('motion_settle_time').value
        )
        self._forward_distance = float(
            self.get_parameter('forward_distance').value
        )
        self._forward_speed = float(
            self.get_parameter('forward_speed').value
        )
        self._turn_angle = float(
            self.get_parameter('turn_angle').value
        )
        self._turn_speed = float(
            self.get_parameter('turn_speed').value
        )
        self._motion_timeout = float(
            self.get_parameter('motion_timeout').value
        )
        self._stall_timeout = float(
            self.get_parameter('stall_timeout').value
        )
        self._sensor_timeout = float(
            self.get_parameter('sensor_timeout').value
        )

        self._tracker = ConvergenceTracker(
            max_x_variance=self.get_parameter(
                'max_x_variance'
            ).value,
            max_y_variance=self.get_parameter(
                'max_y_variance'
            ).value,
            max_yaw_variance=self.get_parameter(
                'max_yaw_variance'
            ).value,
            required_samples=self.get_parameter(
                'required_stable_samples'
            ).value,
            max_position_jump=self.get_parameter(
                'max_position_jump'
            ).value,
            max_yaw_jump=self.get_parameter('max_yaw_jump').value,
        )

        self._command_publisher = self.create_publisher(
            Twist,
            self.get_parameter('cmd_vel_topic').value,
            10,
        )
        self._diagnostic_publisher = self.create_publisher(
            DiagnosticArray,
            'diagnostics',
            10,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            self.get_parameter('amcl_pose_topic').value,
            self._on_amcl_pose,
            10,
        )
        self.create_subscription(
            Odometry,
            self.get_parameter('odom_topic').value,
            self._on_odometry,
            20,
        )
        self.create_subscription(
            LaserScan,
            self.get_parameter('scan_topic').value,
            self._on_scan,
            qos_profile_sensor_data,
        )

        self._global_client = self.create_client(
            Empty,
            'reinitialize_global_localization',
        )
        self._nomotion_client = self.create_client(
            Empty,
            'request_nomotion_update',
        )
        self._safety_clients = {
            'velocity_smoother': self.create_client(
                GetState,
                'velocity_smoother/get_state',
            ),
            'collision_monitor': self.create_client(
                GetState,
                'collision_monitor/get_state',
            ),
        }
        self._safety_futures = {
            name: None for name in self._safety_clients
        }
        self._safety_active = {
            name: False for name in self._safety_clients
        }
        self._clear_clients = {
            'global': self.create_client(
                ClearEntireCostmap,
                'global_costmap/clear_entirely_global_costmap',
            ),
            'local': self.create_client(
                ClearEntireCostmap,
                'local_costmap/clear_entirely_local_costmap',
            ),
        }
        self._clear_futures = {
            name: None for name in self._clear_clients
        }
        self._clear_done = {
            name: False for name in self._clear_clients
        }

        self._state = 'waiting_for_dependencies'
        self._failure_reason = ''
        self._global_future = None
        self._nomotion_future = None
        self._last_nomotion_request = 0.0
        self._last_scan_time = None
        self._last_odom_time = None
        self._odom_pose = None
        self._last_turn_yaw = None
        self._turn_accumulated = 0.0
        self._motion_start = None
        self._motion_deadline = None
        self._last_progress = 0.0
        self._last_progress_time = None
        self._state_deadline = None
        self._last_diagnostic_time = 0.0

        self.create_timer(0.10, self._tick)
        self.get_logger().warning(
            'Automatic global localization is enabled. Motion permission '
            f'is {self._allow_motion}; waiting for the independent safety '
            'command chain and fresh sensors.'
        )

    def _on_scan(self, _message):
        self._last_scan_time = time.monotonic()

    def _on_odometry(self, message):
        now = time.monotonic()
        pose = message.pose.pose
        yaw = yaw_from_quaternion(
            pose.orientation.z,
            pose.orientation.w,
        )
        self._odom_pose = (pose.position.x, pose.position.y, yaw)
        self._last_odom_time = now

        if self._state == 'turning' and self._last_turn_yaw is not None:
            increment = normalize_angle(yaw - self._last_turn_yaw)
            self._turn_accumulated += abs(increment)
            self._last_turn_yaw = yaw

    def _on_amcl_pose(self, message):
        pose = message.pose.pose
        yaw = yaw_from_quaternion(
            pose.orientation.z,
            pose.orientation.w,
        )
        quality = self._tracker.update(
            x=pose.position.x,
            y=pose.position.y,
            yaw=yaw,
            covariance=message.pose.covariance,
        )
        if quality.converged and self._state not in (
            'clearing_costmaps',
            'complete',
        ):
            self.get_logger().info(
                'AMCL convergence criteria met: '
                f'variance=({quality.x_variance:.4f}, '
                f'{quality.y_variance:.4f}, '
                f'{quality.yaw_variance:.4f}), '
                f'stable_samples={quality.stable_samples}.'
            )

    def _tick(self):
        now = time.monotonic()
        if now - self._last_diagnostic_time >= 1.0:
            self._publish_diagnostics()
            self._last_diagnostic_time = now

        if self._state in ('complete', 'failed'):
            return

        if self._state == 'waiting_for_dependencies':
            self._tick_waiting(now)
        elif self._state == 'requesting_global_localization':
            self._tick_global_request(now)
        elif self._state.startswith('settling_'):
            self._tick_settling(now)
        elif self._state in ('forward_1', 'forward_2'):
            self._tick_forward(now)
        elif self._state == 'turning':
            self._tick_turn(now)
        elif self._state == 'clearing_costmaps':
            self._tick_clear_costmaps(now)

    def _tick_waiting(self, now):
        if not self._poll_safety_state():
            return
        if not self._fresh(self._last_scan_time, now):
            return
        if not self._fresh(self._last_odom_time, now):
            return
        if not self._global_client.service_is_ready():
            return

        self._tracker.reset()
        self._global_future = self._global_client.call_async(
            Empty.Request()
        )
        self._state = 'requesting_global_localization'
        self.get_logger().info(
            'Safety nodes are active and sensors are fresh; requesting one '
            'uniform AMCL global-localization search.'
        )

    def _tick_global_request(self, now):
        if self._global_future is None or not self._global_future.done():
            return
        try:
            self._global_future.result()
        except Exception as error:  # noqa: BLE001 - ROS futures vary
            self._fail(f'global-localization service failed: {error}')
            return

        self._state = 'settling_initial'
        self._state_deadline = now + self._initial_settle_time
        self.get_logger().info(
            'Global hypotheses initialized; collecting stationary scan '
            'updates before requesting motion.'
        )

    def _tick_settling(self, now):
        self._publish_stop()
        self._request_nomotion_update(now)
        if self._tracker.quality.converged:
            self._succeed(now)
            return
        if now < self._state_deadline:
            return

        if self._state == 'settling_initial':
            self._start_forward('forward_1', now)
        elif self._state == 'settling_forward_1':
            self._start_turn(now)
        elif self._state == 'settling_turn':
            self._start_forward('forward_2', now)
        else:
            self._fail(
                'AMCL did not converge after the complete guarded search '
                'pattern; manual assistance is required.'
            )

    def _start_forward(self, state, now):
        if not self._allow_motion:
            self._fail(
                'AMCL needs a changed viewpoint, but motor motion was not '
                'authorized. Relaunch with both arm_motors:=true and '
                'auto_localize:=true in a clear, supervised area.'
            )
            return
        if self._odom_pose is None:
            self._fail('odometry disappeared before forward motion')
            return

        self._state = state
        self._motion_start = self._odom_pose[:2]
        self._motion_deadline = now + self._motion_timeout
        self._last_progress = 0.0
        self._last_progress_time = now
        leg_number = 1 if state == 'forward_1' else 2
        self.get_logger().warning(
            f'Starting guarded localization leg {leg_number}: '
            f'{self._forward_distance:.2f} m forward at '
            f'{self._forward_speed:.2f} m/s.'
        )

    def _tick_forward(self, now):
        if self._tracker.quality.converged:
            self._succeed(now)
            return
        if not self._motion_sensors_fresh(now):
            return

        distance = planar_distance(
            self._motion_start,
            self._odom_pose[:2],
        )
        if distance >= self._forward_distance:
            next_state = (
                'settling_forward_1'
                if self._state == 'forward_1'
                else 'settling_forward_2'
            )
            self._publish_stop()
            self._state = next_state
            self._state_deadline = now + self._motion_settle_time
            self.get_logger().info(
                f'Forward localization leg completed at {distance:.3f} m; '
                'collecting stopped scan updates.'
            )
            return

        if distance >= self._last_progress + 0.005:
            self._last_progress = distance
            self._last_progress_time = now
        if now - self._last_progress_time > self._stall_timeout:
            self._fail(
                'forward localization motion stalled, likely because the '
                'collision monitor stopped for a nearby obstacle'
            )
            return
        if now >= self._motion_deadline:
            self._fail('forward localization motion timed out')
            return

        command = Twist()
        command.linear.x = self._forward_speed
        self._command_publisher.publish(command)

    def _start_turn(self, now):
        if self._odom_pose is None:
            self._fail('odometry disappeared before turn')
            return
        self._state = 'turning'
        self._last_turn_yaw = self._odom_pose[2]
        self._turn_accumulated = 0.0
        self._motion_deadline = now + self._motion_timeout
        self._last_progress = 0.0
        self._last_progress_time = now
        self.get_logger().warning(
            'Starting guarded left turn for localization: '
            f'{math.degrees(self._turn_angle):.0f} degrees at '
            f'{self._turn_speed:.2f} rad/s.'
        )

    def _tick_turn(self, now):
        if self._tracker.quality.converged:
            self._succeed(now)
            return
        if not self._motion_sensors_fresh(now):
            return
        if self._turn_accumulated >= self._turn_angle:
            self._publish_stop()
            self._state = 'settling_turn'
            self._state_deadline = now + self._motion_settle_time
            self.get_logger().info(
                'Localization turn completed at '
                f'{math.degrees(self._turn_accumulated):.1f} degrees; '
                'collecting stopped scan updates.'
            )
            return

        if self._turn_accumulated >= self._last_progress + 0.01:
            self._last_progress = self._turn_accumulated
            self._last_progress_time = now
        if now - self._last_progress_time > self._stall_timeout:
            self._fail(
                'turning stalled, likely because the collision monitor '
                'stopped for a nearby obstacle'
            )
            return
        if now >= self._motion_deadline:
            self._fail('localization turn timed out')
            return

        command = Twist()
        command.angular.z = self._turn_speed
        self._command_publisher.publish(command)

    def _motion_sensors_fresh(self, now):
        if not self._fresh(self._last_scan_time, now):
            self._fail('LiDAR scan became stale during localization motion')
            return False
        if not self._fresh(self._last_odom_time, now):
            self._fail('odometry became stale during localization motion')
            return False
        return True

    def _succeed(self, now):
        self._publish_stop()
        self._state = 'clearing_costmaps'
        self._state_deadline = now + 20.0
        quality = self._tracker.quality
        self.get_logger().info(
            'AUTOMATIC LOCALIZATION VERIFIED; stopping all search motion '
            'and clearing Nav2 costmaps. Final variance='
            f'({quality.x_variance:.4f}, {quality.y_variance:.4f}, '
            f'{quality.yaw_variance:.4f}).'
        )

    def _tick_clear_costmaps(self, now):
        self._publish_stop()
        for name, client in self._clear_clients.items():
            if self._clear_done[name]:
                continue
            future = self._clear_futures[name]
            if future is None:
                if client.service_is_ready():
                    self._clear_futures[name] = client.call_async(
                        ClearEntireCostmap.Request()
                    )
                continue
            if not future.done():
                continue
            try:
                future.result()
            except Exception as error:  # noqa: BLE001 - ROS futures vary
                self.get_logger().warning(
                    f'Could not clear {name} costmap yet: {error}'
                )
                self._clear_futures[name] = None
                continue
            self._clear_done[name] = True

        if all(self._clear_done.values()):
            self._state = 'complete'
            self.get_logger().info(
                'Automatic localization complete; both costmaps cleared '
                'and navigation goals may now be accepted.'
            )
        elif now >= self._state_deadline:
            self._state = 'complete'
            self.get_logger().warning(
                'Localization converged, but one or more costmap clear '
                'services did not become ready within 20 seconds.'
            )

    def _request_nomotion_update(self, now):
        if now - self._last_nomotion_request < 0.5:
            return
        if self._nomotion_future is not None:
            if not self._nomotion_future.done():
                return
            self._nomotion_future = None
        if not self._nomotion_client.service_is_ready():
            return
        self._nomotion_future = self._nomotion_client.call_async(
            Empty.Request()
        )
        self._last_nomotion_request = now

    def _poll_safety_state(self):
        for name, client in self._safety_clients.items():
            if self._safety_active[name]:
                continue
            future = self._safety_futures[name]
            if future is None:
                if client.service_is_ready():
                    self._safety_futures[name] = client.call_async(
                        GetState.Request()
                    )
                continue
            if not future.done():
                continue
            try:
                response = future.result()
                self._safety_active[name] = (
                    response.current_state.id
                    == State.PRIMARY_STATE_ACTIVE
                )
            except Exception:  # noqa: BLE001 - retry transient ROS errors
                self._safety_active[name] = False
            self._safety_futures[name] = None
        return all(self._safety_active.values())

    def _fresh(self, timestamp, now):
        return (
            timestamp is not None
            and now - timestamp <= self._sensor_timeout
        )

    def _publish_stop(self):
        self._command_publisher.publish(Twist())

    def _fail(self, reason):
        self._publish_stop()
        self._failure_reason = reason
        self._state = 'failed'
        self.get_logger().error(
            f'AUTOMATIC LOCALIZATION STOPPED: {reason}. The robot will '
            'remain stopped and navigation goals stay locked.'
        )

    def _publish_diagnostics(self):
        quality = self._tracker.quality
        if self._state == 'complete':
            level = DiagnosticStatus.OK
            message = 'Localized automatically; navigation unlocked'
        elif self._state == 'failed':
            level = DiagnosticStatus.ERROR
            message = self._failure_reason
        else:
            level = DiagnosticStatus.WARN
            message = f'Automatic localization in progress: {self._state}'

        status = DiagnosticStatus()
        status.level = level
        status.name = 'Roam-E automatic localization'
        status.hardware_id = 'amcl:office-map'
        status.message = message
        status.values = [
            KeyValue(key='state', value=self._state),
            KeyValue(
                key='motion_authorized',
                value=str(self._allow_motion).lower(),
            ),
            KeyValue(
                key='stable_samples',
                value=str(quality.stable_samples),
            ),
            KeyValue(
                key='x_variance',
                value=f'{quality.x_variance:.6f}',
            ),
            KeyValue(
                key='y_variance',
                value=f'{quality.y_variance:.6f}',
            ),
            KeyValue(
                key='yaw_variance',
                value=f'{quality.yaw_variance:.6f}',
            ),
        ]
        message_out = DiagnosticArray()
        message_out.header.stamp = self.get_clock().now().to_msg()
        message_out.status = [status]
        self._diagnostic_publisher.publish(message_out)

    def destroy_node(self):
        self._publish_stop()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AutoLocalizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError:
        if rclpy.ok():
            raise
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
