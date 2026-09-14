import time

from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
import rclpy

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ManageLifecycleNodes
from tf2_ros import Buffer, TransformListener

from roam_e.localization_convergence import (
    ConvergenceTracker,
    yaw_from_quaternion,
)


class GoalPoseBridge(Node):
    """Start Nav2 after localization and forward Foxglove pose goals."""

    def __init__(self):
        super().__init__('goal_pose_bridge')
        self.declare_parameter('goal_topic', 'goal_pose')
        self.declare_parameter('action_name', 'navigate_to_pose')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('start_navigation_when_localized', False)
        self.declare_parameter(
            'lifecycle_manager_name',
            'lifecycle_manager_navigation',
        )
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('startup_check_period', 0.5)
        self.declare_parameter('amcl_pose_topic', 'amcl_pose')
        self.declare_parameter('max_x_variance', 0.04)
        self.declare_parameter('max_y_variance', 0.04)
        self.declare_parameter('max_yaw_variance', 0.09)
        self.declare_parameter('required_stable_samples', 3)
        self.declare_parameter('max_position_jump', 0.15)
        self.declare_parameter('max_yaw_jump', 0.35)
        self.declare_parameter('goal_timeout', 180.0)

        goal_topic = self.get_parameter('goal_topic').value
        action_name = self.get_parameter('action_name').value
        self._global_frame = self.get_parameter('global_frame').value
        self._base_frame = self.get_parameter('base_frame').value
        self._action_client = ActionClient(
            self,
            NavigateToPose,
            action_name,
        )
        self._active_goal_handle = None
        self._active_goal_started_at = None
        self._goal_cancel_requested = False
        self._goal_timeout = float(self.get_parameter('goal_timeout').value)
        if self._goal_timeout <= 0.0:
            raise ValueError('goal_timeout must be greater than zero')
        self.create_timer(0.25, self._enforce_goal_timeout)
        self.create_subscription(PoseStamped, goal_topic, self._on_goal, 10)
        self._localization_tracker = ConvergenceTracker(
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
        self._localization_ready = False
        self.create_subscription(
            PoseWithCovarianceStamped,
            self.get_parameter('amcl_pose_topic').value,
            self._on_localization,
            10,
        )

        self.get_logger().info(
            f'Forwarding {goal_topic} PoseStamped messages to '
            f'{action_name}'
        )

        self._startup_complete = not self.get_parameter(
            'start_navigation_when_localized'
        ).value
        self._startup_future = None
        self._startup_timer = None
        if not self._startup_complete:
            manager_name = self.get_parameter(
                'lifecycle_manager_name'
            ).value.strip('/')
            self._startup_service = f'/{manager_name}/manage_nodes'
            self._startup_client = self.create_client(
                ManageLifecycleNodes,
                self._startup_service,
            )
            self._tf_buffer = Buffer()
            self._tf_listener = TransformListener(self._tf_buffer, self)
            self._startup_timer = self.create_timer(
                self.get_parameter('startup_check_period').value,
                self._maybe_start_navigation,
            )
            self.get_logger().info(
                'Navigation startup is gated until AMCL covariance is low '
                'and stable and localization provides '
                f'{self._global_frame} -> {self._base_frame}.'
            )

    def _on_localization(self, message):
        pose = message.pose.pose
        quality = self._localization_tracker.update(
            x=pose.position.x,
            y=pose.position.y,
            yaw=yaw_from_quaternion(
                pose.orientation.z,
                pose.orientation.w,
            ),
            covariance=message.pose.covariance,
        )
        self._set_localization_ready(quality.converged)

    def _set_localization_ready(self, ready):
        if ready == self._localization_ready:
            return
        self._localization_ready = ready
        if ready:
            quality = self._localization_tracker.quality
            self.get_logger().info(
                'Localization quality gate passed: '
                f'variance=({quality.x_variance:.4f}, '
                f'{quality.y_variance:.4f}, '
                f'{quality.yaw_variance:.4f}).'
            )
            return

        self.get_logger().error(
            'Localization quality gate was lost; rejecting new navigation '
            'goals and cancelling the active goal, if any.'
        )
        self._cancel_active_goal('Cancelling goal after localization loss.')

    def _localization_is_ready(self):
        # AMCL publishes amcl_pose when its filter updates, not as a
        # stationary heartbeat. Keep a verified estimate latched between
        # updates; a later imprecise or inconsistent estimate still revokes
        # the gate in _on_localization().
        return self._localization_ready

    def _maybe_start_navigation(self):
        localization_ready = self._localization_is_ready()
        if self._startup_complete:
            return
        if self._startup_future is not None:
            return
        if not localization_ready:
            return
        if not self._startup_client.service_is_ready():
            return
        if not self._tf_buffer.can_transform(
            self._global_frame,
            self._base_frame,
            Time(),
            timeout=Duration(seconds=0.0),
        ):
            return

        request = ManageLifecycleNodes.Request()
        request.command = ManageLifecycleNodes.Request.STARTUP
        self._startup_future = self._startup_client.call_async(request)
        self._startup_future.add_done_callback(
            self._on_navigation_startup_response
        )
        self.get_logger().info(
            'Localization transform is available; requesting Nav2 startup.'
        )

    def _on_navigation_startup_response(self, future):
        try:
            response = future.result()
        except Exception as error:  # noqa: BLE001 - ROS future exceptions vary
            self.get_logger().warning(
                f'Nav2 startup request failed: {error}; retrying.'
            )
            self._startup_future = None
            return

        if not response.success:
            self.get_logger().warning(
                'Nav2 lifecycle manager rejected startup; retrying.'
            )
            self._startup_future = None
            return

        self._startup_complete = True
        self.get_logger().info(
            'Nav2 lifecycle nodes are active and ready for goals.'
        )

    def _on_goal(self, pose):
        if not self._localization_is_ready():
            self.get_logger().warning(
                'Rejected goal because AMCL localization is not precise and '
                'stable yet.'
            )
            return
        if not pose.header.frame_id:
            pose.header.frame_id = self._global_frame
        if pose.header.frame_id != self._global_frame:
            self.get_logger().error(
                f'Rejected goal in frame {pose.header.frame_id!r}; '
                f'expected {self._global_frame!r}'
            )
            return
        if not self._action_client.server_is_ready():
            self.get_logger().warning(
                'NavigateToPose is not ready; wait for the Nav2 lifecycle '
                'nodes to become active and click the goal again.'
            )
            return

        self._cancel_active_goal('Cancelling the previous navigation goal.')

        goal = NavigateToPose.Goal()
        goal.pose = pose
        future = self._action_client.send_goal_async(goal)
        future.add_done_callback(self._on_goal_response)
        self.get_logger().info(
            'Sent Foxglove goal: '
            f'x={pose.pose.position.x:.3f}, '
            f'y={pose.pose.position.y:.3f}'
        )

    def _on_goal_response(self, future):
        try:
            goal_handle = future.result()
        except Exception as error:  # noqa: BLE001 - ROS future exceptions vary
            self.get_logger().error(f'Failed to send navigation goal: {error}')
            return
        if not goal_handle.accepted:
            self.get_logger().warning('Nav2 rejected the goal.')
            return

        self._active_goal_handle = goal_handle
        self._active_goal_started_at = time.monotonic()
        self._goal_cancel_requested = False
        self.get_logger().info(
            f'Navigation goal accepted; timeout is {self._goal_timeout:.1f} s.'
        )
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda result, handle=goal_handle: self._on_result(
                result,
                handle,
            )
        )

    def _on_result(self, future, goal_handle):
        try:
            result = future.result()
        except Exception as error:  # noqa: BLE001 - ROS future exceptions vary
            self.get_logger().error(
                f'Failed to receive NavigateToPose result: {error}'
            )
            result = None
        if result is not None:
            self.get_logger().info(
                f'NavigateToPose finished with status {result.status}.'
            )
        if self._active_goal_handle == goal_handle:
            self._active_goal_handle = None
            self._active_goal_started_at = None
            self._goal_cancel_requested = False

    def _enforce_goal_timeout(self):
        if self._active_goal_handle is None:
            return
        if self._active_goal_started_at is None:
            return
        if self._goal_cancel_requested:
            return

        elapsed = time.monotonic() - self._active_goal_started_at
        if elapsed < self._goal_timeout:
            return

        self._cancel_active_goal(
            f'Navigation goal exceeded the {self._goal_timeout:.1f} s '
            'mission timeout; cancelling it.'
        )

    def _cancel_active_goal(self, reason):
        goal_handle = self._active_goal_handle
        if goal_handle is None or self._goal_cancel_requested:
            return

        self._goal_cancel_requested = True
        self.get_logger().warning(reason)
        cancel_future = goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(
            lambda response, handle=goal_handle: self._on_cancel_response(
                response,
                handle,
            )
        )

    def _on_cancel_response(self, future, goal_handle):
        if self._active_goal_handle != goal_handle:
            return
        try:
            response = future.result()
        except Exception as error:  # noqa: BLE001 - ROS future exceptions vary
            self.get_logger().error(f'Goal cancellation failed: {error}')
            self._goal_cancel_requested = False
            return

        if not response.goals_canceling:
            self.get_logger().warning(
                'Nav2 did not accept the goal cancellation request.'
            )
            self._goal_cancel_requested = False
            return

        self.get_logger().info(
            'Nav2 accepted the goal cancellation request; waiting for the '
            'terminal result.'
        )


def main(args=None):
    rclpy.init(args=args)
    node = GoalPoseBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError:
        # A launch-wide SIGINT can invalidate the shared rclpy context while
        # the executor is taking a subscription message. Treat that specific
        # shutdown race as clean, but do not hide real runtime failures.
        if rclpy.ok():
            raise
    finally:
        # Launch may have already shut down the shared context when SIGINT is
        # delivered, in which case a second shutdown raises RCLError.
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
