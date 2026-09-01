#!/usr/bin/env python3
"""Publish stationary wheel joints for RViz until encoder feedback exists."""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState


class PreviewJointStatePublisher(Node):
    def __init__(self):
        super().__init__('preview_joint_state_publisher')
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.1, self.publish_joint_state)
        self.get_logger().info(
            'Publishing stationary wheel joints; disable with publish_joints:=false '
            'when encoder feedback is active.'
        )

    def publish_joint_state(self):
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = ['left_wheel_joint', 'right_wheel_joint']
        message.position = [0.0, 0.0]
        self.publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = PreviewJointStatePublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        # A launch-wide shutdown can invalidate the context while spin_once()
        # is constructing its wait set. Preserve genuine runtime failures.
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
