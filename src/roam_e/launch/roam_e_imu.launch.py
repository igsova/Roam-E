#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='base_to_imu_static_transform',
                arguments=[
                    '--x', '-0.030',
                    '--y', '0.000',
                    '--z', '0.105',
                    '--roll', '3.141592653589793',
                    '--pitch', '0.0',
                    '--yaw', '0.0',
                    '--frame-id', 'base_link',
                    '--child-frame-id', 'imu_link',
                ],
                output='screen',
            ),
            Node(
                package='roam_e',
                executable='bno055_imu_node',
                name='bno055_imu_node',
                parameters=[
                    {
                        'i2c_bus': 3,
                        'i2c_addr': 0x29,
                        'frame_id': 'imu_link',
                        'publish_rate_hz': 20.0,
                    }
                ],
                output='screen',
            ),
        ]
    )
