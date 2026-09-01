#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            # The URDF owns base_link -> imu_link so standalone and combined
            # launches cannot publish conflicting static transforms.
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare('roam_e_description'),
                        'launch',
                        'display.launch.py',
                    ])
                ),
                launch_arguments={
                    'rviz': 'false',
                    'publish_joints': 'false',
                }.items(),
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
