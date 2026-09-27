"""Temporary camera-only native-depth diagnostic; no fitted calibration applied.

Do not run alongside astra_camera.launch.py. Native depth is deliberately NOT
RGB-registered. Stop this launch and restore astra_camera.launch.py after capture.
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sdk = '/home/robot/OpenNI-Linux-Arm64-2.3.0.65/Samples/Bin'
    return LaunchDescription([
        SetEnvironmentVariable('LD_LIBRARY_PATH', [sdk, ':', EnvironmentVariable('LD_LIBRARY_PATH', default_value='')]),
        SetEnvironmentVariable('OPENNI2_DRIVERS_PATH', sdk+'/OpenNI2/Drivers'),
        ComposableNodeContainer(
            name='native_depth_container', namespace='camera',
            package='rclcpp_components', executable='component_container',
            composable_node_descriptions=[ComposableNode(
                package='openni2_camera', plugin='openni2_wrapper::OpenNI2Driver',
                name='driver', namespace='camera', parameters=[{
                    'depth_registration': False, 'use_device_time': True,
                    'rgb_frame_id': 'camera_rgb_optical_frame',
                    'depth_frame_id': 'camera_depth_optical_frame',
                    'ir_frame_id': 'camera_ir_optical_frame',
                }])], output='screen'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('roam_e'), 'launch', 'astra_camera.launch.py'])),
            launch_arguments={'start_depth': 'false'}.items()),
        Node(package='roam_e', executable='depth_preview_node', name='depth_preview',
             output='screen', parameters=[{
                 'input_topic': '/camera/depth_raw/image',
                 'output_topic': '/camera/depth_preview/compressed',
                 'min_depth_m': 0.2, 'max_depth_m': 5.0,
                 'publish_rate': 5.0, 'downsample': 2, 'jpeg_quality': 70,
             }]),
    ])
