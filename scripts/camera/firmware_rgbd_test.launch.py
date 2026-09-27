"""Add an isolated cloud/optional odometry test to the RUNNING camera launch.

No camera, robot bringup, motor driver, Nav2, SLAM map, or TF writer is launched.
The additional Foxglove bridge is loopback-only and cannot command ROS.
"""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('tracking', default_value='false'),
        DeclareLaunchArgument('adapter', default_value='true'),
        DeclareLaunchArgument('bridge', default_value='true'),
        DeclareLaunchArgument('tracking_max_rate', default_value='2.0'),
        ExecuteProcess(
            cmd=['python3', str(Path(__file__).with_name('firmware_rgbd_test.py'))],
            condition=IfCondition(LaunchConfiguration('adapter')), output='screen'),
        Node(package='foxglove_bridge', executable='foxglove_bridge',
             name='rgbd_test_bridge', condition=IfCondition(LaunchConfiguration('bridge')),
             output='screen', parameters=[{
                 'address': '127.0.0.1', 'port': 8766,
                 'topic_whitelist': [
                     '^/rgbd_test/points$', '^/rgbd_test/status$', '^/rgbd_test/odom$',
                     '^/rgbd_test/odom_info_lite$', '^/rgbd_test/rgb/camera_info$',
                     '^/rgbd_test/depth/camera_info$',
                     '^/camera/color/image_raw/compressed$',
                     '^/camera/depth_preview/compressed$', '^/tf_static$', '^/tf$'],
                 'param_whitelist': ['^$'], 'service_whitelist': ['^$'],
                 'client_topic_whitelist': ['^$'], 'capabilities': ['connectionGraph'],
                 'send_buffer_limit': 2000000, 'message_backlog_size': 8, 'sysinfo': False,
             }]),
        Node(package='rtabmap_odom', executable='rgbd_odometry',
             namespace='rgbd_test', name='rgbd_odometry', output='screen',
             additional_env={'OMP_NUM_THREADS': '1', 'OPENBLAS_NUM_THREADS': '1'},
             condition=IfCondition(LaunchConfiguration('tracking')),
             parameters=[{
                 # Existing camera_link -> optical TF is consumed, never written.
                 'frame_id': 'camera_link', 'odom_frame_id': 'rgbd_test_odom',
                 'publish_tf': False, 'publish_null_when_lost': True,
                 'approx_sync': False, 'topic_queue_size': 3, 'sync_queue_size': 3,
                 'qos': 2, 'qos_camera_info': 2,
                 'wait_for_transform': 0.1,
                 'always_process_most_recent_frame': True,
                 'max_update_rate': ParameterValue(
                     LaunchConfiguration('tracking_max_rate'), value_type=float),
                 'Reg/Strategy': ParameterValue('0', value_type=str),
                 'Odom/Strategy': ParameterValue('0', value_type=str),
                 'Odom/ResetCountdown': ParameterValue('0', value_type=str),
                 'Rtabmap/ImagesAlreadyRectified': ParameterValue('true', value_type=str),
             }],
             remappings=[('rgb/image', 'rgb/image_rect'), ('depth/image', 'depth/image_rect'),
                         ('rgb/camera_info', 'rgb/camera_info'), ('odom', 'odom'),
                         ('diagnostics', 'diagnostics')]),
    ])
