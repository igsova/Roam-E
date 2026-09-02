from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    arm_motors = LaunchConfiguration('arm_motors')
    use_foxglove = LaunchConfiguration('use_foxglove')
    foxglove_port = LaunchConfiguration('foxglove_port')
    slam_params_file = LaunchConfiguration('slam_params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    max_linear_velocity = LaunchConfiguration('max_linear_velocity')
    max_angular_velocity = LaunchConfiguration('max_angular_velocity')

    drive_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('roam_e'),
                'launch',
                'drive.launch.py',
            ])
        ),
        launch_arguments={
            'arm_motors': arm_motors,
            'use_rviz': 'false',
            'use_foxglove': use_foxglove,
            'foxglove_port': foxglove_port,
            'use_ekf': 'true',
            'max_linear_velocity': max_linear_velocity,
            'max_angular_velocity': max_angular_velocity,
            'max_wheel_ticks_per_second': '500.0',
        }.items(),
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('slam_toolbox'),
                'launch',
                'online_async_launch.py',
            ])
        ),
        launch_arguments={
            'autostart': 'true',
            'use_lifecycle_manager': 'false',
            'use_sim_time': use_sim_time,
            'slam_params_file': slam_params_file,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'arm_motors',
            default_value='false',
            description=(
                'Explicit motor-output gate. Leave false for the static '
                'SLAM health check and set true only for supervised mapping.'
            ),
        ),
        DeclareLaunchArgument('use_foxglove', default_value='true'),
        DeclareLaunchArgument('foxglove_port', default_value='8765'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('roam_e'),
                'config',
                'slam.yaml',
            ]),
        ),
        DeclareLaunchArgument(
            'max_linear_velocity',
            default_value='0.10',
            description='Mapping-mode linear speed limit in m/s.',
        ),
        DeclareLaunchArgument(
            'max_angular_velocity',
            default_value='0.5',
            description='Mapping-mode yaw-rate limit in rad/s.',
        ),
        drive_launch,
        slam_launch,
    ])
