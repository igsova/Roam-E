from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    arm_motors = LaunchConfiguration('arm_motors')
    use_foxglove = LaunchConfiguration('use_foxglove')
    foxglove_port = LaunchConfiguration('foxglove_port')
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_file = LaunchConfiguration('map')
    amcl_params_file = LaunchConfiguration('amcl_params_file')
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

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': map_file,
            'use_sim_time': use_sim_time,
        }],
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            amcl_params_file,
            {'use_sim_time': use_sim_time},
        ],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['map_server', 'amcl'],
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'arm_motors',
            default_value='false',
            description=(
                'Explicit motor-output gate. Set true only for supervised '
                'localization motion in a clear area.'
            ),
        ),
        DeclareLaunchArgument('use_foxglove', default_value='true'),
        DeclareLaunchArgument('foxglove_port', default_value='8765'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'map',
            default_value=PathJoinSubstitution([
                EnvironmentVariable('HOME'),
                'Roam-E',
                'maps',
                'office_02.yaml',
            ]),
            description='Absolute path to the Nav2 occupancy-map YAML file.',
        ),
        DeclareLaunchArgument(
            'amcl_params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('roam_e'),
                'config',
                'amcl.yaml',
            ]),
        ),
        DeclareLaunchArgument(
            'max_linear_velocity',
            default_value='0.10',
            description='Localization-mode linear speed limit in m/s.',
        ),
        DeclareLaunchArgument(
            'max_angular_velocity',
            default_value='0.4',
            description='Localization-mode yaw-rate limit in rad/s.',
        ),
        drive_launch,
        map_server,
        amcl,
        lifecycle_manager,
    ])
