from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    arm_motors = LaunchConfiguration('arm_motors')
    use_rviz = LaunchConfiguration('use_rviz')
    use_foxglove = LaunchConfiguration('use_foxglove')
    foxglove_port = LaunchConfiguration('foxglove_port')
    use_ekf = LaunchConfiguration('use_ekf')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    max_linear_velocity = LaunchConfiguration('max_linear_velocity')
    max_angular_velocity = LaunchConfiguration('max_angular_velocity')
    max_wheel_ticks_per_second = LaunchConfiguration(
        'max_wheel_ticks_per_second'
    )

    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('roam_e'),
                'launch',
                'sensors_viz.launch.py',
            ])
        ),
        launch_arguments={
            'use_rviz': use_rviz,
            'use_ekf': use_ekf,
            'enable_motor_control': arm_motors,
            # Foxglove Teleop supports geometry_msgs/msg/Twist.
            'use_stamped_cmd_vel': 'false',
            'cmd_vel_topic': cmd_vel_topic,
            'command_timeout': '0.25',
            'command_rate': '20.0',
            'max_linear_velocity': max_linear_velocity,
            'max_angular_velocity': max_angular_velocity,
            'max_wheel_ticks_per_second': (
                max_wheel_ticks_per_second
            ),
        }.items(),
    )

    foxglove_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('foxglove_bridge'),
                'launch',
                'foxglove_bridge_launch.xml',
            ])
        ),
        condition=IfCondition(use_foxglove),
        launch_arguments={
            'port': foxglove_port,
            'address': '0.0.0.0',
            'capabilities': (
                '[clientPublish,parameters,parametersSubscribe,services,'
                'connectionGraph,assets]'
            ),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'arm_motors',
            default_value='false',
            description=(
                'Explicit motor-output gate. Set true only when the area '
                'around the robot is clear.'
            ),
        ),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument('use_foxglove', default_value='true'),
        DeclareLaunchArgument(
            'use_ekf',
            default_value='true',
            description=(
                'Fuse wheel odometry and IMU data into odometry/filtered.'
            ),
        ),
        DeclareLaunchArgument('foxglove_port', default_value='8765'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='cmd_vel'),
        DeclareLaunchArgument(
            'max_linear_velocity',
            default_value='0.20',
            description=(
                'Robot-side linear speed clamp in m/s. The Foxglove '
                'Normal and Fast presets request 0.06 and 0.18 m/s.'
            ),
        ),
        DeclareLaunchArgument(
            'max_angular_velocity',
            default_value='0.6',
            description='Robot-side yaw-rate clamp in rad/s.',
        ),
        DeclareLaunchArgument(
            'max_wheel_ticks_per_second',
            default_value='700.0',
            description='Independent firmware target clamp per wheel.',
        ),
        sensors_launch,
        foxglove_launch,
    ])
