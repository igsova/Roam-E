from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    serial_port = LaunchConfiguration('serial_port')
    imu_addr = LaunchConfiguration('imu_addr')
    use_rviz = LaunchConfiguration('use_rviz')
    publish_joints = LaunchConfiguration('publish_joints')
    use_encoders = LaunchConfiguration('use_encoders')
    motor_serial_port = LaunchConfiguration('motor_serial_port')
    encoder_ticks_per_revolution = LaunchConfiguration(
        'encoder_ticks_per_revolution'
    )
    wheel_radius = LaunchConfiguration('wheel_radius')
    wheel_separation = LaunchConfiguration('wheel_separation')
    publish_odom_tf = LaunchConfiguration('publish_odom_tf')
    enable_motor_control = LaunchConfiguration('enable_motor_control')
    use_stamped_cmd_vel = LaunchConfiguration('use_stamped_cmd_vel')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    command_timeout = LaunchConfiguration('command_timeout')
    command_rate = LaunchConfiguration('command_rate')
    max_linear_velocity = LaunchConfiguration('max_linear_velocity')
    max_angular_velocity = LaunchConfiguration('max_angular_velocity')
    max_wheel_ticks_per_second = LaunchConfiguration(
        'max_wheel_ticks_per_second'
    )
    velocity_pid_kp = LaunchConfiguration('velocity_pid_kp')
    velocity_pid_ki = LaunchConfiguration('velocity_pid_ki')
    velocity_feedforward_kv = LaunchConfiguration(
        'velocity_feedforward_kv'
    )
    velocity_static_pwm = LaunchConfiguration('velocity_static_pwm')
    left_forward_static_pwm = LaunchConfiguration(
        'left_forward_static_pwm'
    )
    left_reverse_static_pwm = LaunchConfiguration(
        'left_reverse_static_pwm'
    )
    right_forward_static_pwm = LaunchConfiguration(
        'right_forward_static_pwm'
    )
    right_reverse_static_pwm = LaunchConfiguration(
        'right_reverse_static_pwm'
    )

    # Convert a hexadecimal or decimal string to an integer at launch time.
    imu_addr_int = PythonExpression(["str(int('", imu_addr, "', 0))"])

    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port',
            default_value=(
                '/dev/serial/by-id/'
                'usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_'
                'Controller_0001-if00-port0'
            ),
        ),
        DeclareLaunchArgument('imu_addr', default_value='0x29'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument(
            'use_encoders',
            default_value='true',
            description=(
                'Read wheel encoders and publish joint states plus odometry.'
            ),
        ),
        DeclareLaunchArgument(
            'motor_serial_port',
            default_value=(
                '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0'
            ),
        ),
        DeclareLaunchArgument(
            'encoder_ticks_per_revolution', default_value='748'
        ),
        DeclareLaunchArgument('wheel_radius', default_value='0.033975'),
        DeclareLaunchArgument('wheel_separation', default_value='0.175150'),
        DeclareLaunchArgument('publish_odom_tf', default_value='true'),
        DeclareLaunchArgument(
            'enable_motor_control',
            default_value='false',
            description=(
                'Permit cmd_vel to drive the motors. Defaults to false so '
                'the launch remains encoder-only and cannot move the robot.'
            ),
        ),
        DeclareLaunchArgument(
            'use_stamped_cmd_vel',
            default_value='true',
            description='Subscribe to TwistStamped instead of Twist.',
        ),
        DeclareLaunchArgument('cmd_vel_topic', default_value='cmd_vel'),
        DeclareLaunchArgument('command_timeout', default_value='0.25'),
        DeclareLaunchArgument('command_rate', default_value='20.0'),
        DeclareLaunchArgument(
            'max_linear_velocity', default_value='0.15'
        ),
        DeclareLaunchArgument(
            'max_angular_velocity', default_value='1.5'
        ),
        DeclareLaunchArgument(
            'max_wheel_ticks_per_second', default_value='1200.0'
        ),
        DeclareLaunchArgument('velocity_pid_kp', default_value='0.080'),
        DeclareLaunchArgument('velocity_pid_ki', default_value='0.200'),
        DeclareLaunchArgument(
            'velocity_feedforward_kv', default_value='0.000'
        ),
        DeclareLaunchArgument('velocity_static_pwm', default_value='75'),
        DeclareLaunchArgument(
            'left_forward_static_pwm', default_value='75'
        ),
        DeclareLaunchArgument(
            'left_reverse_static_pwm', default_value='60'
        ),
        DeclareLaunchArgument(
            'right_forward_static_pwm', default_value='75'
        ),
        DeclareLaunchArgument(
            'right_reverse_static_pwm', default_value='60'
        ),
        DeclareLaunchArgument(
            'publish_joints',
            default_value=PythonExpression([
                "'false' if '",
                use_encoders,
                "'.lower() == 'true' else 'true'",
            ]),
            description=(
                'Publish static preview joints; defaults off when encoders run.'
            ),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('roam_e_description'),
                    'launch',
                    'display.launch.py',
                ])
            ),
            launch_arguments={
                'rviz': use_rviz,
                'publish_joints': publish_joints,
            }.items(),
        ),

        Node(
            package='roam_e',
            executable='encoder_odometry_node',
            name='encoder_odometry_node',
            condition=IfCondition(use_encoders),
            parameters=[{
                'serial_port': motor_serial_port,
                'ticks_per_revolution': ParameterValue(
                    encoder_ticks_per_revolution, value_type=int
                ),
                'wheel_radius': ParameterValue(
                    wheel_radius, value_type=float
                ),
                'wheel_separation': ParameterValue(
                    wheel_separation, value_type=float
                ),
                'publish_odom_tf': ParameterValue(
                    publish_odom_tf, value_type=bool
                ),
                'enable_motor_control': ParameterValue(
                    enable_motor_control, value_type=bool
                ),
                'use_stamped_cmd_vel': ParameterValue(
                    use_stamped_cmd_vel, value_type=bool
                ),
                'cmd_vel_topic': cmd_vel_topic,
                'command_timeout': ParameterValue(
                    command_timeout, value_type=float
                ),
                'command_rate': ParameterValue(
                    command_rate, value_type=float
                ),
                'max_linear_velocity': ParameterValue(
                    max_linear_velocity, value_type=float
                ),
                'max_angular_velocity': ParameterValue(
                    max_angular_velocity, value_type=float
                ),
                'max_wheel_ticks_per_second': ParameterValue(
                    max_wheel_ticks_per_second, value_type=float
                ),
                'velocity_pid_kp': ParameterValue(
                    velocity_pid_kp, value_type=float
                ),
                'velocity_pid_ki': ParameterValue(
                    velocity_pid_ki, value_type=float
                ),
                'velocity_feedforward_kv': ParameterValue(
                    velocity_feedforward_kv, value_type=float
                ),
                'velocity_static_pwm': ParameterValue(
                    velocity_static_pwm, value_type=int
                ),
                'left_forward_static_pwm': ParameterValue(
                    left_forward_static_pwm, value_type=int
                ),
                'left_reverse_static_pwm': ParameterValue(
                    left_reverse_static_pwm, value_type=int
                ),
                'right_forward_static_pwm': ParameterValue(
                    right_forward_static_pwm, value_type=int
                ),
                'right_reverse_static_pwm': ParameterValue(
                    right_reverse_static_pwm, value_type=int
                ),
            }],
            output='screen',
        ),

        Node(
            package='roam_e',
            executable='bno055_imu_node',
            name='bno055_imu_node',
            parameters=[{
                'i2c_addr': ParameterValue(imu_addr_int, value_type=int),
                'frame_id': 'imu_link',
                'publish_rate_hz': 20.0,
            }],
            output='screen',
        ),

        Node(
            package='roam_e',
            executable='rplidar_a1_node',
            name='rplidar_a1_node',
            parameters=[{
                'serial_port': serial_port,
                'frame_id': 'laser_frame',
                'range_max': 12.0,
                'range_min': 0.15,
            }],
            output='screen',
        ),
    ])
