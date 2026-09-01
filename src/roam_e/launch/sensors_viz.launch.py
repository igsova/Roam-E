from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
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
            'publish_joints',
            default_value='true',
            description='Publish zero wheel positions until encoders provide them.',
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
