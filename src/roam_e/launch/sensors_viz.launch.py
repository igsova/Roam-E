from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    serial_port = LaunchConfiguration('serial_port')
    imu_addr = LaunchConfiguration('imu_addr')          # e.g. '0x28' or '40'
    use_rviz = LaunchConfiguration('use_rviz')          # 'true' or 'false'

    # Convert hex or decimal string to int at launch time
    imu_addr_int = PythonExpression(["str(int('", imu_addr, "', 0))"])

    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('imu_addr', default_value='0x28'),
        DeclareLaunchArgument('use_rviz', default_value='true'),

        Node(
            package='roam_e',
            executable='bno055_imu_node',
            name='bno055_imu_node',
            parameters=[{
                'i2c_addr': ParameterValue(imu_addr_int, value_type=int),
                'frame_id': 'imu_link',
                'publish_rate_hz': 50.0,
            }],
            output='screen'
        ),

        Node(
            package='roam_e',
            executable='rplidar_a1_node',
            name='rplidar_a1_node',
            parameters=[{
                'serial_port': serial_port,
                'frame_id': 'laser_frame',
                'scan_rate_hz': 10.0,
                'range_max': 12.0,
                'range_min': 0.15,
            }],
            output='screen'
        ),

        TimerAction(
            period=2.0,
            actions=[
                ExecuteProcess(
                    cmd=['rviz2', '-d', get_package_share_directory('roam_e') + '/rviz/sensors_viz.rviz'],
                    output='screen',
                    condition=IfCondition(use_rviz),
                )
            ]
        )
    ])

