from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import xacro


def robot_state_node(context, package_dir):
    description = xacro.process_file(
        str(package_dir / 'urdf' / 'roam_e.urdf.xacro'),
        mappings={
            'calibration_file': LaunchConfiguration('calibration_file').perform(context),
            'include_camera_optical_frames': LaunchConfiguration(
                'include_camera_optical_frames'
            ).perform(context),
        },
    ).toxml()
    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {'robot_description': ParameterValue(description, value_type=str)},
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
        )
    ]


def generate_launch_description():
    package_dir = Path(get_package_share_directory('roam_e_description'))
    default_rviz = str(package_dir / 'rviz' / 'roam_e.rviz')

    rviz_config = LaunchConfiguration('rviz_config')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'rviz_config', default_value=default_rviz, description='RViz configuration file.'
            ),
            DeclareLaunchArgument(
                'use_sim_time', default_value='false', description='Use the /clock topic.'
            ),
            DeclareLaunchArgument(
                'calibration_file',
                default_value=str(package_dir / 'config' / 'calibration.yaml'),
                description='Sensor frames and physical wheel geometry, in SI units.',
            ),
            DeclareLaunchArgument(
                'include_camera_optical_frames',
                default_value='false',
                description='Publish the CAD optical frames; keep false when the camera '
                'driver publishes them.',
            ),
            DeclareLaunchArgument(
                'publish_joints',
                default_value='true',
                description='Publish zero wheel positions for a static RViz preview.',
            ),
            DeclareLaunchArgument('rviz', default_value='true', description='Start RViz.'),
            OpaqueFunction(function=robot_state_node, args=[package_dir]),
            Node(
                package='roam_e_description',
                executable='preview_joint_state_publisher',
                name='preview_joint_state_publisher',
                output='screen',
                condition=IfCondition(LaunchConfiguration('publish_joints')),
                parameters=[{'use_sim_time': use_sim_time}],
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_config],
                condition=IfCondition(LaunchConfiguration('rviz')),
                parameters=[{'use_sim_time': use_sim_time}],
            ),
        ]
    )
