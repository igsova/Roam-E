"""RTAB-Map RGB-D + lidar + wheel-odometry mapping or localization.

Mapping (default) is driven by teleop. With localization_only:=true the saved
database is only matched against, never extended; navigation.launch.py uses
that mode with localization:=rtabmap. Run the cameras separately
(astra_camera.launch.py use_foxglove:=false). RTAB-Map owns map->odom, so do
not run AMCL or slam_toolbox at the same time. The EKF keeps owning
odom->base_footprint. In localization mode the stored map is published on
/map for Nav2's static layer.

The Astra's registered depth is shifted onto the calibrated colour image
(depth_align_node, offset measured 16 Oct 2026), so every colour feature
gets the depth under it and the colour calibration describes both. The lidar refines each link with ICP; Force3DoF keeps the
robot on the floor plane.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution,
    PythonExpression)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


RTABMAP_PARAMETERS = {
    'frame_id': 'base_footprint',
    'odom_frame_id': 'odom',
    'map_frame_id': 'map',
    'publish_tf': True,
    'subscribe_rgbd': True,
    'subscribe_depth': False,
    'subscribe_rgb': False,
    'subscribe_scan': True,
    'approx_sync': True,
    'sync_queue_size': 10,
    'topic_queue_size': 10,
    'wait_for_transform': 0.3,
    # RTAB-Map core parameters are strings.
    'Rtabmap/DetectionRate': '1',
    'Reg/Strategy': '1',
    'Reg/Force3DoF': 'true',
    'Optimizer/GravitySigma': '0',
    'RGBD/NeighborLinkRefining': 'true',
    'RGBD/ProximityBySpace': 'true',
    'RGBD/LinearUpdate': '0.05',
    'RGBD/AngularUpdate': '0.05',
    'Icp/VoxelSize': '0.05',
    'Icp/MaxCorrespondenceDistance': '0.10',
    'Icp/PointToPlane': 'false',
    'Vis/MaxFeatures': '600',
    'Kp/MaxFeatures': '400',
    'Grid/Sensor': '2',
    'Grid/3D': 'true',
    'Grid/CellSize': '0.05',
    'Grid/RangeMax': '3.0',
    'Grid/MaxObstacleHeight': '1.5',
    'Grid/MinGroundHeight': '-0.05',
    'Grid/MaxGroundHeight': '0.03',
    'Grid/RayTracing': 'true',
}


def generate_launch_description():
    arm_motors = LaunchConfiguration('arm_motors')
    database_path = LaunchConfiguration('database_path')

    drive_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('roam_e'), 'launch', 'drive.launch.py'])),
        launch_arguments={
            'arm_motors': arm_motors,
            'use_rviz': 'false',
            'use_foxglove': LaunchConfiguration('use_foxglove'),
            'foxglove_port': LaunchConfiguration('foxglove_port'),
            'use_ekf': 'true',
            'max_linear_velocity': LaunchConfiguration('max_linear_velocity'),
            'max_angular_velocity': LaunchConfiguration('max_angular_velocity'),
            'max_wheel_ticks_per_second': '500.0',
        }.items(),
    )

    depth_align = Node(
        package='roam_e',
        executable='depth_align_node',
        name='depth_align',
        namespace='rtabmap',
        output='screen',
        parameters=[{
            'input_topic': '/camera/depth_raw/image',
            'output_topic': '/camera/depth_aligned/image',
            'shift_x_px': -7,
            'shift_y_px': -1,
        }],
    )

    rgbd_sync = Node(
        package='rtabmap_sync',
        executable='rgbd_sync',
        name='rgbd_sync',
        namespace='rtabmap',
        output='screen',
        parameters=[{
            'approx_sync': True,
            # Colour arrives at ~1.5 Hz, depth at ~18 Hz: keep enough depth
            # frames queued to find the one nearest each colour frame.
            'approx_sync_max_interval': 0.1,
            'sync_queue_size': 30,
            'topic_queue_size': 30,
        }],
        remappings=[
            ('rgb/image', '/camera/color/image_rect'),
            ('rgb/camera_info', '/camera/color/camera_info'),
            ('depth/image', '/camera/depth_aligned/image'),
        ],
    )

    localization = LaunchConfiguration('localization_only')
    mapping = PythonExpression(["'", localization, "' != 'true'"])
    rtabmap_parameters = [dict(
        RTABMAP_PARAMETERS,
        database_path=database_path,
        **{
            # RTAB-Map declares its parameters as strings; keep launch from
            # turning 'true'/'false' into booleans.
            'Mem/IncrementalMemory': ParameterValue(PythonExpression(
                ["'false' if '", localization, "' == 'true' else 'true'"]),
                value_type=str),
            'Mem/InitWMWithAllNodes': ParameterValue(localization, value_type=str),
            # 8 = GFTT/ORB (RTAB-Map default), 1 = SIFT. A database must be
            # used with the feature type it was built or reprocessed with.
            'Kp/DetectorStrategy': ParameterValue(
                LaunchConfiguration('feature_type'), value_type=str),
            'Vis/FeatureType': ParameterValue(
                LaunchConfiguration('feature_type'), value_type=str),
        },
    )]

    def rtabmap_node(condition, arguments, remappings):
        return Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            namespace='rtabmap',
            output='screen',
            condition=condition,
            parameters=rtabmap_parameters,
            remappings=[('scan', '/scan'), ('odom', '/odometry/filtered')] + remappings,
            arguments=arguments,
        )

    # Mapping keeps the map on /rtabmap/map; localization serves it as /map.
    rtabmap_continue = rtabmap_node(
        IfCondition(PythonExpression([
            mapping, " and '", LaunchConfiguration('new_map'), "' != 'true'"])),
        [], [])
    rtabmap_new = rtabmap_node(
        IfCondition(PythonExpression([
            mapping, " and '", LaunchConfiguration('new_map'), "' == 'true'"])),
        ['--delete_db_on_start'], [])
    rtabmap_localization = rtabmap_node(
        IfCondition(localization), [], [('map', '/map')])

    return LaunchDescription([
        DeclareLaunchArgument(
            'arm_motors', default_value='false',
            description='Motor-output gate; true only for supervised teleop mapping.'),
        DeclareLaunchArgument('use_foxglove', default_value='true'),
        DeclareLaunchArgument('foxglove_port', default_value='8765'),
        DeclareLaunchArgument(
            'max_linear_velocity', default_value='0.10',
            description='Mapping-mode linear speed limit in m/s.'),
        DeclareLaunchArgument(
            'max_angular_velocity', default_value='0.5',
            description='Mapping-mode yaw-rate limit in rad/s.'),
        DeclareLaunchArgument(
            'database_path',
            default_value=PathJoinSubstitution([
                EnvironmentVariable('HOME'), 'Roam-E', 'maps', 'rtabmap',
                'office_rtabmap.db']),
            description='RTAB-Map database; an existing one is extended.'),
        DeclareLaunchArgument(
            'new_map', default_value='false',
            description='true deletes database_path at start for a fresh map.'),
        DeclareLaunchArgument(
            'feature_type', default_value='8',
            description='Visual feature type: 8 GFTT/ORB (default), 1 SIFT.'),
        DeclareLaunchArgument(
            'localization_only', default_value='false',
            description='true: match against the saved map without changing it.'),
        drive_launch,
        depth_align,
        rgbd_sync,
        rtabmap_continue,
        rtabmap_new,
        rtabmap_localization,
    ])
