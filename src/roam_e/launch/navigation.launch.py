from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    arm_motors = LaunchConfiguration('arm_motors')
    use_foxglove = LaunchConfiguration('use_foxglove')
    foxglove_port = LaunchConfiguration('foxglove_port')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    auto_localize = LaunchConfiguration('auto_localize')
    map_file = LaunchConfiguration('map')
    amcl_params_file = LaunchConfiguration('amcl_params_file')
    navigation_params_file = LaunchConfiguration('navigation_params_file')
    navigation_bt_xml = LaunchConfiguration('navigation_bt_xml')
    navigation_goal_timeout = LaunchConfiguration(
        'navigation_goal_timeout'
    )
    localization = LaunchConfiguration('localization')
    use_amcl = PythonExpression(["'", localization, "' == 'amcl'"])
    use_rtabmap = PythonExpression(["'", localization, "' == 'rtabmap'"])

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('roam_e'),
                'launch',
                'localization.launch.py',
            ])
        ),
        condition=IfCondition(use_amcl),
        launch_arguments={
            'arm_motors': arm_motors,
            'use_foxglove': use_foxglove,
            'foxglove_port': foxglove_port,
            'use_sim_time': use_sim_time,
            'map': map_file,
            'amcl_params_file': amcl_params_file,
            # Independent final clamps in the guarded motor node.
            'max_linear_velocity': '0.08',
            'max_angular_velocity': '0.35',
        }.items(),
    )

    # RTAB-Map localization on its saved RGB-D/lidar database instead of
    # AMCL on the 2D map. It serves the stored map on /map and map->odom.
    rtabmap_localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('roam_e'),
                'launch',
                'rtabmap_mapping.launch.py',
            ])
        ),
        condition=IfCondition(use_rtabmap),
        launch_arguments={
            # Distinct name: included arguments share this launch's scope.
            'localization_only': 'true',
            'arm_motors': arm_motors,
            'use_foxglove': use_foxglove,
            'foxglove_port': foxglove_port,
            'database_path': LaunchConfiguration('rtabmap_database'),
            'feature_type': LaunchConfiguration('rtabmap_feature_type'),
            'max_linear_velocity': '0.08',
            'max_angular_velocity': '0.35',
        }.items(),
    )

    common_parameters = [
        navigation_params_file,
        {
            'use_sim_time': ParameterValue(
                use_sim_time,
                value_type=bool,
            ),
        },
    ]
    tf_remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=common_parameters,
        remappings=tf_remappings + [('cmd_vel', 'cmd_vel_nav')],
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=common_parameters,
        remappings=tf_remappings,
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=common_parameters,
        remappings=tf_remappings + [('cmd_vel', 'cmd_vel_nav')],
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=common_parameters + [{
            'default_nav_to_pose_bt_xml': ParameterValue(
                navigation_bt_xml,
                value_type=str,
            ),
        }],
        remappings=tf_remappings,
    )

    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=common_parameters,
        remappings=tf_remappings + [('cmd_vel', 'cmd_vel_nav')],
    )

    collision_monitor = Node(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        output='screen',
        parameters=common_parameters,
        remappings=tf_remappings,
    )

    safety_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_safety',
        output='screen',
        parameters=[{
            'use_sim_time': ParameterValue(
                use_sim_time,
                value_type=bool,
            ),
            # These map-independent command guards must be active before an
            # automatic localization maneuver can publish any velocity.
            'autostart': True,
            'bond_timeout': 15.0,
            'node_names': [
                'velocity_smoother',
                'collision_monitor',
            ],
        }],
    )

    auto_localization_node = Node(
        package='roam_e',
        executable='auto_localization_node',
        name='auto_localization_node',
        output='screen',
        condition=IfCondition(PythonExpression([
            "'", auto_localize, "' == 'true' and ", use_amcl])),
        parameters=[{
            'use_sim_time': ParameterValue(
                use_sim_time,
                value_type=bool,
            ),
            'allow_motion': ParameterValue(
                arm_motors,
                value_type=bool,
            ),
        }],
    )

    goal_pose_bridge = Node(
        package='roam_e',
        executable='goal_pose_bridge',
        name='goal_pose_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': ParameterValue(
                use_sim_time,
                value_type=bool,
            ),
            'goal_topic': 'goal_pose',
            'amcl_pose_topic': PythonExpression([
                "'/rtabmap/localization_pose' if '", localization,
                "' == 'rtabmap' else 'amcl_pose'"]),
            # RTAB-Map's pose arrives irregularly (gaps of several seconds
            # under load), so check jumps in its map->odom correction, not in
            # the robot pose, which also moves with the robot.
            'jump_check_frame': ParameterValue(PythonExpression([
                "'odom' if '", localization, "' == 'rtabmap' else ''"]),
                value_type=str),
            'action_name': 'navigate_to_pose',
            'global_frame': 'map',
            'base_frame': 'base_footprint',
            'start_navigation_when_localized': ParameterValue(
                autostart,
                value_type=bool,
            ),
            'lifecycle_manager_name': 'lifecycle_manager_navigation',
            'goal_timeout': ParameterValue(
                navigation_goal_timeout,
                value_type=float,
            ),
        }],
    )

    # Converts the local costmap's camera voxel grid (nav2_msgs/VoxelGrid,
    # which Foxglove cannot draw) into /local_costmap/voxel_marked_cloud.
    local_voxel_cloud = Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d_cloud',
        name='voxel_cloud',
        namespace='local_costmap',
        output='screen',
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': ParameterValue(
                use_sim_time,
                value_type=bool,
            ),
            # GoalPoseBridge starts this manager only after AMCL covariance
            # is low and stable. The safety nodes use their separate manager.
            'autostart': False,
            # The 1 GB Pi can pause several seconds while all Nav2 plugins and
            # Foxglove discovery are busy. Avoid treating that as a crash.
            'bond_timeout': 15.0,
            'node_names': [
                'controller_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
            ],
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'arm_motors',
            default_value='false',
            description=(
                'Explicit motor-output gate. Keep false while validating '
                'localization, plans, costmaps, and safety zones.'
            ),
        ),
        DeclareLaunchArgument('use_foxglove', default_value='true'),
        DeclareLaunchArgument('foxglove_port', default_value='8765'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument(
            'auto_localize',
            default_value='false',
            description=(
                'Run one automatic global-localization search. Physical '
                'motion occurs only when arm_motors is also true.'
            ),
        ),
        DeclareLaunchArgument(
            'localization',
            default_value='amcl',
            description=(
                'amcl: AMCL on the 2D map (default). rtabmap: RTAB-Map '
                'localization on rtabmap_database (auto_localize ignored).'
            ),
        ),
        DeclareLaunchArgument(
            'rtabmap_database',
            default_value=PathJoinSubstitution([
                EnvironmentVariable('HOME'), 'Roam-E', 'maps', 'rtabmap',
                'office_rtabmap.db']),
        ),
        DeclareLaunchArgument(
            'rtabmap_feature_type', default_value='8',
            description='RTAB-Map visual features of rtabmap_database: 8 GFTT/ORB, 1 SIFT.'),
        DeclareLaunchArgument(
            'map',
            default_value=PathJoinSubstitution([
                EnvironmentVariable('HOME'),
                'Roam-E',
                'maps',
                'office_02.yaml',
            ]),
            description='Absolute path to the occupancy-map YAML file.',
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
            'navigation_params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('roam_e'),
                'config',
                'nav2.yaml',
            ]),
        ),
        DeclareLaunchArgument(
            'navigation_bt_xml',
            default_value=PathJoinSubstitution([
                FindPackageShare('roam_e'),
                'behavior_trees',
                'navigate_to_pose_bounded.xml',
            ]),
            description=(
                'Behavior tree used for NavigateToPose goals. The default '
                'performs one costmap recovery cycle and never commands an '
                'automatic spin or reverse maneuver.'
            ),
        ),
        DeclareLaunchArgument(
            'navigation_goal_timeout',
            default_value='180.0',
            description=(
                'Seconds before requesting cancellation of an accepted '
                'Foxglove goal; not a hard motor-stop guarantee.'
            ),
        ),
        localization_launch,
        rtabmap_localization_launch,
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        velocity_smoother,
        collision_monitor,
        safety_lifecycle_manager,
        auto_localization_node,
        goal_pose_bridge,
        lifecycle_manager,
        local_voxel_cloud,
    ])
