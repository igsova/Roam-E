from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
NAVIGATION_CONFIG = PACKAGE_ROOT / 'config' / 'nav2.yaml'
NAVIGATION_LAUNCH = PACKAGE_ROOT / 'launch' / 'navigation.launch.py'
GOAL_BRIDGE = PACKAGE_ROOT / 'roam_e' / 'goal_pose_bridge.py'
AUTO_LOCALIZER = PACKAGE_ROOT / 'roam_e' / 'auto_localization_node.py'
BOUNDED_NAVIGATION_TREE = (
    PACKAGE_ROOT / 'behavior_trees' / 'navigate_to_pose_bounded.xml'
)
SETUP_FILE = PACKAGE_ROOT / 'setup.py'


def load_navigation_parameters():
    with NAVIGATION_CONFIG.open(encoding='utf-8') as stream:
        return yaml.safe_load(stream)


def test_costmaps_share_the_cad_footprint():
    config = load_navigation_parameters()
    local = config['local_costmap']['local_costmap']['ros__parameters']
    global_ = config['global_costmap']['global_costmap']['ros__parameters']

    assert local['footprint'] == global_['footprint']
    assert local['footprint_padding'] == 0.01
    assert global_['footprint_padding'] == 0.01
    assert 'robot_radius' not in local
    assert 'robot_radius' not in global_


def test_first_navigation_speed_is_conservative():
    config = load_navigation_parameters()
    navigator = config['bt_navigator']['ros__parameters']
    controller = config['controller_server']['ros__parameters']
    smoother = config['velocity_smoother']['ros__parameters']

    assert navigator['default_server_timeout'] >= 1000
    assert controller['FollowPath']['desired_linear_vel'] <= 0.08
    assert controller['FollowPath']['min_approach_linear_velocity'] >= 0.06
    assert controller['progress_checker']['plugin'] == (
        'nav2_controller::PoseProgressChecker'
    )
    assert controller['progress_checker']['required_movement_angle'] <= 0.15
    assert smoother['max_velocity'][0] <= 0.08
    assert smoother['max_velocity'][2] <= 0.35


def test_collision_monitor_is_the_only_final_cmd_vel_output():
    config = load_navigation_parameters()
    monitor = config['collision_monitor']['ros__parameters']
    stop_points = yaml.safe_load(monitor['StopZone']['points'])
    slowdown_points = yaml.safe_load(monitor['SlowdownZone']['points'])

    assert monitor['cmd_vel_in_topic'] == 'cmd_vel_smoothed'
    assert monitor['cmd_vel_out_topic'] == 'cmd_vel'
    assert monitor['source_timeout'] <= 0.5
    assert monitor['polygons'] == ['StopZone', 'SlowdownZone']
    assert monitor['StopZone']['action_type'] == 'stop'
    assert monitor['SlowdownZone']['action_type'] == 'slowdown'
    assert monitor['SlowdownZone']['slowdown_ratio'] >= 0.75
    assert max(point[0] for point in stop_points) >= 0.30
    assert max(point[0] for point in slowdown_points) >= 0.50


def test_navigation_startup_waits_for_localization_transform():
    launch_source = NAVIGATION_LAUNCH.read_text(encoding='utf-8')
    bridge_source = GOAL_BRIDGE.read_text(encoding='utf-8')

    assert "'autostart': False" in launch_source
    assert "'start_navigation_when_localized': ParameterValue(" in launch_source
    assert 'can_transform(' in bridge_source
    assert 'ConvergenceTracker(' in bridge_source
    assert 'if not localization_ready:' in bridge_source
    assert 'ManageLifecycleNodes.Request.STARTUP' in bridge_source
    assert "self._base_frame = self.get_parameter('base_frame').value" in (
        bridge_source
    )
    assert 'except RuntimeError:' in bridge_source
    assert 'if rclpy.ok():' in bridge_source


def test_safety_chain_starts_before_automatic_localization_motion():
    launch_source = NAVIGATION_LAUNCH.read_text(encoding='utf-8')
    localizer_source = AUTO_LOCALIZER.read_text(encoding='utf-8')

    assert "name='lifecycle_manager_safety'" in launch_source
    assert "'node_names': [\n                'velocity_smoother'," in (
        launch_source
    )
    # The AMCL search maneuver runs only when requested and AMCL localizes.
    assert '"\'", auto_localize, "\' == \'true\' and ", use_amcl' in launch_source
    assert "'allow_motion': ParameterValue(" in launch_source
    assert "default_value='false'" in launch_source
    assert "'cmd_vel_topic', 'cmd_vel_nav'" in localizer_source
    assert "'collision_monitor/get_state'" in localizer_source
    assert 'State.PRIMARY_STATE_ACTIVE' in localizer_source
    assert 'qos_profile_sensor_data' in localizer_source
    assert "self._publish_stop()" in localizer_source


def test_navigation_recovery_is_bounded_and_does_not_move_blindly():
    tree = ET.parse(BOUNDED_NAVIGATION_TREE).getroot()
    recovery = tree.find(
        ".//BehaviorTree/RecoveryNode[@name='NavigateRecovery']"
    )

    assert recovery is not None
    assert int(recovery.attrib['number_of_retries']) == 1
    assert tree.findall('.//BackUp') == []
    assert tree.findall('.//Spin') == []
    assert tree.findall('.//ClearEntireCostmap')


def test_goal_timeout_and_bounded_tree_are_installed_and_wired():
    launch_source = NAVIGATION_LAUNCH.read_text(encoding='utf-8')
    bridge_source = GOAL_BRIDGE.read_text(encoding='utf-8')
    setup_source = SETUP_FILE.read_text(encoding='utf-8')

    assert "'navigation_bt_xml'" in launch_source
    assert "'default_nav_to_pose_bt_xml': ParameterValue(" in launch_source
    assert "'navigation_goal_timeout'" in launch_source
    assert "'goal_timeout': ParameterValue(" in launch_source
    assert "default_value='180.0'" in launch_source
    assert "self.declare_parameter('goal_timeout', 180.0)" in bridge_source
    assert 'def _enforce_goal_timeout(self):' in bridge_source
    assert 'time.monotonic()' in bridge_source
    assert 'goal_handle.cancel_goal_async()' in bridge_source
    assert "'share/' + package_name + '/behavior_trees'" in setup_source
    assert "'behavior_trees/navigate_to_pose_bounded.xml'" in setup_source


def test_r19_camera_forward_geometry():
    # 14 Oct 2026: R19 camera-forward CAD hull (+X camera, caster rear).
    config = load_navigation_parameters()
    local = config['local_costmap']['local_costmap']['ros__parameters']
    points = yaml.safe_load(local['footprint'])
    assert min(p[0] for p in points) == -0.143811
    assert max(p[0] for p in points) == 0.06965
    assert min(p[1] for p in points) == -0.12
    assert max(p[1] for p in points) == 0.11906
    monitor = config['collision_monitor']['ros__parameters']
    stop = yaml.safe_load(monitor['StopZone']['points'])
    assert min(p[0] for p in stop) < min(p[0] for p in points) - 0.15


def test_depth_camera_voxel_layer_marks_low_obstacles():
    # 15 Oct 2026: Astra depth points feed a 3D voxel layer in the local
    # costmap only; heights between the floor and the 24 cm lidar plane.
    config = load_navigation_parameters()
    local = config['local_costmap']['local_costmap']['ros__parameters']
    assert local['plugins'] == [
        'obstacle_layer', 'camera_voxel_layer', 'inflation_layer']
    layer = local['camera_voxel_layer']
    assert layer['plugin'] == 'nav2_costmap_2d::VoxelLayer'
    camera = layer['camera']
    assert camera['topic'] == '/camera/obstacle_points'
    assert camera['data_type'] == 'PointCloud2'
    assert 0.0 < camera['min_obstacle_height'] < 0.24
    assert camera['max_obstacle_height'] <= layer['max_obstacle_height']
    assert layer['z_voxels'] * layer['z_resolution'] >= layer['max_obstacle_height']
    assert camera['obstacle_min_range'] >= 0.45
    global_ = config['global_costmap']['global_costmap']['ros__parameters']
    assert 'camera_voxel_layer' not in global_['plugins']

    launch_source = NAVIGATION_LAUNCH.read_text(encoding='utf-8')
    assert "executable='nav2_costmap_2d_cloud'" in launch_source
    assert "namespace='local_costmap'" in launch_source


def test_rtabmap_localization_replaces_amcl_when_selected():
    # 15 Oct 2026: localization:=rtabmap matches the saved RTAB-Map
    # database instead of running AMCL; exactly one map->odom owner.
    launch_source = NAVIGATION_LAUNCH.read_text(encoding='utf-8')
    rtabmap_source = (
        PACKAGE_ROOT / 'launch' / 'rtabmap_mapping.launch.py'
    ).read_text(encoding='utf-8')
    assert "default_value='amcl'" in launch_source
    assert 'condition=IfCondition(use_amcl)' in launch_source
    assert 'condition=IfCondition(use_rtabmap)' in launch_source
    assert "'localization_only': 'true'" in launch_source
    assert "'/rtabmap/localization_pose' if '" in launch_source
    # Jumps are checked on RTAB-Map's map->odom correction, not robot motion.
    assert "'jump_check_frame': ParameterValue(PythonExpression([" in launch_source
    assert "max_position_jump" not in launch_source
    assert "IfCondition(localization), [], [('map', '/map')]" in rtabmap_source
    assert "'Mem/IncrementalMemory': ParameterValue(PythonExpression(" in rtabmap_source
    assert "'publish_tf': True" in rtabmap_source
