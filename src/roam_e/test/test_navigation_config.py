from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
NAVIGATION_CONFIG = PACKAGE_ROOT / 'config' / 'nav2.yaml'
NAVIGATION_LAUNCH = PACKAGE_ROOT / 'launch' / 'navigation.launch.py'
GOAL_BRIDGE = PACKAGE_ROOT / 'roam_e' / 'goal_pose_bridge.py'
AUTO_LOCALIZER = PACKAGE_ROOT / 'roam_e' / 'auto_localization_node.py'


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
    assert "condition=IfCondition(auto_localize)" in launch_source
    assert "'allow_motion': ParameterValue(" in launch_source
    assert "default_value='false'" in launch_source
    assert "'cmd_vel_topic', 'cmd_vel_nav'" in localizer_source
    assert "'collision_monitor/get_state'" in localizer_source
    assert 'State.PRIMARY_STATE_ACTIVE' in localizer_source
    assert 'qos_profile_sensor_data' in localizer_source
    assert "self._publish_stop()" in localizer_source
