from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
NAVIGATION_CONFIG = PACKAGE_ROOT / 'config' / 'nav2.yaml'
NAVIGATION_LAUNCH = PACKAGE_ROOT / 'launch' / 'navigation.launch.py'
GOAL_BRIDGE = PACKAGE_ROOT / 'roam_e' / 'goal_pose_bridge.py'


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
    controller = config['controller_server']['ros__parameters']
    smoother = config['velocity_smoother']['ros__parameters']

    assert controller['FollowPath']['desired_linear_vel'] <= 0.08
    assert smoother['max_velocity'][0] <= 0.08
    assert smoother['max_velocity'][2] <= 0.35


def test_collision_monitor_is_the_only_final_cmd_vel_output():
    config = load_navigation_parameters()
    monitor = config['collision_monitor']['ros__parameters']

    assert monitor['cmd_vel_in_topic'] == 'cmd_vel_smoothed'
    assert monitor['cmd_vel_out_topic'] == 'cmd_vel'
    assert monitor['source_timeout'] <= 0.5
    assert monitor['polygons'] == ['StopZone', 'SlowdownZone']
    assert monitor['StopZone']['action_type'] == 'stop'
    assert monitor['SlowdownZone']['action_type'] == 'slowdown'


def test_navigation_startup_waits_for_localization_transform():
    launch_source = NAVIGATION_LAUNCH.read_text(encoding='utf-8')
    bridge_source = GOAL_BRIDGE.read_text(encoding='utf-8')

    assert "'autostart': False" in launch_source
    assert "'start_navigation_when_localized': ParameterValue(" in launch_source
    assert 'can_transform(' in bridge_source
    assert 'ManageLifecycleNodes.Request.STARTUP' in bridge_source
    assert "self._base_frame = self.get_parameter('base_frame').value" in (
        bridge_source
    )
    assert 'except RuntimeError:' in bridge_source
    assert 'if rclpy.ok():' in bridge_source
