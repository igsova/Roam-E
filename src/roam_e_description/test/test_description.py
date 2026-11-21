"""Offline checks of the robot description; they need xacro and PyYAML but no running ROS."""

import ast
import copy
import importlib.util
import math
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import xacro
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
WORKSPACE = PACKAGE.parents[1]
XACRO_FILE = PACKAGE / 'urdf/roam_e.urdf.xacro'
CALIBRATION = PACKAGE / 'config/calibration.yaml'
BUILD_SCRIPT = WORKSPACE / 'tools/description/build_description.py'
PACKAGE_URI = 'package://roam_e_description/'
OPTICAL_FRAMES = {
    'camera_rgb_optical_frame',
    'camera_depth_optical_frame',
    'camera_ir_optical_frame',
}
TOF_LINKS = {
    'tof_front_l_link',
    'tof_front_c_link',
    'tof_front_r_link',
    'tof_rear_l_link',
    'tof_rear_c_link',
    'tof_rear_r_link',
    'tof_left_link',
    'tof_right_link',
}


def expand(optics=False, calibration=CALIBRATION):
    document = xacro.process_file(
        str(XACRO_FILE),
        mappings={
            'include_camera_optical_frames': str(optics).lower(),
            'calibration_file': str(calibration),
        },
    )
    return ET.fromstring(document.toxml())


def joint_origin(root, joint, attribute='xyz'):
    element = root.find(f"joint[@name='{joint}']/origin")
    return [float(v) for v in element.get(attribute).split()]


def link_names(root):
    return {link.get('name') for link in root.findall('link')}


def load_calibration():
    return yaml.safe_load(CALIBRATION.read_text())


def write_calibration(config, folder):
    path = Path(folder) / 'calibration.yaml'
    path.write_text(yaml.safe_dump(config))
    return path


def launch_defaults(path):
    """Default values of the DeclareLaunchArgument calls in a launch file."""
    defaults = {}
    for node in ast.walk(ast.parse(path.read_text())):
        if isinstance(node, ast.Call) and getattr(node.func, 'id', '') == 'DeclareLaunchArgument':
            keywords = {k.arg: k.value for k in node.keywords}
            if isinstance(keywords.get('default_value'), ast.Constant):
                defaults[ast.literal_eval(node.args[0])] = keywords['default_value'].value
    return defaults


def test_camera_optical_frames_are_opt_in():
    default = ET.fromstring(xacro.process_file(str(XACRO_FILE)).toxml())
    assert 'camera_link' in link_names(default)
    assert not OPTICAL_FRAMES & link_names(default)
    assert link_names(expand(optics=True)) - link_names(expand(optics=False)) == OPTICAL_FRAMES
    defaults = launch_defaults(PACKAGE / 'launch/display.launch.py')
    assert defaults['include_camera_optical_frames'] == 'false'


def test_every_mesh_reference_resolves():
    meshes = expand(optics=True).findall('.//visual/geometry/mesh')
    assert meshes
    for mesh in meshes:
        uri = mesh.get('filename')
        assert uri.startswith(PACKAGE_URI)
        assert (PACKAGE / uri[len(PACKAGE_URI) :]).is_file(), uri


def test_urdf_uses_physical_wheel_geometry():
    wheels = load_calibration()['physical_wheels']
    root = expand()
    assert joint_origin(root, 'base_footprint_joint')[2] == pytest.approx(wheels['radius'])
    assert joint_origin(root, 'left_wheel_joint')[1] == pytest.approx(wheels['separation'] / 2)
    assert joint_origin(root, 'right_wheel_joint')[1] == pytest.approx(-wheels['separation'] / 2)
    cylinder = root.find("link[@name='left_wheel_link']/collision/geometry/cylinder")
    assert float(cylinder.get('radius')) == pytest.approx(wheels['radius'])
    assert float(cylinder.get('length')) == pytest.approx(wheels['width'])


def test_effective_odometry_does_not_change_the_urdf(tmp_path):
    # The floor-calibrated odometry values are for the odometry node only.
    config = load_calibration()
    config['effective_odometry']['radius'] *= 2
    config['effective_odometry']['separation'] *= 2
    edited = expand(calibration=write_calibration(config, tmp_path))
    assert ET.tostring(edited) == ET.tostring(expand())


def test_calibration_file_sets_sensor_frames(tmp_path):
    config = load_calibration()
    config['frames']['laser_frame']['xyz'] = [0.1, 0.2, 0.3]
    config['frames']['camera_link']['rpy'] = [0.0, 0.1, 0.2]
    root = expand(calibration=write_calibration(config, tmp_path))
    assert joint_origin(root, 'laser_frame_joint') == [0.1, 0.2, 0.3]
    assert joint_origin(root, 'camera_link_joint', 'rpy') == [0.0, 0.1, 0.2]


def test_tof_frames_follow_rep103():
    frames = load_calibration()['frames']
    root = expand()
    assert {n for n in link_names(root) if n.startswith('tof_')} == TOF_LINKS
    for name in TOF_LINKS:
        x, y, _ = joint_origin(root, f'{name}_joint')
        assert [x, y] == pytest.approx(frames[name]['xyz'][:2])
        station = name[len('tof_') : -len('_link')]
        if station.endswith(('_l', 'left')):
            assert y > 0, name
        elif station.endswith(('_r', 'right')):
            assert y < 0, name
        else:
            assert y == pytest.approx(0, abs=1e-6), name
    front = joint_origin(root, 'tof_front_c_link_joint')[0]
    rear = joint_origin(root, 'tof_rear_c_link_joint')[0]
    assert front > 0 > rear


def test_imu_axes_are_the_measured_mount():
    frames = load_calibration()['frames']
    # imu_mount_link is yawed by pi and imu_link by pi/2 on it: net -pi/2, Z up.
    assert frames['imu_mount_link']['rpy'] == pytest.approx([0, 0, math.pi], abs=1e-6)
    assert frames['imu_link']['rpy'] == pytest.approx([0, 0, math.pi / 2])


def test_committed_urdf_is_up_to_date():
    def strip(node):
        node.text = node.tail = None
        for child in node:
            strip(child)
        return node

    committed = strip(ET.parse(PACKAGE / 'urdf/roam_e.urdf').getroot())
    assert ET.tostring(committed) == ET.tostring(strip(expand(optics=True)))


@pytest.mark.skipif(
    not BUILD_SCRIPT.exists(), reason='needs the tools/ directory of the repository'
)
def test_regeneration_preserves_calibration(tmp_path):
    spec = importlib.util.spec_from_file_location('build_description', BUILD_SCRIPT)
    build = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(build)

    package = tmp_path / 'roam_e_description'
    # generate() reads the manifest and the xacro files, not the meshes.
    for directory in ('config', 'urdf'):
        shutil.copytree(PACKAGE / directory, package / directory)
    calibration = package / 'config/calibration.yaml'
    config = copy.deepcopy(load_calibration())
    config['frames']['laser_frame']['xyz'][2] += 0.001
    calibration.write_text(yaml.safe_dump(config))
    before = calibration.read_bytes()

    build.generate(package)

    assert calibration.read_bytes() == before
    laser_z = joint_origin(ET.parse(package / 'urdf/roam_e.urdf').getroot(), 'laser_frame_joint')
    assert laser_z[2] == pytest.approx(config['frames']['laser_frame']['xyz'][2])
    generated = (package / 'urdf/visuals.xacro').read_text()
    assert generated == (PACKAGE / 'urdf/visuals.xacro').read_text()
