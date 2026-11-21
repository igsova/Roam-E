#!/usr/bin/env python3
"""Check the generated robot description; needs no running ROS nodes or hardware.

Checks that the meshes match the manifest and the source .blend, the TF tree with and
without the camera optical frames, the sensor frame conventions (REP-103 ToF names,
optical axes, camera and IMU orientation), the wheel geometry, that an edited
calibration file changes the URDF, and that urdf/roam_e.urdf is up to date.
Requires the xacro and PyYAML Python packages.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
import struct
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import xacro
import yaml

PACKAGE = Path('src/roam_e_description')
PACKAGE_URI = 'package://roam_e_description/'
OPTICAL_FRAMES = {
    'camera_rgb_optical_frame',
    'camera_depth_optical_frame',
    'camera_ir_optical_frame',
}
WHEEL_SIGNS = {'left_wheel_link': 1, 'right_wheel_link': -1}
TOF_STATIONS = 8
# Objects that must never reach the visual meshes: the old LiDAR reference model and
# reserved-space placeholders.
FORBIDDEN_OBJECT = 'Component | LIDAR'
FORBIDDEN_MARKER = 'reserve'

EXACT = 1e-12
STL_HEADER_BYTES = 80
STL_TRIANGLE_BYTES = 50
MAX_MESH_COORDINATE_M = 1.0  # larger values mean a mesh is not in metres
MM_PER_M = 1000
MESH_BOUNDS_TOLERANCE_M = 1e-7
LOD_BOUNDS_TOLERANCE_M = 0.0005
AXIS_TOLERANCE = 1e-6
IMU_ROTATION_TOLERANCE = 1e-8
WHEEL_RADIUS_TOLERANCE_M = 2e-5
# The camera faces forward; measured mount corrections are a few degrees.
MAX_CAMERA_YAW_DEG = 10

# Edits applied to a copy of calibration.yaml to show that the URDF follows the file.
LASER_X_DELTA_M = 0.01
TOF_YAW_DELTA = 0.05
SEPARATION_DELTA_M = 0.002
RADIUS_DELTA_M = 0.001


class ValidationError(Exception):
    """A check on the robot description failed."""


def check(condition: bool, message: str) -> None:
    if not condition:
        raise ValidationError(message)


def max_difference(a, b) -> float:
    """Largest absolute difference between two equally shaped nested lists."""
    if isinstance(a, (list, tuple)):
        check(len(a) == len(b), f'Shape mismatch: {a} vs {b}')
        return max((max_difference(x, y) for x, y in zip(a, b, strict=True)), default=0.0)
    return abs(a - b)


def rotation_matrix(rpy) -> list[list[float]]:
    """Rotation matrix of URDF roll-pitch-yaw angles (fixed axes X, Y, Z)."""
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def determinant(m) -> float:
    return (
        m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
        - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
        + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
    )


def origin(joint, attribute: str = 'xyz') -> list[float]:
    return [float(v) for v in joint.find('origin').get(attribute, '0 0 0').split()]


def binary_stl_stats(path: Path) -> dict:
    """Triangle count and bounds of a binary STL, checking its size and coordinates."""
    with path.open('rb') as f:
        f.read(STL_HEADER_BYTES)
        (count,) = struct.unpack('<I', f.read(4))
        expected_size = STL_HEADER_BYTES + 4 + STL_TRIANGLE_BYTES * count
        check(path.stat().st_size == expected_size, f'{path.name}: size does not match {count}')
        check(count > 0, f'{path.name}: no triangles')
        low = [math.inf] * 3
        high = [-math.inf] * 3
        for _ in range(count):
            values = struct.unpack('<12fH', f.read(STL_TRIANGLE_BYTES))
            check(all(math.isfinite(v) for v in values[:12]), f'{path.name}: non-finite value')
            for vertex in (values[3:6], values[6:9], values[9:12]):
                for axis in range(3):
                    low[axis] = min(low[axis], vertex[axis])
                    high[axis] = max(high[axis], vertex[axis])
    extent = max(abs(v) for v in low + high)
    check(extent < MAX_MESH_COORDINATE_M, f'{path.name}: coordinate {extent} m is not in metres')
    return {'triangles': count, 'bounds_m': [list(pair) for pair in zip(low, high, strict=True)]}


def check_tree(root) -> tuple[set[str], dict]:
    """Check that the URDF is a single tree rooted at base_footprint.

    Returns the link names and the joints keyed by child link.
    """
    link_names = [n.get('name') for n in root.findall('link')]
    links = set(link_names)
    check(len(links) == len(link_names), 'Duplicate link names')
    joints = root.findall('joint')
    joint_names = [j.get('name') for j in joints]
    check(len(set(joint_names)) == len(joint_names), 'Duplicate joint names')

    parents = {}
    by_child = {}
    for joint in joints:
        name = joint.get('name')
        parent = joint.find('parent').get('link')
        child = joint.find('child').get('link')
        check(parent in links and child in links, f'{name} refers to a missing link')
        check(child not in parents, f'{child} has more than one parent')
        parents[child] = parent
        by_child[child] = joint
        for attribute in ('xyz', 'rpy'):
            values = origin(joint, attribute)
            check(
                len(values) == 3 and all(math.isfinite(v) for v in values),
                f'{name}: invalid {attribute} {values}',
            )

    roots = links - set(parents)
    check(roots == {'base_footprint'}, f'Expected the single root base_footprint, got {roots}')
    check(len(joints) == len(links) - 1, 'The joints do not form a tree')
    for child in parents:
        current, seen = child, set()
        while current in parents:
            check(current not in seen, f'TF cycle through {current}')
            seen.add(current)
            current = parents[current]
    return links, by_child


def check_source(manifest: dict, source_workspace: Path) -> None:
    source = source_workspace / manifest['source']
    digest = hashlib.sha256(source.read_bytes()).hexdigest()
    check(digest == manifest['source_sha256'], f'{source} has changed since the export')


def check_meshes(package: Path, manifest: dict, preview, config: dict) -> dict:
    """Check every mesh reference, scale, file hash, triangle count and bounds."""
    wheels = config['physical_wheels']
    wheel_scale = [
        wheels['radius'] / manifest['physical_wheel_radius_m'],
        wheels['width'] / manifest['physical_wheel_width_m'],
        wheels['radius'] / manifest['physical_wheel_radius_m'],
    ]
    referenced = set()
    for link in preview.findall('link'):
        for mesh in link.findall('visual/geometry/mesh'):
            uri = mesh.get('filename')
            check(uri.startswith(PACKAGE_URI), f'{uri} is not a package URI')
            path = (package / uri[len(PACKAGE_URI) :]).resolve()
            check(path.is_relative_to(package.resolve()) and path.is_file(), f'Missing {uri}')
            scale = [float(v) for v in mesh.get('scale').split()]
            expected = wheel_scale if link.get('name') in WHEEL_SIGNS else [1, 1, 1]
            check(max_difference(scale, expected) < EXACT, f'{uri}: scale {scale}')
            referenced.add(path.name)
    check(referenced == set(manifest['mesh_stats']), 'URDF meshes differ from the manifest')

    paths = {
        Path(v['mesh']).name: package / v['mesh']
        for visuals in manifest['visuals'].values()
        for v in visuals
    }
    stats = {}
    for name, expected in manifest['mesh_stats'].items():
        path = paths[name]
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        check(digest == expected['sha256'], f'{name}: file differs from the manifest')
        actual = binary_stl_stats(path)
        check(actual['triangles'] == expected['triangles'], f'{name}: triangle count')
        check(
            max_difference(actual['bounds_m'], expected['bounds_m']) < MESH_BOUNDS_TOLERANCE_M,
            f'{name}: bounds differ from the manifest',
        )
        stats[name] = actual
    return stats


def check_frames_match_calibration(joints: dict, config: dict) -> None:
    for child, frame in config['frames'].items():
        if child not in joints:
            continue
        check(joints[child].find('parent').get('link') == frame['parent'], f'{child}: parent')
        for attribute in ('xyz', 'rpy'):
            difference = max_difference(origin(joints[child], attribute), frame[attribute])
            check(difference < EXACT, f'{child}: URDF {attribute} differs from calibration.yaml')


def body_centre(manifest: dict) -> tuple[float, float]:
    """XY of the CAD origin (the body centre) in base_link, in metres."""
    rotation = manifest['cad_to_ros_rotation']
    offset = [-v / MM_PER_M for v in manifest['base_link_origin_cad_mm']]
    centre = [sum(rotation[row][i] * offset[i] for i in range(3)) for row in range(3)]
    return centre[0], centre[1]


def check_tof_frames(links: set[str], config: dict, manifest: dict) -> None:
    """ToF beams point outward with +Z up, and names follow REP-103 (left is +Y)."""
    names = sorted(n for n in links if n.startswith('tof_'))
    check(len(names) == TOF_STATIONS, f'Expected {TOF_STATIONS} ToF links, got {names}')
    centre_x, centre_y = body_centre(manifest)
    for name in names:
        frame = config['frames'][name]
        x, y = frame['xyz'][0], frame['xyz'][1]
        rotation = rotation_matrix(frame['rpy'])
        beam_out = rotation[0][0] * (x - centre_x) + rotation[1][0] * (y - centre_y)
        check(beam_out > 0, f'{name}: beam points into the body')
        check(abs(rotation[2][2] - 1) < AXIS_TOLERANCE, f'{name}: +Z is not up')

        words = name[len('tof_') : -len('_link')].split('_')
        if words[-1] in ('l', 'left'):
            check(y > 0, f'{name} is on the right (y = {y})')
        elif words[-1] in ('r', 'right'):
            check(y < 0, f'{name} is on the left (y = {y})')
        elif words[-1] == 'c':
            check(abs(y) < AXIS_TOLERANCE, f'{name} is off centre (y = {y})')
        if words[0] == 'front':
            check(x > centre_x, f'{name} is behind the body centre')
        elif words[0] == 'rear':
            check(x < centre_x, f'{name} is in front of the body centre')


def check_camera_frames(config: dict, manifest: dict) -> None:
    # Optical +Z looks along camera_link +X, optical +X is right and +Y is down.
    for name in sorted(OPTICAL_FRAMES):
        r = rotation_matrix(config['frames'][name]['rpy'])
        axes_ok = (
            abs(r[0][2] - 1) < AXIS_TOLERANCE
            and abs(r[1][0] + 1) < AXIS_TOLERANCE
            and abs(r[2][1] + 1) < AXIS_TOLERANCE
        )
        check(axes_ok, f'{name}: axes are not the REP-103 optical convention')
    yaw = config['frames']['camera_link']['rpy'][2]
    facing = 1 if manifest['forward'] == 'camera' else -1
    check(
        math.cos(yaw) * facing > math.cos(math.radians(MAX_CAMERA_YAW_DEG)),
        f'camera_link yaw {math.degrees(yaw):.1f} deg does not face the camera end',
    )


def check_imu(config: dict, expected_rpy) -> None:
    actual = rotation_matrix(config['frames']['imu_link']['rpy'])
    expected = rotation_matrix(expected_rpy)
    check(
        max_difference(actual, expected) < IMU_ROTATION_TOLERANCE,
        f'imu_link rpy {config["frames"]["imu_link"]["rpy"]} differs from {list(expected_rpy)}',
    )


def check_export_records(manifest: dict) -> None:
    """Wheel split, LOD accuracy and the set of exported objects."""
    counts = [w['rotating_vertices'] for w in manifest['wheels']]
    check(
        len(counts) == len(WHEEL_SIGNS) and len(set(counts)) == 1 and counts[0] > 0,
        f'The two wheels should have the same, non-zero number of rotating vertices: {counts}',
    )
    worst = max(r['bounds_error_m'] for r in manifest['source_objects'])
    check(worst <= LOD_BOUNDS_TOLERANCE_M, f'LOD bounds error {worst} m is too large')
    exported = {r['object'] for r in manifest['source_objects']}
    check(not exported & set(manifest['excluded_objects']), 'An excluded object was exported')
    check(FORBIDDEN_OBJECT not in exported, f'{FORBIDDEN_OBJECT} was exported')
    reserved = sorted(n for n in exported if FORBIDDEN_MARKER in n.lower())
    check(not reserved, f'Reserved-space objects were exported: {reserved}')


def check_wheels(joints: dict, config: dict, manifest: dict, meshes: dict) -> None:
    wheels = config['physical_wheels']
    radial_scale = wheels['radius'] / manifest['physical_wheel_radius_m']
    for link, sign in WHEEL_SIGNS.items():
        joint = joints[link]
        check(joint.get('type') == 'continuous', f'{link}: joint is not continuous')
        check(joint.find('axis').get('xyz') == '0 1 0', f'{link}: axis is not +Y')
        offset = origin(joint)[1] - sign * wheels['separation'] / 2
        check(abs(offset) < EXACT, f'{link}: joint is not at half the wheel separation')
        bounds = [meshes[Path(v['mesh']).name]['bounds_m'] for v in manifest['visuals'][link]]
        bottom = min(b[2][0] for b in bounds) * radial_scale
        top = max(b[2][1] for b in bounds) * radial_scale
        check(
            abs(bottom + wheels['radius']) < WHEEL_RADIUS_TOLERANCE_M
            and abs(top - wheels['radius']) < WHEEL_RADIUS_TOLERANCE_M,
            f'{link}: mesh radius does not match physical_wheels.radius',
        )


def check_calibration_override(expand, config: dict, joints: dict) -> None:
    """An edited calibration file must move frames and change the wheel geometry."""
    edited = copy.deepcopy(config)
    edited['frames']['laser_frame']['xyz'][0] += LASER_X_DELTA_M
    edited['frames']['tof_front_c_link']['rpy'][2] += TOF_YAW_DELTA
    edited['physical_wheels']['separation'] += SEPARATION_DELTA_M
    edited['physical_wheels']['radius'] += RADIUS_DELTA_M
    with tempfile.TemporaryDirectory(prefix='roam-e-calibration-') as folder:
        path = Path(folder) / 'override.yaml'
        path.write_text(yaml.safe_dump(edited))
        override = expand(path, optics=True)
    _, edited_joints = check_tree(override)

    def moved(link, attribute, axis):
        return origin(edited_joints[link], attribute)[axis] - origin(joints[link], attribute)[axis]

    tolerance = 1e-10
    check(abs(moved('laser_frame', 'xyz', 0) - LASER_X_DELTA_M) < tolerance, 'laser_frame')
    check(abs(moved('tof_front_c_link', 'rpy', 2) - TOF_YAW_DELTA) < tolerance, 'ToF yaw')
    check(
        abs(moved('left_wheel_link', 'xyz', 1) - SEPARATION_DELTA_M / 2) < tolerance,
        'Wheel separation did not follow the calibration file',
    )
    check(
        abs(moved('base_link', 'xyz', 2) - RADIUS_DELTA_M) < tolerance,
        'base_link height did not follow the wheel radius',
    )
    scales = [
        float(mesh.get('scale').split()[0])
        for mesh in override.findall("link[@name='left_wheel_link']/visual/geometry/mesh")
    ]
    check(scales and all(s > 1 for s in scales), 'Wheel meshes did not follow the wheel radius')


def check_urdf_up_to_date(package: Path, preview) -> None:
    """urdf/roam_e.urdf must equal the xacro expansion, ignoring whitespace."""
    committed = ET.parse(package / 'urdf/roam_e.urdf').getroot()

    def strip(node):
        node.text = None
        node.tail = None
        for child in node:
            strip(child)
        return node

    expected = strip(copy.deepcopy(preview))
    check(
        ET.tostring(strip(committed)) == ET.tostring(expected),
        'urdf/roam_e.urdf is stale; run tools/description/build_description.py',
    )


def validate(
    workspace: Path,
    source_workspace: Path | None = None,
    expected_imu_rpy=(0.0, 0.0, math.pi / 2),
) -> dict:
    """Run every check and return a summary; raises ValidationError on the first failure."""
    package = workspace / PACKAGE
    manifest = json.loads((package / 'config/mesh_manifest.json').read_text())
    check_source(manifest, source_workspace or workspace)
    ET.parse(package / 'package.xml')
    calibration = package / 'config/calibration.yaml'
    config = yaml.safe_load(calibration.read_text())
    check(
        config['forward'] == manifest['forward'],
        f'calibration.yaml is for forward={config["forward"]}, the meshes for '
        f'forward={manifest["forward"]}',
    )

    def expand(calibration_file=calibration, optics=True):
        document = xacro.process_file(
            str(package / 'urdf/roam_e.urdf.xacro'),
            mappings={
                'calibration_file': str(calibration_file),
                'include_camera_optical_frames': str(optics).lower(),
            },
        )
        return ET.fromstring(document.toxml())

    preview = expand(optics=True)
    live = expand(optics=False)
    links, joints = check_tree(preview)
    live_links, _ = check_tree(live)
    check(OPTICAL_FRAMES <= links, 'Optical frames missing with include_camera_optical_frames')
    check(not OPTICAL_FRAMES & live_links, 'Optical frames published by default')
    expected_links = {
        'base_footprint',
        'base_link',
        'imu_link',
        *manifest['visuals'],
        *OPTICAL_FRAMES,
    }
    check(links == expected_links, f'Unexpected links: {sorted(links ^ expected_links)}')

    meshes = check_meshes(package, manifest, preview, config)
    check_frames_match_calibration(joints, config)
    check_tof_frames(links, config, manifest)
    check_camera_frames(config, manifest)
    check_imu(config, expected_imu_rpy)
    check_export_records(manifest)
    check_wheels(joints, config, manifest, meshes)
    check_calibration_override(expand, config, joints)
    check_urdf_up_to_date(package, preview)
    check(
        abs(determinant(manifest['cad_to_ros_rotation']) - 1) < 1e-9,
        'The CAD to ROS rotation mirrors the geometry',
    )

    return {
        'status': 'PASS',
        'forward': manifest['forward'],
        'preview_links': len(links),
        'live_links': len(live_links),
        'mesh_files': len(meshes),
        'triangles': sum(m['triangles'] for m in meshes.values()),
        'tof_stations': TOF_STATIONS,
        'source_blend_unchanged': True,
        'calibration_override': 'PASS',
        'camera_tf_modes': 'PASS',
        'bounds_m': manifest['assembled_bounds_m'],
        'not_checked': [
            'Sensor extrinsics against the physical robot',
            'Runtime behaviour in robot_state_publisher and RViz',
        ],
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split('\n', 1)[0])
    parser.add_argument(
        'workspace', type=Path, nargs='?', default=Path('.'), help='Repository root'
    )
    parser.add_argument(
        '--source-workspace',
        type=Path,
        help='Checkout that holds mechanical/roam_e.blend, if not the workspace',
    )
    parser.add_argument(
        '--expected-imu-rpy',
        type=float,
        nargs=3,
        default=(0.0, 0.0, math.pi / 2),
        metavar=('ROLL', 'PITCH', 'YAW'),
        help='Expected imu_link rotation relative to imu_mount_link (default: 0 0 pi/2)',
    )
    args = parser.parse_args()
    workspace = args.workspace.resolve()
    try:
        result = validate(workspace, args.source_workspace, args.expected_imu_rpy)
    except ValidationError as error:
        print(f'FAIL: {error}', file=sys.stderr)
        sys.exit(1)
    out = workspace / 'build/description'
    out.mkdir(parents=True, exist_ok=True)
    (out / 'validation.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
