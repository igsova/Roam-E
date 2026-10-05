#!/usr/bin/env python3
"""Prepare an isolated R19 camera-forward description, never production deploy.

Requires Blender, xacro, PyYAML. Creates a NEW workspace with COLCON_IGNORE
at its root so normal parent-workspace discovery cannot find a duplicate
package. Build it only from inside that workspace using --base-paths src.
All hardware launches, Nav2, firmware and saved IMU offsets remain untouched.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import shutil
import subprocess
import zipfile

import yaml
import xacro

import build_r19_description
import validate_r19_description as validate_description

ROOT = Path(__file__).resolve().parents[1]
IMU_RELATIVE_RPY = (0.0, 0.0, math.pi / 2)


def hashes(root):
    return {str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in sorted(root.rglob('*')) if p.is_file()
            and not any(part in ('__pycache__', '.pytest_cache') for part in p.parts)}


def compose(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)]
            for i in range(3)]


def candidate_checks(baseline, package):
    old = json.loads((baseline / 'config/mesh_export_r19.json').read_text())
    new = json.loads((package / 'config/mesh_export_r19.json').read_text())
    config = yaml.safe_load((package / 'config/calibration.yaml').read_text())
    geometry = yaml.safe_load((package / 'config/geometry.yaml').read_text())
    assert old['forward'] == 'caster' and new['forward'] == config['forward'] == 'camera'
    half_turn = [[-1, 0, 0], [0, -1, 0], [0, 0, 1]]
    rotate = lambda p: [-p[0], -p[1], p[2]]
    # Every non-panel sensor origin/orientation must be a coordinate change,
    # not a physical relocation. Working IMU orientation is checked separately.
    sensors = ['laser_frame', 'imu_mount_link', 'camera_link'] + [
        name for name in old['frames'] if name.startswith('tof_')]
    for name in sensors:
        a, b = old['frames'][name], new['frames'][name]
        assert max(abs(x-y) for x,y in zip(rotate(a['xyz']), b['xyz'])) < 1e-7, name
        expected = compose(half_turn, validate_description.rotation(a['rpy']))
        actual = validate_description.rotation(b['rpy'])
        assert max(abs(x-y) for aa,bb in zip(expected,actual)
                   for x,y in zip(aa,bb)) < 1e-6, name
    original_wheels = {w['object']: w['wheel_link'] for w in old['wheel_separation_checks']}
    for w in new['wheel_separation_checks']:
        assert w['wheel_link'] != original_wheels[w['object']]
    assert config['physical_wheels'] == yaml.safe_load(
        (baseline / 'config/calibration.yaml').read_text())['physical_wheels']
    assert config['effective_odometry'] == yaml.safe_load(
        (baseline / 'config/calibration.yaml').read_text())['effective_odometry']
    old_hull = {(-p[0], -p[1]) for p in old['navigation_footprint_candidate']}
    new_hull = {tuple(p) for p in new['navigation_footprint_candidate']}
    # Independent LOD export may differ by float32 rounding; compare nearest
    # hull coordinates and assembled bounds within 0.1 mm, not string equality.
    for source, target in ((old_hull, new_hull), (new_hull, old_hull)):
        assert all(min(math.hypot(p[0]-q[0], p[1]-q[1]) for q in target) < .0001
                   for p in source)
    assert geometry['navigation_footprint_candidate']['points'] == new['navigation_footprint_candidate']
    for i, sign in enumerate((-1, -1, 1)):
        expected = ([sign * old['assembled_bounds_m'][i][1], sign * old['assembled_bounds_m'][i][0]]
                    if sign == -1 else old['assembled_bounds_m'][i])
        assert max(abs(x-y) for x,y in zip(expected, new['assembled_bounds_m'][i])) < .0001
    mount = validate_description.rotation(config['frames']['imu_mount_link']['rpy'])
    native = validate_description.rotation(config['frames']['imu_link']['rpy'])
    total = compose(mount, native)
    expected = [[0, 1, 0], [-1, 0, 0], [0, 0, 1]]
    assert max(abs(x-y) for aa,bb in zip(total,expected) for x,y in zip(aa,bb)) < 1e-6
    return {'sensor_frame_reexpression': 'PASS', 'physical_wheel_label_swap': 'PASS',
            'footprint_half_turn': 'PASS', 'assembled_bounds_half_turn': 'PASS',
            'native_imu_to_base_rotation': total,
            'imu_direction_evidence': '2026-10-05 manual tilts and confirmed CCW turn',
            'imu_exact_angles': 'Not metrologically fitted; cardinal-axis candidate',
            'accelerometer_calibration': '1/3 at capture; incomplete',
            'nav2_or_drive_activation': False}


def prepare(output, blender, baseline=None):
    output = output.resolve()
    baseline = (baseline or ROOT / 'src/roam_e_description').resolve()
    if output.exists():
        raise ValueError('Output already exists; use a new isolated workspace')
    if output == ROOT or output.is_relative_to(ROOT / 'src'):
        raise ValueError('Candidate must not be written into the source workspace src tree')
    if output.is_relative_to(baseline):
        raise ValueError('Candidate must not be inside the baseline package')
    if not (baseline / 'config/mesh_export_r19.json').is_file():
        raise ValueError('An R19 baseline package is required. The published runtime may '
                         'still be the legacy description: supply --baseline-package '
                         'with the preserved standalone R19 package and its meshes.')
    before = hashes(baseline)
    output.mkdir(parents=True)
    (output / 'COLCON_IGNORE').write_text('Isolated candidate; build explicitly from inside it.\n')
    package = output / 'src/roam_e_description'
    shutil.copytree(baseline, package, ignore=shutil.ignore_patterns('__pycache__', '.pytest_cache'))
    # Start fresh output meshes, preserving the copied baseline ones in a
    # recoverable backup directory outside the ROS package (never deleting).
    shutil.move(str(package / 'meshes/r19'), str(output / 'baseline_meshes_backup'))
    command = [str(blender), '--background', '--disable-autoexec', '--python',
               str(ROOT / 'tools/export_blender_description.py'), '--',
               '--package', str(package), '--forward', 'camera']
    subprocess.run(command, check=True)
    build_r19_description.generate(package, reset=True)
    config_path = package / 'config/calibration.yaml'
    config = yaml.safe_load(config_path.read_text())
    config['revision'] = 'R19 camera-forward isolated candidate 2026-10-05; NOT production'
    config['frames']['imu_link']['rpy'] = list(IMU_RELATIVE_RPY)
    config['frames']['imu_link']['status'] = (
        'Axis-direction checks only; parent nominal basis yaw=pi gives net native-to-base '
        'yaw=-pi/2, Z up. Accel calibration 1/3; xyz CAD nominal; no exact-angle fit.')
    config['activation'] = {'production': False, 'motor_commands': False,
        'pending': ['accelerometer calibration quality', 'measured sensor extrinsics',
                    'LiDAR forward heading', 'new footprint clearance',
                    'physical motor/encoder wheel mapping and velocity signs']}
    config_path.write_text('# ISOLATED CANDIDATE ONLY. Do not use with production drive/Nav2.\n'
                           + yaml.safe_dump(config, sort_keys=False))
    tree = xacro.process_file(str(package / 'urdf/roam_e.urdf.xacro'), mappings={
        'calibration_file': str(config_path), 'include_camera_optical_frames': 'true'})
    (package / 'urdf/roam_e.urdf').write_text(tree.toprettyxml(indent='  '))
    report = validate_description.validate(output, ROOT, IMU_RELATIVE_RPY)
    report['camera_forward_candidate_checks'] = candidate_checks(baseline, package)
    assert hashes(baseline) == before, 'Original source package changed'
    report['original_source_package_unchanged'] = True
    (output / 'validation.json').write_text(json.dumps(report, indent=2) + '\n')
    # Replace only the candidate copy's legacy README, to avoid contradictory
    # caster-forward instructions travelling with this standalone package.
    (package / 'README.md').write_text(
        '# ISOLATED R19 camera-forward candidate — 2026-10-05\n\n'
        '+X faces the camera, +Y is physical left, +Z up. Caster is rear.\n'
        'Left/right wheel labels are swapped relative to the old model; firmware is NOT changed.\n'
        'The candidate IMU net native-to-base yaw is -pi/2, with Z up.\n'
        'Direction checks passed; exact angles/positions and accel calibration remain provisional.\n'
        'Never source this candidate together with production or launch drive/navigation with it.\n'
        'Use only a separate ROS_DOMAIN_ID and description-only preview.\n'
        'Camera optical preview TF remains off by default; OpenNI retains ownership for live use.\n'
        'Footprint is a rotated CAD candidate, NOT a physically verified Nav2 footprint.\n')
    print(json.dumps({'workspace': str(output), **report}, indent=2))


def package_candidate(output):
    """Bundle only active description sources/meshes, not baseline backups/CAD."""
    output = output.resolve()
    package = output / 'src/roam_e_description'
    report = json.loads((output / 'validation.json').read_text())
    assert report['status'] == 'PASS' and report['forward'] == 'camera'
    manifest = json.loads((package / 'config/mesh_export_r19.json').read_text())
    files = [package / name for name in ('package.xml', 'CMakeLists.txt', 'README.md')]
    for directory in ('config', 'launch', 'rviz', 'scripts', 'test'):
        files += [p for p in (package / directory).rglob('*') if p.is_file()
                  and not any(part in ('__pycache__', '.pytest_cache') for part in p.parts)]
    files += [package / 'urdf' / name for name in
              ('roam_e.urdf.xacro', 'roam_e.urdf', 'r19_visuals.xacro')]
    files += [package / 'meshes/r19' / name for name in manifest['mesh_stats']]
    dest = output / 'R19_CAMERA_FORWARD_CANDIDATE.zip'
    if dest.exists():
        raise ValueError('Candidate archive already exists; refusing overwrite')
    checks = {}
    with zipfile.ZipFile(dest, 'w', zipfile.ZIP_DEFLATED) as archive:
        for path in sorted(files):
            name = 'src/roam_e_description/' + str(path.relative_to(package))
            archive.write(path, name)
            checks[name] = hashlib.sha256(path.read_bytes()).hexdigest()
        archive.write(output / 'validation.json', 'validation.json')
        archive.write(output / 'COLCON_IGNORE', 'COLCON_IGNORE')
    with zipfile.ZipFile(dest) as archive:
        assert archive.testzip() is None
        for name, digest in checks.items():
            assert hashlib.sha256(archive.read(name)).hexdigest() == digest
    result = {'archive': dest.name, 'sha256': hashlib.sha256(dest.read_bytes()).hexdigest(),
              'bytes': dest.stat().st_size, 'package_files': len(files),
              'files_sha256': checks, 'zip_checks': 'PASS', 'production_deployment': False}
    (output / 'package_checks.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps({k: v for k, v in result.items() if k != 'files_sha256'}, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--blender', type=Path)
    parser.add_argument('--baseline-package', type=Path,
                        help='Preserved standalone caster-forward R19 package including meshes')
    parser.add_argument('--package-only', action='store_true',
                        help='Package an already validated isolated candidate; no export')
    args = parser.parse_args()
    if args.package_only:
        package_candidate(args.output)
    elif args.blender is None:
        parser.error('--blender is required for preparation')
    else:
        prepare(args.output, args.blender, args.baseline_package)
