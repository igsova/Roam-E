#!/usr/bin/env python3
"""Generate an OFFLINE navigation candidate in a new directory, never deploy.

Requires PyYAML, a preserved camera-forward CAD manifest/calibration and the
JSON stdout report from check_r19_caster_sweep.py. No ROS or hardware access.
Only footprints/padding and collision-monitor point lists may change relative
to the supplied Nav2 baseline. Launch defaults and the source assets are read-only.
"""
import argparse
import copy
import hashlib
import json
import math
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
PADDING = 0.01
# Proposed static detection clearances FROM the padded body's bounding box.
# These are not measured stopping distances or qualified safety thresholds.
CLEARANCES = {
    'StopZone': {'front': 0.30, 'rear': 0.15, 'side': 0.10},
    'SlowdownZone': {'front': 0.50, 'rear': 0.25, 'side': 0.20},
}


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def cross(a, b, p):
    return (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0])


def hull(points):
    points = sorted(set(tuple(p) for p in points))
    halves = []
    for sequence in (points, list(reversed(points))):
        half = []
        for point in sequence:
            while len(half) >= 2 and cross(half[-2], half[-1], point) <= 0:
                half.pop()
            half.append(point)
        halves.append(half[:-1])
    return [list(p) for p in halves[0] + halves[1]]


def validate_polygon(points):
    if len(points) < 3 or any(len(p) != 2 for p in points):
        raise ValueError('Expected at least three XY pairs')
    if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v)
           for p in points for v in p):
        raise ValueError('Polygon coordinates must be finite numbers')
    if len(set(tuple(p) for p in points)) != len(points):
        raise ValueError('Duplicate footprint vertex')
    expected = hull(points)
    if len(expected) != len(points):
        raise ValueError('Footprint must be convex, without collinear vertices')
    # Check actual ordering too; a shuffled point set is not a usable polygon.
    start = expected.index(list(points[0]))
    if list(map(list, points)) != expected[start:] + expected[:start]:
        raise ValueError('Footprint must be in counterclockwise boundary order')
    if any(cross(a, b, [0, 0]) < -1e-12
           for a, b in zip(points, points[1:] + points[:1])):
        raise ValueError('Footprint must contain the axle origin')


def disc_clearance(points, centre, radius):
    validate_polygon(points)
    if radius <= 0 or not math.isfinite(radius) or len(centre) != 2 or not all(
            math.isfinite(v) for v in centre):
        raise ValueError('Invalid caster disc')
    return min(cross(a, b, centre) / math.dist(a, b) - radius
               for a, b in zip(points, points[1:] + points[:1]))


def cover_caster(points, centre, radius):
    clearance = disc_clearance(points, centre, radius)
    if clearance >= 0:
        return copy.deepcopy(points), clearance
    # If a future caster sticks out, enclose its entire analytic disc using a
    # circumscribed square, not points sampled ON a circle (which underbound it).
    corners = [[centre[0] + sx * radius, centre[1] + sy * radius]
               for sx in (-1, 1) for sy in (-1, 1)]
    result = hull(points + corners)
    validate_polygon(result)
    return result, clearance


def nav2_pad(points, padding):
    if padding < 0 or not math.isfinite(padding):
        raise ValueError('Invalid padding')
    # Jazzy padFootprint adds sign0(coordinate)*padding on EACH coordinate;
    # this is NOT an exact Euclidean constant-distance polygon offset.
    return [[v + (padding if v > 0 else -padding if v < 0 else 0)
             for v in point] for point in points]


def bounds(points):
    return [[min(p[i] for p in points), max(p[i] for p in points)] for i in (0, 1)]


def zone(points, clearances):
    if set(clearances) != {'front', 'rear', 'side'} or any(
            not math.isfinite(v) or v <= 0 for v in clearances.values()):
        raise ValueError('Zone clearances must be positive and finite')
    (xmin, xmax), (ymin, ymax) = bounds(points)
    # Outward rounding to mm keeps requested clearance even at float boundaries.
    low = lambda v: math.floor(v * 1000 + 1e-9) / 1000
    high = lambda v: math.ceil(v * 1000 - 1e-9) / 1000
    xmin, xmax = low(xmin - clearances['rear']), high(xmax + clearances['front'])
    ymin, ymax = low(ymin - clearances['side']), high(ymax + clearances['side'])
    return [[xmin, ymin], [xmax, ymin], [xmax, ymax], [xmin, ymax]]


def allowed_changes_only(baseline, candidate):
    restored = copy.deepcopy(candidate)
    for name in ('local_costmap', 'global_costmap'):
        old = baseline[name][name]['ros__parameters']
        new = restored[name][name]['ros__parameters']
        for key in ('footprint', 'footprint_padding'):
            new[key] = old[key]
    old = baseline['collision_monitor']['ros__parameters']
    new = restored['collision_monitor']['ros__parameters']
    for name in CLEARANCES:
        new[name]['points'] = old[name]['points']
    if restored != baseline:
        raise ValueError('Unexpected non-geometry parameter change')


def build(baseline, manifest, calibration, caster):
    if manifest['forward'] != 'camera' or calibration['forward'] != 'camera':
        raise ValueError('Camera-forward manifest AND calibration are required')
    if caster['source_sha256'] != manifest['source_sha256'] or not caster['source_unchanged']:
        raise ValueError('Caster and CAD manifest provenance mismatch')
    if caster['physical_swivel_axis_verified'] is not False:
        raise ValueError('Unexpected claim of physical caster verification')
    camera = calibration['frames']['camera_link']
    if camera['parent'] != 'base_link' or any(abs(v) > 1e-6 for v in camera['rpy']):
        raise ValueError('Camera body does not point along base +X')
    matrix = manifest['cad_to_ros_rotation']
    if matrix != [[0, -1, 0], [1, 0, 0], [0, 0, 1]]:
        raise ValueError('Unexpected CAD-to-camera-forward rotation')
    origin = manifest['base_link_origin_cad_mm']
    xy = caster['nominal_pivot_cad_xy_mm']
    delta = [xy[0] - origin[0], xy[1] - origin[1], 0]
    centre = [sum(matrix[i][j] * delta[j] for j in range(3)) / 1000 for i in (0, 1)]
    original = manifest['navigation_footprint_candidate']
    points, clearance = cover_caster(original, centre, caster['sweep_radius_outward_rounded_m'])
    padded = nav2_pad(points, PADDING)
    candidate = copy.deepcopy(baseline)
    for name in ('local_costmap', 'global_costmap'):
        params = candidate[name][name]['ros__parameters']
        if params['robot_base_frame'] != 'base_footprint' or 'robot_radius' in params:
            raise ValueError('Unexpected costmap base frame or radius override')
        params['footprint'] = json.dumps(points, separators=(',', ':'))
        params['footprint_padding'] = PADDING
    monitor = candidate['collision_monitor']['ros__parameters']
    if monitor['base_frame_id'] != 'base_footprint':
        raise ValueError('Collision monitor base-frame mismatch')
    zones = {}
    for name, clearances in CLEARANCES.items():
        polygon = zone(padded, clearances)
        zones[name] = polygon
        old = yaml.safe_load(monitor[name]['points'])
        # Do not shrink the original baseline's detection rectangle.
        (xl, xh), (yl, yh) = bounds(polygon)
        if any(not (xl <= x <= xh and yl <= y <= yh) for x, y in old):
            raise ValueError('Candidate zone would shrink baseline detection extent')
        monitor[name]['points'] = json.dumps(polygon, separators=(',', ':'))
    allowed_changes_only(baseline, candidate)
    report = {
        'status': 'OFFLINE_CANDIDATE', 'forward': 'camera', 'base_frame': 'base_footprint',
        'activation': {'pi_deployment': False, 'robot_motion': False, 'production_default': False},
        'caster': {'nominal_centre_base_xy_m': centre,
                   'analytic_disc_radius_m': caster['sweep_radius_outward_rounded_m'],
                   'original_hull_disc_clearance_m': clearance,
                   'hull_extended_for_caster': points != original,
                   'physical_swivel_axis_verified': False},
        'footprint_vertices': len(points), 'footprint_points_m': points,
        'body_bounds_xy_m': bounds(points), 'nav2_padding_parameter_m': PADDING,
        'nav2_padded_bounds_xy_m': bounds(padded), 'padded_points_m': padded,
        'zones': {name: {'points_m': polygon, 'clearances_from_padded_bbox_m': CLEARANCES[name]}
                  for name, polygon in zones.items()},
        'non_geometry_parameters_unchanged': True,
        'limits': ['CAD geometry only; external harness/assembled protrusions unverified.',
                   'Caster axis is a symmetry inference, not physical metrology.',
                   '10 mm Nav2 padding and static zones are proposals, not safety acceptance.',
                   'Zones use only existing /scan; low/high obstacles and close blind areas remain.',
                   'Stopping distance, wheel signs, odometry and sensor extrinsics unqualified.',
                   'Existing launch chain is not yet integrated with camera-forward TF/drive.',
                   'No ROS/Nav2 runtime or Foxglove check was performed.'],
    }
    return candidate, report


def prepare(output, baseline_path, manifest_path, calibration_path, caster_path):
    output = output.resolve()
    if output.exists():
        raise ValueError('Output already exists; use a new directory')
    if output == ROOT or output.is_relative_to(ROOT / 'src') or output.is_relative_to(ROOT / '.git'):
        raise ValueError('Output must be an isolated directory outside runtime src/.git')
    inputs = [baseline_path, manifest_path, calibration_path, caster_path]
    if any(path.resolve().is_relative_to(output) for path in inputs):
        raise ValueError('Output must not contain its own input files')
    digests = {str(p.resolve()): sha(p) for p in inputs}
    manifest = json.loads(manifest_path.read_text())
    source = ROOT / manifest['source']
    if sha(source) != manifest['source_sha256']:
        raise ValueError('Current Blender source does not match exported manifest')
    candidate, report = build(yaml.safe_load(baseline_path.read_text()), manifest,
                              yaml.safe_load(calibration_path.read_text()),
                              json.loads(caster_path.read_text()))
    report['inputs_sha256'] = digests
    report['blender_source_sha256'] = manifest['source_sha256']
    output.mkdir(parents=True)
    banner = '# OFFLINE R19 CAMERA-FORWARD CANDIDATE — NOT APPROVED FOR ROBOT USE.\n'
    banner += '# Not selected by any launch. Read ../R19_NAVIGATION_GEOMETRY_CANDIDATE.md.\n'
    (output / 'nav2.yaml').write_text(banner + yaml.safe_dump(candidate, sort_keys=False))
    (output / 'caster_sweep.json').write_text(caster_path.read_text())
    (output / 'geometry_report.json').write_text(json.dumps(report, indent=2) + '\n')
    if any(sha(p) != digests[str(p.resolve())] for p in inputs) or sha(source) != manifest['source_sha256']:
        raise ValueError('An input changed during preparation')
    print(json.dumps({'output': str(output), 'caster': report['caster'],
                      'body_bounds_xy_m': report['body_bounds_xy_m'],
                      'zones': report['zones'], 'activation': report['activation']}, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--baseline', type=Path, default=ROOT / 'src/roam_e/config/nav2.yaml')
    parser.add_argument('--manifest', required=True, type=Path)
    parser.add_argument('--calibration', required=True, type=Path)
    parser.add_argument('--caster-report', required=True, type=Path)
    args = parser.parse_args()
    prepare(args.output, args.baseline, args.manifest, args.calibration, args.caster_report)
