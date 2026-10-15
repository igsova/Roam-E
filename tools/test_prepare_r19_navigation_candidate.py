"""Offline tests; standard unittest + PyYAML, no ROS/Blender/hardware access."""
import contextlib
import copy
import io
import json
import math
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

import yaml

import prepare_r19_navigation_candidate as nav

ARTIFACTS = nav.ROOT / 'docs/navigation/r19_candidate'
BASELINE = nav.ROOT / 'src/roam_e/config/nav2.yaml'


def fixture():
    baseline = yaml.safe_load(BASELINE.read_text())
    manifest = {'forward': 'camera', 'source_sha256': 'test-source',
                'cad_to_ros_rotation': [[0, -1, 0], [1, 0, 0], [0, 0, 1]],
                'base_link_origin_cad_mm': [0, -37, 32.5],
                'navigation_footprint_candidate': [
                    [-0.143811, -0.12], [0.06965, -0.12],
                    [0.06965, 0.11906], [-0.143811, 0.11906]]}
    calibration = {'forward': 'camera', 'frames': {
        'camera_link': {'parent': 'base_link', 'rpy': [0, 0, 0]}}}
    caster = {'source_sha256': 'test-source', 'source_unchanged': True,
              'physical_swivel_axis_verified': False,
              'nominal_pivot_cad_xy_mm': [0, 57.75795936584473],
              'sweep_radius_outward_rounded_m': 0.034}
    return baseline, manifest, calibration, caster


class GeometryMath(unittest.TestCase):
    def setUp(self):
        self.square = [[-1, -1], [1, -1], [1, 1], [-1, 1]]

    def test_analytic_disc_contains_every_angle_not_just_samples(self):
        self.assertAlmostEqual(nav.disc_clearance(self.square, [0, 0], 0.5), 0.5)
        self.assertLess(nav.disc_clearance(self.square, [0.8, 0], 0.3), 0)

    def test_tangent_disc_does_not_force_extension(self):
        result, clearance = nav.cover_caster(self.square, [0, 0], 1)
        self.assertEqual(result, self.square)
        self.assertEqual(clearance, 0)

    def test_outside_disc_uses_circumscribed_envelope(self):
        points, clearance = nav.cover_caster(self.square, [1.2, 0], 0.5)
        self.assertLess(clearance, 0)
        self.assertGreaterEqual(nav.disc_clearance(points, [1.2, 0], 0.5), -1e-12)
        for a, b in zip(points, points[1:] + points[:1]):
            for p in self.square:
                self.assertGreaterEqual(nav.cross(a, b, p), -1e-12)

    def test_invalid_radius_and_centre_are_rejected(self):
        for centre, radius in (([0, 0], 0), ([0, 0], math.nan),
                               ([math.inf, 0], 1), ([0], 1)):
            with self.subTest(centre=centre, radius=radius), self.assertRaises(ValueError):
                nav.disc_clearance(self.square, centre, radius)

    def test_reversed_shuffled_and_concave_polygons_are_rejected(self):
        for polygon in (self.square[::-1], [self.square[i] for i in (0, 2, 1, 3)],
                        [[-1, -1], [1, -1], [0, 0], [1, 1], [-1, 1]]):
            with self.subTest(polygon=polygon), self.assertRaises(ValueError):
                nav.validate_polygon(polygon)

    def test_nonfinite_duplicate_collinear_and_outside_origin_rejected(self):
        for polygon in ([[[math.nan, 0], [1, 0], [0, 1]], self.square + [self.square[0]],
                         [[0, 0], [1, 0], [2, 0]], [[1, 1], [2, 1], [2, 2], [1, 2]]]):
            with self.subTest(polygon=polygon), self.assertRaises(ValueError):
                nav.validate_polygon(polygon)

    def test_jazzy_padding_matches_sign0_per_coordinate(self):
        self.assertEqual(nav.nav2_pad([[-0.1, 0], [0.1, 0.2]], 0.01),
                         [[-0.11, 0], [0.11, 0.21000000000000002]])
        with self.assertRaises(ValueError):
            nav.nav2_pad(self.square, -0.01)

    def test_zone_rounding_is_outward_not_nearest(self):
        points = [[-0.153811, -0.13], [0.07965, -0.13],
                  [0.07965, 0.12906], [-0.153811, 0.12906]]
        result = nav.zone(points, nav.CLEARANCES['StopZone'])
        self.assertEqual(nav.bounds(result), [[-0.304, 0.38], [-0.23, 0.23]])

    def test_clearance_parameters_reject_negative_nan_and_missing_keys(self):
        for values in ({'front': 0, 'rear': 1, 'side': 1},
                       {'front': math.nan, 'rear': 1, 'side': 1}, {'front': 1}):
            with self.subTest(values=values), self.assertRaises(ValueError):
                nav.zone(self.square, values)


class CandidateSafety(unittest.TestCase):
    def test_only_allowed_geometry_fields_change(self):
        baseline, manifest, calibration, caster = fixture()
        before = copy.deepcopy(baseline)
        candidate, report = nav.build(baseline, manifest, calibration, caster)
        nav.allowed_changes_only(baseline, candidate)
        self.assertEqual(baseline, before)
        self.assertTrue(report['non_geometry_parameters_unchanged'])
        self.assertFalse(any(report['activation'].values()))

    def test_costmaps_share_footprint_padding_and_base_frame(self):
        candidate, _ = nav.build(*fixture())
        local = candidate['local_costmap']['local_costmap']['ros__parameters']
        global_ = candidate['global_costmap']['global_costmap']['ros__parameters']
        self.assertEqual(local['footprint'], global_['footprint'])
        self.assertEqual(local['footprint_padding'], global_['footprint_padding'])
        self.assertEqual(local['robot_base_frame'], 'base_footprint')

    def test_nested_zones_contain_padded_body_and_all_baseline_vertices(self):
        baseline, manifest, calibration, caster = fixture()
        _, report = nav.build(baseline, manifest, calibration, caster)
        stop = report['zones']['StopZone']['points_m']
        slow = report['zones']['SlowdownZone']['points_m']
        for name, points in (('StopZone', report['padded_points_m']),
                             ('SlowdownZone', stop)):
            (xl, xh), (yl, yh) = nav.bounds(report['zones'][name]['points_m'])
            old = yaml.safe_load(baseline['collision_monitor']['ros__parameters'][name]['points'])
            self.assertTrue(all(xl <= x <= xh and yl <= y <= yh for x, y in points + old))
        self.assertEqual(nav.bounds(slow), [[-0.404, 0.58], [-0.33, 0.33]])

    def test_velocity_gain_timeout_and_command_chain_changes_are_forbidden(self):
        baseline, manifest, calibration, caster = fixture()
        candidate, _ = nav.build(baseline, manifest, calibration, caster)
        for node, key, value in (('velocity_smoother', 'max_velocity', [0.2, 0, 1]),
                                 ('collision_monitor', 'source_timeout', 1),
                                 ('collision_monitor', 'cmd_vel_out_topic', 'unsafe')):
            changed = copy.deepcopy(candidate)
            changed[node]['ros__parameters'][key] = value
            with self.subTest(node=node, key=key), self.assertRaises(ValueError):
                nav.allowed_changes_only(baseline, changed)

    def test_wrong_forward_metadata_provenance_or_camera_axes_are_rejected(self):
        for failure in ('manifest_forward', 'calibration_forward', 'hash', 'camera_yaw', 'matrix'):
            baseline, manifest, calibration, caster = fixture()
            if failure == 'manifest_forward':
                manifest['forward'] = 'caster'
            elif failure == 'calibration_forward':
                calibration['forward'] = 'caster'
            elif failure == 'hash':
                caster['source_sha256'] = 'other'
            elif failure == 'camera_yaw':
                calibration['frames']['camera_link']['rpy'][2] = math.pi
            else:
                manifest['cad_to_ros_rotation'][0][1] = 1
            with self.subTest(failure=failure), self.assertRaises(ValueError):
                nav.build(baseline, manifest, calibration, caster)

    def test_unexpected_radius_override_and_frame_changes_rejected(self):
        for failure in ('radius', 'costmap_frame', 'monitor_frame', 'physical_claim'):
            baseline, manifest, calibration, caster = fixture()
            local = baseline['local_costmap']['local_costmap']['ros__parameters']
            if failure == 'radius':
                local['robot_radius'] = 0.1
            elif failure == 'costmap_frame':
                local['robot_base_frame'] = 'base_link'
            elif failure == 'monitor_frame':
                baseline['collision_monitor']['ros__parameters']['base_frame_id'] = 'other'
            else:
                caster['physical_swivel_axis_verified'] = True
            with self.subTest(failure=failure), self.assertRaises(ValueError):
                nav.build(baseline, manifest, calibration, caster)

    def test_existing_output_and_runtime_directory_are_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            for output in (Path(folder), nav.ROOT / 'src/new_candidate'):
                with self.subTest(output=output), self.assertRaises(ValueError):
                    nav.prepare(output, *([Path('/missing')] * 4))

    def test_complete_generation_preserves_inputs_and_writes_no_launch(self):
        baseline, manifest, calibration, caster = fixture()
        with tempfile.TemporaryDirectory() as folder:
            folder = Path(folder)
            source = folder / 'source.blend'
            source.write_bytes(b'test source placeholder; never opened in Blender')
            manifest['source'] = str(source)
            manifest['source_sha256'] = caster['source_sha256'] = nav.sha(source)
            inputs = [folder / name for name in ('baseline.yaml', 'manifest.json', 'cal.yaml', 'caster.json')]
            for path, value, is_json in zip(inputs, (baseline, manifest, calibration, caster), (False, True, False, True)):
                path.write_text(json.dumps(value) if is_json else yaml.safe_dump(value))
            before = [nav.sha(p) for p in [source] + inputs]
            with contextlib.redirect_stdout(io.StringIO()):
                nav.prepare(folder / 'output', *inputs)
            self.assertEqual([nav.sha(p) for p in [source] + inputs], before)
            self.assertEqual({p.name for p in (folder / 'output').iterdir()},
                             {'nav2.yaml', 'caster_sweep.json', 'geometry_report.json'})


class SavedCandidateRegression(unittest.TestCase):
    def test_padded_hull_is_convex_and_contains_the_original_hull(self):
        report = json.loads((ARTIFACTS / 'geometry_report.json').read_text())
        points = report['padded_points_m']
        nav.validate_polygon(points)
        for a, b in zip(points, points[1:] + points[:1]):
            self.assertTrue(all(nav.cross(a, b, p) >= -1e-12
                                for p in report['footprint_points_m']))

    def test_saved_zone_clearances_hold_after_outward_rounding(self):
        report = json.loads((ARTIFACTS / 'geometry_report.json').read_text())
        (xl, xh), (yl, yh) = report['nav2_padded_bounds_xy_m']
        for values in report['zones'].values():
            (zl, zh), (zyl, zyh) = nav.bounds(values['points_m'])
            distances = values['clearances_from_padded_bbox_m']
            self.assertGreaterEqual(zh - xh + 1e-12, distances['front'])
            self.assertGreaterEqual(xl - zl + 1e-12, distances['rear'])
            self.assertGreaterEqual(yl - zyl + 1e-12, distances['side'])
            self.assertGreaterEqual(zyh - yh + 1e-12, distances['side'])

    def test_saved_caster_disc_is_analytically_inside_cad_hull(self):
        report = json.loads((ARTIFACTS / 'geometry_report.json').read_text())
        caster = report['caster']
        clearance = nav.disc_clearance(report['footprint_points_m'],
                                       caster['nominal_centre_base_xy_m'],
                                       caster['analytic_disc_radius_m'])
        self.assertAlmostEqual(clearance, 0.014923489420429217)
        self.assertFalse(caster['hull_extended_for_caster'])

    def test_saved_yaml_has_only_the_intended_changes_and_serializable_types(self):
        baseline = yaml.safe_load(BASELINE.read_text())
        candidate = yaml.safe_load((ARTIFACTS / 'nav2.yaml').read_text())
        nav.allowed_changes_only(baseline, candidate)
        for name in ('local_costmap', 'global_costmap'):
            params = candidate[name][name]['ros__parameters']
            self.assertIsInstance(params['footprint'], str)
            nav.validate_polygon(json.loads(params['footprint']))
            self.assertIsInstance(params['footprint_padding'], float)

    def test_unchanged_launch_default_does_not_reference_candidate(self):
        source = (nav.ROOT / 'src/roam_e/launch/navigation.launch.py').read_text()
        self.assertNotIn('r19_navigation_candidate', source)
        self.assertIn("'nav2.yaml'", source)
        self.assertIn("'arm_motors',\n            default_value='false'", source)

    def test_preserved_description_has_matching_base_and_camera_axes(self):
        package = nav.ROOT / 'mechanical/ros_r19_camera_forward_candidate_20261005/src/roam_e_description'
        urdf = package / 'urdf/roam_e.urdf'
        if not urdf.is_file():
            self.skipTest('Separately preserved CAD description bundle is required')
        tree = ET.parse(urdf).getroot()
        edges = {joint.find('child').get('link'): joint for joint in tree.findall('joint')}
        def xyz(name):
            return [float(v) for v in edges[name].find('origin').get('xyz').split()]
        def rpy(name):
            return [float(v) for v in edges[name].find('origin').get('rpy').split()]
        self.assertEqual(edges['base_link'].find('parent').get('link'), 'base_footprint')
        self.assertEqual(xyz('base_link')[:2], [0, 0])
        self.assertAlmostEqual(xyz('base_link')[2], 0.033875)
        self.assertTrue(all(abs(v) < 1e-6 for v in rpy('base_link') + rpy('camera_link')))
        self.assertEqual(edges['camera_link'].find('parent').get('link'), 'base_link')
        self.assertAlmostEqual(xyz('left_wheel_link')[1], 0.17363 / 2)
        self.assertAlmostEqual(xyz('right_wheel_link')[1], -0.17363 / 2)
        self.assertEqual(xyz('left_wheel_link')[0], 0)
        self.assertEqual(xyz('right_wheel_link')[0], 0)


if __name__ == '__main__':
    unittest.main()
