"""Offline numerical tests. No ROS runtime or camera required."""
from types import SimpleNamespace as NS
import unittest
import numpy as np
from firmware_rgbd_math import (camera_model, cloud_array, depth_array, nearest_depth,
                               rectification_maps, rectify_pair)


def info():
    return NS(width=640, height=480, header=NS(frame_id='camera_rgb_optical_frame'),
              distortion_model='plumb_bob', d=[0.] * 5,
              k=[500., 0., 320., 0., 500., 240., 0., 0., 1.], r=np.eye(3).ravel(),
              binning_x=0, binning_y=0, roi=NS(x_offset=0, y_offset=0, width=0, height=0))


class GeometryTest(unittest.TestCase):
    def test_cloud_units_axes_rgb_and_invalid_mask(self):
        rgb = np.array([[[255, 0, 0], [0, 255, 0]], [[0, 0, 255], [255, 255, 255]]], np.uint8)
        depth = np.array([[1000, 2000], [0, 6000]], np.uint16)
        k = np.diag([2., 2., 1.])
        points = cloud_array(rgb, depth, k, stride=1)
        self.assertEqual(points.dtype.itemsize, 16)
        np.testing.assert_allclose(points['x'], [0., 1.])
        np.testing.assert_allclose(points['z'], [1., 2.])
        self.assertEqual(list(points['rgb']), [0xff0000, 0x00ff00])

    def test_decimation_retains_original_pixel_coordinates(self):
        p = cloud_array(np.zeros((5, 5, 3), np.uint8), np.full((5, 5), 1000, np.uint16),
                        np.eye(3), stride=4)
        np.testing.assert_allclose(p['x'], [0., 4., 0., 4.])
        np.testing.assert_allclose(p['y'], [0., 0., 4., 4.])

    def test_nearest_pair_is_bounded(self):
        self.assertEqual(nearest_depth(100, [(80, 'a'), (108, 'b')], 10), (108, 'b'))
        self.assertIsNone(nearest_depth(100, [(80, 'a')], 10))
        self.assertIsNone(nearest_depth(100, [], 10))

    def test_depth_endian_and_padding(self):
        padded = np.full((480, 642), 1234, dtype='>u2')
        msg = NS(encoding='16UC1', width=640, height=480, step=1284, is_bigendian=True,
                 data=padded.tobytes())
        d = depth_array(msg)
        self.assertEqual(d.shape, (480, 640))
        self.assertEqual(int(d[0, 0]), 1234)
        msg.step = 1
        with self.assertRaises(ValueError):
            depth_array(msg)

    def test_metadata_rejects_wrong_frame_resolution_calibration(self):
        good = info()
        camera_model(good)
        for attribute, value in [('width', 320), ('distortion_model', 'equidistant'),
                                  ('k', [0.] * 9), ('binning_x', 2)]:
            bad = info()
            setattr(bad, attribute, value)
            with self.assertRaises(ValueError):
                camera_model(bad)

    def test_joint_identity_rectification_keeps_depth_invalid(self):
        k, d = camera_model(info())
        rgb = np.zeros((480, 640, 3), np.uint8)
        rgb[:, 320:, 0] = 255
        depth = np.full((480, 640), 1500, np.uint16)
        depth[:, :320] = 0
        r, z = rectify_pair(rgb, depth, rectification_maps(k, d))
        np.testing.assert_array_equal(z, depth)
        np.testing.assert_array_equal(r, rgb)

    def test_depth_rectification_does_not_invent_intermediate_depths(self):
        k, _ = camera_model(info())
        depth = np.full((480, 640), 5000, np.uint16)
        depth[:, :320] = 1000
        rgb = np.zeros((480, 640, 3), np.uint8)
        _, z = rectify_pair(rgb, depth, rectification_maps(k, np.array([.1, -.2, .01, .01, 0.])))
        self.assertTrue(set(np.unique(z)).issubset({0, 1000, 5000}))


if __name__ == '__main__':
    unittest.main()
