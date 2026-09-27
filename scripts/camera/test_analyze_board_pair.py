import unittest
import numpy as np
from analyze_board_pair import plane_depth_mm


class BoardPlaneTest(unittest.TestCase):
    def test_front_parallel_plane(self):
        values = plane_depth_mm([[0, 0], [0.5, -0.2]], np.eye(3), np.zeros(5),
                                np.zeros(3), np.array([0., 0., 1.]))
        np.testing.assert_allclose(values, [1000, 1000])

    def test_tilted_plane_intersection(self):
        values = plane_depth_mm([[0, 0], [1, 0]], np.eye(3), np.zeros(5),
                                np.array([0., np.pi/4, 0.]), np.array([0., 0., 1.]))
        np.testing.assert_allclose(values, [1000, 500])

    def test_parallel_ray_rejected(self):
        with self.assertRaises(ValueError):
            plane_depth_mm([[-1, 0]], np.eye(3), np.zeros(5),
                           np.array([0., np.pi/4, 0.]), np.array([0., 0., 1.]))

    def test_behind_camera_rejected(self):
        with self.assertRaises(ValueError):
            plane_depth_mm([[0, 0]], np.eye(3), np.zeros(5),
                           np.zeros(3), np.array([0., 0., -1.]))


if __name__ == '__main__':
    unittest.main()
