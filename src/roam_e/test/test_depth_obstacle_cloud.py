import numpy as np

from roam_e.depth_obstacle_cloud_node import depth_to_points

K = [570.34, 0.0, 319.5, 0.0, 570.34, 239.5, 0.0, 0.0, 1.0]


def test_flat_wall_projects_to_constant_depth():
    depth = np.full((480, 640), 1500, dtype=np.uint16)
    points = depth_to_points(depth, K, 4, 0.45, 3.0)
    assert points.shape == (120 * 160, 3)
    assert np.allclose(points[:, 2], 1.5)
    centre = points[np.argmin(np.abs(points[:, 0]) + np.abs(points[:, 1]))]
    assert abs(centre[0]) < 0.02 and abs(centre[1]) < 0.02


def test_invalid_and_out_of_range_pixels_dropped():
    depth = np.zeros((480, 640), dtype=np.uint16)
    depth[0:4, 0:4] = 1000    # one sampled valid pixel at (0, 0)
    depth[8, 8] = 300         # too close
    depth[12, 12] = 4000      # too far
    points = depth_to_points(depth, K, 4, 0.45, 3.0)
    assert points.shape == (1, 3)
    x, y, z = points[0]
    assert np.isclose(z, 1.0)
    assert np.isclose(x, (0 - 319.5) / 570.34) and np.isclose(y, (0 - 239.5) / 570.34)


def test_optical_axes_left_is_negative_x():
    depth = np.zeros((480, 640), dtype=np.uint16)
    depth[240, 100] = 2000    # left part of the image
    points = depth_to_points(depth, K, 4, 0.45, 3.0)
    assert points[0, 0] < 0   # optical x points right
