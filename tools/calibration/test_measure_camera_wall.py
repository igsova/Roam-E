"""Synthetic geometry tests for the camera-vs-lidar wall check; no ROS or hardware."""
import math
from pathlib import Path
import sys
import unittest
import numpy as np
sys.path.insert(0, str(Path(__file__).parent))
import measure_camera_wall as m

K = [570.34, 0, 319.5, 0, 570.34, 239.5, 0, 0, 1]
# Optical frame axes in base: z forward=+x, x right=-y, y down=-z.
R_OPT = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], dtype=float)
T_CAM = np.array([0.067, -0.0235, 0.129])


def render_wall(distance, yaw_deg, scale=1.0, cam_rot=R_OPT):
    """Depth image (mm) of a vertical wall n·p = distance in base frame."""
    yaw = math.radians(yaw_deg)
    n = np.array([math.cos(yaw), math.sin(yaw), 0.0])
    h, w = 480, 640
    ys, xs = np.mgrid[0:h, 0:w]
    rays_c = np.stack(((xs - K[2]) / K[0], (ys - K[5]) / K[4], np.ones_like(xs, dtype=float)), -1)
    rays_b = rays_c @ cam_rot.T
    t = (distance - n @ T_CAM) / (rays_b @ n)  # z along optical axis = t
    return (t * 1000 * scale).astype(np.uint16)


def solve(depth, cam_rot=R_OPT):
    pts = m.depth_points(depth, K)
    n_c, d_c, rms, used = m.fit_plane(pts)
    return m.plane_in_base(n_c, d_c, cam_rot, T_CAM)


class WallTest(unittest.TestCase):
    def test_straight_wall(self):
        n_b, d_b = solve(render_wall(1.0, 0))
        out = m.summarize((n_b, d_b), (np.array([1.0, 0, 0]), 1.0), 1.0 - m.FRONT_X_M)
        self.assertAlmostEqual(out['camera_minus_lidar_m'], 0, delta=0.003)
        self.assertAlmostEqual(out['camera_wall_yaw_deg'], 0, delta=0.2)
        self.assertAlmostEqual(out['camera_wall_tilt_deg'], 0, delta=0.2)
        self.assertAlmostEqual(out['lidar_minus_tape_m'], 0, delta=1e-6)

    def test_rotated_wall_and_scale(self):
        n_b, d_b = solve(render_wall(1.2, 8, scale=1.02))
        out = m.summarize((n_b, d_b), (np.array([math.cos(math.radians(8)), math.sin(math.radians(8)), 0]), 1.2), None)
        self.assertAlmostEqual(out['camera_minus_lidar_yaw_deg'], 0, delta=0.5)
        self.assertGreater(out['camera_minus_lidar_m'], 0.015)  # 2 % long depth shows up

    def test_camera_pitched_down_detected(self):
        pitch = math.radians(3)
        ry = np.array([[math.cos(pitch), 0, math.sin(pitch)], [0, 1, 0], [-math.sin(pitch), 0, math.cos(pitch)]])
        true_rot = ry @ R_OPT          # real camera pitched 3 deg down
        depth = render_wall(1.0, 0, cam_rot=true_rot)
        n_b, d_b = solve(depth, cam_rot=R_OPT)  # analysed with the nominal mount
        out = m.summarize((n_b, d_b), (np.array([1.0, 0, 0]), 1.0), None)
        self.assertAlmostEqual(abs(out['camera_wall_tilt_deg']), 3, delta=0.3)

    def test_line_fit(self):
        y = np.linspace(-0.3, 0.3, 50)
        pts = np.column_stack((np.full_like(y, 0.9) + 0.002 * np.sin(y * 40), y))
        n, d, rms, used = m.fit_line(pts)
        self.assertAlmostEqual(d, 0.9, delta=0.003)
        self.assertAlmostEqual(abs(n[0]), 1, delta=1e-3)

    def test_quaternion_identity_and_yaw(self):
        self.assertTrue(np.allclose(m.quaternion_matrix(0, 0, 0, 1), np.eye(3)))
        r = m.quaternion_matrix(0, 0, math.sin(math.pi / 4), math.cos(math.pi / 4))
        self.assertTrue(np.allclose(r @ [1, 0, 0], [0, 1, 0]))


if __name__ == '__main__':
    unittest.main()
