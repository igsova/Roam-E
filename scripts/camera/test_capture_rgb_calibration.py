import unittest

import cv2
import numpy as np

from capture_rgb_calibration import board_quality, grid_distance
from check_calibration_target import detect


class CaptureTests(unittest.TestCase):
    def test_grid_distance_ignores_symmetry(self):
        grid = np.mgrid[0:7, 0:7].T.astype(np.float32) * 30
        for quarter_turns in range(4):
            rotated = np.rot90(grid, quarter_turns, axes=(0, 1))
            self.assertEqual(grid_distance(grid, rotated), 0)
            self.assertEqual(grid_distance(grid, rotated[::-1]), 0)
        self.assertAlmostEqual(grid_distance(grid, grid + [20, 0]), 20)

    def test_sharp_and_blurred_target(self):
        cv2.setNumThreads(1)
        target = np.full((400, 400), 255, dtype=np.uint8)
        for row in range(8):
            for col in range(8):
                target[40+row*40:80+row*40, 40+col*40:80+col*40] = 255 * ((row + col) % 2)
        corners = detect(target, (7, 7))
        self.assertIsNotNone(corners)
        reason, details = board_quality(target, corners)
        self.assertIsNone(reason)
        self.assertEqual(len(details['edge_transition_px']), 2)
        blurred = cv2.GaussianBlur(target, (21, 21), 4)
        reason, _ = board_quality(blurred, corners)
        self.assertEqual(reason, 'blurred_board')

    def test_small_or_clipped_board(self):
        image = np.zeros((480, 640), dtype=np.uint8)
        grid = np.mgrid[0:7, 0:7].T.reshape(-1, 1, 2).astype(np.float32)
        reason, _ = board_quality(image, grid * 5 + 40)
        self.assertEqual(reason, 'board_too_small_or_steep')
        reason, _ = board_quality(image, grid * 30 + 1)
        self.assertEqual(reason, 'corners_too_close_to_edge')


if __name__ == '__main__':
    unittest.main()
