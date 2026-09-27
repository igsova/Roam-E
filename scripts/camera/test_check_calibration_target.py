import unittest
from unittest.mock import patch
from types import SimpleNamespace

import cv2
import numpy as np

from check_calibration_target import detect, image_to_gray, object_points


class TargetTests(unittest.TestCase):
    def test_classic_fallback_and_subpixel_refinement(self):
        target = np.full((400, 400), 255, dtype=np.uint8)
        for row in range(8):
            for col in range(8):
                target[40+row*40:80+row*40, 40+col*40:80+col*40] = 255 * ((row + col) % 2)
        with patch('check_calibration_target.cv2.findChessboardCornersSB', return_value=(False, None)), \
                patch('check_calibration_target.cv2.cornerSubPix', wraps=cv2.cornerSubPix) as refine:
            corners = detect(target, (7, 7))
        self.assertEqual(len(corners), 49)
        self.assertTrue(np.isfinite(corners).all())
        refine.assert_called_once()

    def test_fallback_rejects_partial_or_nonfinite_grids(self):
        image = np.zeros((400, 400), dtype=np.uint8)
        for corners in (np.zeros((48, 1, 2), np.float32),
                        np.full((49, 1, 2), np.nan, np.float32)):
            with patch('check_calibration_target.cv2.findChessboardCornersSB', return_value=(False, None)), \
                    patch('check_calibration_target.cv2.findChessboardCorners', return_value=(True, corners)), \
                    patch('check_calibration_target.cv2.cornerSubPix') as refine:
                self.assertIsNone(detect(image, (7, 7)))
                refine.assert_not_called()

    def test_sb_success_skips_classic_fallback(self):
        corners = np.zeros((49, 1, 2), np.float32)
        with patch('check_calibration_target.cv2.findChessboardCornersSB', return_value=(True, corners)), \
                patch('check_calibration_target.cv2.findChessboardCorners') as classic:
            self.assertIs(detect(np.zeros((400, 400), np.uint8), (7, 7)), corners)
            classic.assert_not_called()

    def test_measured_grid(self):
        points = object_points(7, 7, 0.035)
        self.assertEqual(points.shape, (49, 3))
        np.testing.assert_allclose(points[-1], [0.210, 0.210, 0], atol=1e-7)
        np.testing.assert_allclose(points[1], [0.035, 0, 0], atol=1e-7)

    def test_positive_and_negative_detection(self):
        cv2.setNumThreads(1)
        target = np.full((400, 400), 255, dtype=np.uint8)
        for row in range(8):
            for col in range(8):
                target[40+row*40:80+row*40, 40+col*40:80+col*40] = 255 * ((row + col) % 2)
        self.assertEqual(len(detect(target, (7, 7))), 49)
        matrix = cv2.getRotationMatrix2D((200, 200), 12, 0.8)
        rotated = cv2.warpAffine(target, matrix, (400, 400), borderValue=255)
        self.assertEqual(len(detect(rotated, (7, 7))), 49)
        self.assertIsNone(detect(np.full((400, 400), 127, dtype=np.uint8), (7, 7)))

    def test_rgb_row_padding(self):
        msg = SimpleNamespace(encoding='rgb8', width=1, height=2, step=4,
                              data=bytes([255, 0, 0, 0, 0, 255, 0, 0]))
        gray = image_to_gray(msg)
        self.assertEqual(gray.shape, (2, 1))
        self.assertGreater(int(gray[1, 0]), int(gray[0, 0]))

    def test_reject_invalid_images(self):
        for encoding, data, step in [('16UC1', bytes(6), 3), ('rgb8', bytes(3), 3), ('rgb8', bytes(6), 2)]:
            with self.subTest(encoding=encoding, length=len(data), step=step):
                with self.assertRaises(ValueError):
                    image_to_gray(SimpleNamespace(encoding=encoding, width=1, height=2, step=step, data=data))


if __name__ == '__main__':
    unittest.main()
