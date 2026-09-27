import unittest

import numpy as np
from sensor_msgs.msg import Image

from inspect_rgbd_pair import depth_edges, image_array, stamp_ns


class PairHelpersTest(unittest.TestCase):
    def test_invalid_depth_not_treated_as_metric_jump(self):
        depth = np.array([[0, 1000, 1005, 1200]], np.uint16)
        jumps, invalid = depth_edges(depth)
        self.assertEqual(jumps.tolist(), [[False, False, True, True]])
        self.assertEqual(invalid.tolist(), [[True, True, False, False]])

    def test_no_uint16_underflow_or_wrap(self):
        for depth in (np.array([[65000, 10]], np.uint16),
                      np.array([[10], [65000]], np.uint16)):
            jumps, invalid = depth_edges(depth)
            self.assertTrue(jumps.all())
            self.assertFalse(invalid.any())

    def test_depth_endian_and_padding(self):
        for big in (False, True):
            rows = np.full((480, 642), 1234, dtype='>u2' if big else '<u2')
            rows[:, 640:] = 65535
            msg = Image(width=640, height=480, encoding='16UC1', step=1284,
                        is_bigendian=int(big), data=rows.tobytes())
            decoded = image_array(msg)
            self.assertEqual(decoded.shape, (480, 640))
            self.assertTrue(np.all(decoded == 1234))

    def test_rgb_padding_and_validation(self):
        msg = Image(width=640, height=480, encoding='rgb8', step=1924,
                    data=bytes(1924*480))
        self.assertEqual(image_array(msg).shape, (480, 640, 3))
        for key, value in [('width', 0), ('encoding', 'mono8'), ('step', 1), ('data', b'')]:
            frame = Image(width=640, height=480, encoding='rgb8', step=1920, data=bytes(1920*480))
            setattr(frame, key, value)
            with self.assertRaises(ValueError):
                image_array(frame)

    def test_timestamp_precision(self):
        frame = Image()
        frame.header.stamp.sec, frame.header.stamp.nanosec = 1789734000, 123456789
        self.assertEqual(stamp_ns(frame), 1789734000123456789)


if __name__ == '__main__':
    unittest.main()
