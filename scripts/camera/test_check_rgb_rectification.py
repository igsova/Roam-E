import unittest

import numpy as np
from sensor_msgs.msg import Image

from check_rgb_rectification import rgb_array, stamp_key


class RectificationHelpersTest(unittest.TestCase):
    def frame(self):
        message = Image()
        message.width, message.height = 640, 480
        message.encoding, message.step = 'rgb8', 1924
        rows = np.zeros((480, 1924), np.uint8)
        rows[:, 0:1920:3] = 37
        rows[:, 1920:] = 255
        message.data = rows.tobytes()
        return message

    def test_stride_padding_is_not_image_data(self):
        array = rgb_array(self.frame())
        self.assertEqual(array.shape, (480, 640, 3))
        self.assertTrue(np.all(array[:, :, 0] == 37))
        self.assertTrue(np.all(array[:, :, 1:] == 0))

    def test_reject_wrong_resolution_encoding_stride_or_size(self):
        for field, value in [('width', 320), ('height', 240), ('encoding', 'bgr8'),
                             ('step', 100), ('data', b'')]:
            with self.subTest(field=field):
                message = self.frame()
                setattr(message, field, value)
                with self.assertRaises(ValueError):
                    rgb_array(message)

    def test_stamp_key_preserves_nanoseconds(self):
        message = self.frame()
        message.header.stamp.sec = 1789733332
        message.header.stamp.nanosec = 987654321
        self.assertEqual(stamp_key(message), (1789733332, 987654321))


if __name__ == '__main__':
    unittest.main()
