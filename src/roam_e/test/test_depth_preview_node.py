import cv2
import numpy as np
from sensor_msgs.msg import Image

from roam_e.depth_preview_node import colorize_depth


def make_depth_image(values):
    array = np.asarray(values, dtype=np.uint16)
    image = Image()
    image.height, image.width = array.shape
    image.encoding = '16UC1'
    image.is_bigendian = False
    image.step = image.width * 2
    image.data = array.tobytes()
    return image


def test_colorize_depth_downsamples_and_masks_invalid_values():
    image = make_depth_image([
        [0, 1000, 2000, 3000],
        [1000, 2000, 3000, 4000],
        [500, 1500, 5000, 6000],
        [1000, 2000, 3000, 4000],
    ])

    preview = colorize_depth(image, 200, 5000, 2)

    assert preview.shape == (2, 2, 3)
    assert preview.dtype == np.uint8
    assert np.array_equal(preview[0, 0], np.zeros(3, dtype=np.uint8))
    assert not np.array_equal(preview[1, 0], np.zeros(3, dtype=np.uint8))


def test_colorized_preview_is_jpeg_encodable():
    image = make_depth_image(np.full((8, 8), 1500, dtype=np.uint16))
    preview = colorize_depth(image, 200, 5000, 2)

    encoded_ok, encoded = cv2.imencode('.jpg', preview)

    assert encoded_ok
    assert encoded.size > 0
