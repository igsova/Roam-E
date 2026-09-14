import cv2
import numpy as np
import pytest
from roam_e.mjpeg_camera_node import jpeg_bytes, decode_rgb


def test_native_jpeg_passthrough_and_decode():
    frame = np.zeros((48, 64, 3), dtype=np.uint8)
    frame[:, :, 2] = 255
    ok, encoded = cv2.imencode('.jpg', frame)
    assert ok
    data = jpeg_bytes(encoded)
    assert data == encoded.tobytes()
    rgb = decode_rgb(data)
    assert rgb.shape == (48, 64, 3)
    assert rgb[0, 0, 0] > 240
    assert rgb[0, 0, 2] < 10


def test_reject_uncompressed_input():
    with pytest.raises(ValueError):
        jpeg_bytes(np.zeros((10, 10, 3), dtype=np.uint8))


def test_reject_invalid_jpeg():
    with pytest.raises(ValueError):
        decode_rgb(b'not a jpeg')


def test_reject_truncated_jpeg():
    with pytest.raises(ValueError):
        jpeg_bytes(np.frombuffer(b'\xff\xd8unfinished', dtype=np.uint8))
