import cv2
import numpy as np
import pytest
import threading
from types import SimpleNamespace
from builtin_interfaces.msg import Time
from roam_e import mjpeg_camera_node
from roam_e.mjpeg_camera_node import MjpegCamera
from roam_e.rgb_calibration import RgbCalibration
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


class FakePublisher:
    def __init__(self, subscribers=0):
        self.messages = []
        self.subscribers = subscribers

    def publish(self, message):
        self.messages.append(message)

    def get_subscription_count(self):
        return self.subscribers


def frame_node(monkeypatch, subscribers=1):
    monkeypatch.setattr(mjpeg_camera_node.time, 'monotonic', lambda: 10.0)
    ok, jpeg = cv2.imencode('.jpg', np.zeros((48, 64, 3), np.uint8))
    assert ok
    k = (60., 0., 32., 0., 60., 24., 0., 0., 1.)
    calibration = RgbCalibration(64, 48, k, (0.,)*5,
                                 (1., 0., 0., 0., 1., 0., 0., 0., 1.),
                                 (60., 0., 32., 0., 0., 60., 24., 0., 0., 0., 1., 0.))
    return SimpleNamespace(
        error=None, lock=threading.Lock(),
        latest=(Time(sec=123, nanosec=456), jpeg.tobytes()),
        preview_fps=5., decode_fps=2., last_preview=0., last_decode=0.,
        width=64, height=48, frame_id='test_rgb_optical', invalid=0,
        compressed=FakePublisher(), raw=FakePublisher(subscribers),
        info=FakePublisher(), calibration=calibration,
    )


@pytest.mark.parametrize('preview_due,raw_due,expected', [
    (True, True, (1, 1, 1)), (True, False, (1, 0, 1)),
    (False, True, (0, 1, 1)), (False, False, (0, 0, 0)),
])
def test_camera_info_matches_union_of_image_schedules(monkeypatch, preview_due, raw_due, expected):
    node = frame_node(monkeypatch)
    node.last_preview = 0. if preview_due else 10.
    node.last_decode = 0. if raw_due else 10.
    source_stamp = node.latest[0]
    MjpegCamera.publish_latest(node)
    assert tuple(len(p.messages) for p in (node.compressed, node.raw, node.info)) == expected
    for pub in (node.compressed, node.raw, node.info):
        for msg in pub.messages:
            assert msg.header.stamp == source_stamp
            assert msg.header.frame_id == node.frame_id
    assert node.latest is None


def test_no_subscribers_does_not_decode(monkeypatch):
    node = frame_node(monkeypatch, subscribers=0)
    def forbidden_decode(data):
        pytest.fail('Native JPEG preview must not decode without raw subscribers')
    monkeypatch.setattr(mjpeg_camera_node, 'decode_rgb', forbidden_decode)
    MjpegCamera.publish_latest(node)
    assert len(node.compressed.messages) == len(node.info.messages) == 1
    assert not node.raw.messages


def test_bad_decode_still_sends_metadata_for_published_preview(monkeypatch):
    node = frame_node(monkeypatch)
    def bad_decode(data):
        raise ValueError('bad frame')
    monkeypatch.setattr(mjpeg_camera_node, 'decode_rgb', bad_decode)
    MjpegCamera.publish_latest(node)
    assert node.invalid == 1
    assert len(node.compressed.messages) == len(node.info.messages) == 1
    assert not node.raw.messages


def test_uncalibrated_preview_and_decoded_dimensions(monkeypatch):
    node = frame_node(monkeypatch)
    node.info = node.calibration = None
    MjpegCamera.publish_latest(node)
    assert len(node.compressed.messages) == len(node.raw.messages) == 1
    node = frame_node(monkeypatch)
    node.width = 640
    with pytest.raises(RuntimeError, match='dimensions'):
        MjpegCamera.publish_latest(node)
    assert not node.raw.messages


def test_missing_frame_does_not_repeat_image_or_metadata(monkeypatch):
    node = frame_node(monkeypatch)
    node.latest = None
    MjpegCamera.publish_latest(node)
    assert not any(pub.messages for pub in (node.compressed, node.raw, node.info))
