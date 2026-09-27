from pathlib import Path

import pytest
import yaml
from builtin_interfaces.msg import Time

from roam_e.rgb_calibration import camera_info_for_frame, load_rgb_calibration


CONFIG = Path(__file__).resolve().parents[1] / 'config' / 'astra_rgb_640x480.yaml'


def test_measured_calibration_and_exact_frame_metadata():
    calibration = load_rgb_calibration(CONFIG, (640, 480))
    stamp = Time(sec=123, nanosec=456789)
    info = camera_info_for_frame(calibration, stamp, 'camera_rgb_optical_frame')
    assert (info.width, info.height) == (640, 480)
    assert info.header.stamp == stamp
    assert info.header.frame_id == 'camera_rgb_optical_frame'
    assert info.distortion_model == 'plumb_bob'
    assert tuple(info.k) == calibration.k
    assert tuple(info.d) == calibration.d
    assert tuple(info.r) == calibration.r
    assert tuple(info.p) == calibration.p
    assert calibration.k[0] == pytest.approx(580.3540663780394)
    assert calibration.d[-1] == 0.0
    assert info.binning_x == info.binning_y == 0
    # Do not share mutable matrices across messages.
    info.k[0] = 1.0
    assert camera_info_for_frame(calibration, stamp, 'rgb').k[0] == calibration.k[0]


@pytest.mark.parametrize('size', [(320, 240), (480, 640), (640, 481)])
def test_resolution_mismatch_is_rejected(size):
    with pytest.raises(ValueError, match='does not match'):
        load_rgb_calibration(CONFIG, size)


@pytest.mark.parametrize('field,value', [
    ('image_width', True),
    ('image_height', -1),
    ('distortion_model', 'equidistant'),
    ('camera_matrix', {'rows': 2, 'cols': 3, 'data': [1] * 6}),
    ('distortion_coefficients', {'rows': 1, 'cols': 5, 'data': [0] * 4}),
    ('rectification_matrix', None),
])
def test_invalid_schema_is_rejected(tmp_path, field, value):
    data = yaml.safe_load(CONFIG.read_text())
    data[field] = value
    path = tmp_path / 'invalid.yaml'
    path.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError):
        load_rgb_calibration(path, (640, 480))


@pytest.mark.parametrize('field,index,value', [
    ('camera_matrix', 0, 0),
    ('camera_matrix', 1, 1),
    ('camera_matrix', 8, 0),
    ('camera_matrix', 0, float('inf')),
    ('distortion_coefficients', 0, float('nan')),
    ('distortion_coefficients', 0, True),
    ('distortion_coefficients', 0, '0.1'),
    ('rectification_matrix', 0, 0.9),
    ('projection_matrix', 5, -1),
    ('projection_matrix', 3, 0.1),
    ('projection_matrix', 10, 0),
])
def test_invalid_matrix_values_are_rejected(tmp_path, field, index, value):
    data = yaml.safe_load(CONFIG.read_text())
    data[field]['data'][index] = value
    path = tmp_path / 'invalid.yaml'
    path.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError):
        load_rgb_calibration(path, (640, 480))


@pytest.mark.parametrize('content', ['', '[]', '[invalid', 'x' * 65537])
def test_unreadable_or_invalid_yaml(tmp_path, content):
    path = tmp_path / 'bad.yaml'
    path.write_text(content)
    with pytest.raises(ValueError):
        load_rgb_calibration(path, (640, 480))
    with pytest.raises(ValueError, match='Cannot load'):
        load_rgb_calibration(tmp_path / 'missing.yaml', (640, 480))
