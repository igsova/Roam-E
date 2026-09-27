"""Validated, resolution-specific monocular RGB calibration; no device access."""
from dataclasses import dataclass
import math
from numbers import Real
from pathlib import Path

import yaml


@dataclass(frozen=True)
class RgbCalibration:
    width: int
    height: int
    k: tuple
    d: tuple
    r: tuple
    p: tuple
    distortion_model: str = 'plumb_bob'


def _matrix(data, key, rows, cols):
    value = data.get(key)
    if not isinstance(value, dict) or (value.get('rows'), value.get('cols')) != (rows, cols):
        raise ValueError(f'{key} must be a {rows}x{cols} matrix')
    values = value.get('data')
    if not isinstance(values, list) or len(values) != rows * cols:
        raise ValueError(f'{key} has an invalid data length')
    if any(isinstance(v, bool) or not isinstance(v, Real) or not math.isfinite(v) for v in values):
        raise ValueError(f'{key} must contain finite numbers')
    return tuple(float(v) for v in values)


def load_rgb_calibration(path, expected_size):
    """Fail before USB is opened if an explicitly requested file is invalid."""
    source = Path(path).expanduser()
    try:
        if source.stat().st_size > 65536:
            raise ValueError('Calibration YAML exceeds 64 KiB')
        data = yaml.safe_load(source.read_text(encoding='utf-8'))
    except (OSError, yaml.YAMLError) as error:
        raise ValueError(f'Cannot load RGB calibration {source}: {error}') from error
    if not isinstance(data, dict):
        raise ValueError('Calibration YAML must contain a mapping')
    size = (data.get('image_width'), data.get('image_height'))
    if any(type(v) is not int or v <= 0 for v in size) or size != tuple(expected_size):
        raise ValueError(f'Calibration size {size} does not match capture size {expected_size}')
    if data.get('distortion_model') != 'plumb_bob':
        raise ValueError('RGB calibration must use the plumb_bob distortion model')
    k = _matrix(data, 'camera_matrix', 3, 3)
    d = _matrix(data, 'distortion_coefficients', 1, 5)
    r = _matrix(data, 'rectification_matrix', 3, 3)
    p = _matrix(data, 'projection_matrix', 3, 4)
    if k[0] <= 0 or k[4] <= 0 or p[0] <= 0 or p[5] <= 0:
        raise ValueError('Calibration focal lengths must be positive')
    if any(abs(k[i]) > 1e-9 for i in (1, 3, 6, 7)) or abs(k[8]-1) > 1e-9:
        raise ValueError('Unsupported intrinsic matrix structure')
    # This file describes one RGB camera, not a calibrated RGB-depth stereo pair.
    identity = (1., 0., 0., 0., 1., 0., 0., 0., 1.)
    if any(abs(a-b) > 1e-9 for a, b in zip(r, identity)):
        raise ValueError('Monocular RGB rectification requires identity R')
    if any(abs(p[i]) > 1e-9 for i in (1, 3, 4, 7, 8, 9, 11)) or abs(p[10]-1) > 1e-9:
        raise ValueError('Unsupported monocular projection matrix structure')
    return RgbCalibration(*size, k, d, r, p)


def camera_info_for_frame(calibration, stamp, frame_id):
    """Use exactly the source image's stamp and optical frame, never wall time."""
    from sensor_msgs.msg import CameraInfo
    msg = CameraInfo()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.width, msg.height = calibration.width, calibration.height
    msg.distortion_model = calibration.distortion_model
    msg.k, msg.d = list(calibration.k), list(calibration.d)
    msg.r, msg.p = list(calibration.r), list(calibration.p)
    return msg
