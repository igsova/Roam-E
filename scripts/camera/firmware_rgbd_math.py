"""Geometry for the experimental firmware-registered RGB-D baseline.

ASSUMPTION, NOT A CALIBRATION RESULT: firmware depth pixels are coindexed with
raw UVC RGB pixels. Use measured RGB K/D to rectify BOTH images onto one grid.
Do not apply the offline RGB--IR candidate to already-registered depth.
"""
import cv2
import numpy as np


def stamp_ns(header):
    return header.stamp.sec * 1_000_000_000 + header.stamp.nanosec


def camera_model(info):
    if info.width != 640 or info.height != 480:
        raise ValueError('This baseline requires 640x480 camera metadata')
    if info.header.frame_id != 'camera_rgb_optical_frame':
        raise ValueError('Expected camera_rgb_optical_frame')
    if info.distortion_model != 'plumb_bob' or len(info.d) != 5:
        raise ValueError('Expected five-coefficient plumb_bob RGB calibration')
    k = np.asarray(info.k, dtype=np.float64).reshape(3, 3)
    d = np.asarray(info.d, dtype=np.float64)
    if not np.isfinite(k).all() or not np.isfinite(d).all():
        raise ValueError('Non-finite camera intrinsics')
    if k[0, 0] <= 0 or k[1, 1] <= 0 or not np.allclose(k[2], [0, 0, 1]):
        raise ValueError('Invalid pinhole matrix')
    if not np.allclose(np.asarray(info.r).reshape(3, 3), np.eye(3)):
        raise ValueError('Non-identity input rectification is not supported')
    if info.binning_x not in (0, 1) or info.binning_y not in (0, 1):
        raise ValueError('Binned input is not supported')
    if any((info.roi.x_offset, info.roi.y_offset, info.roi.width, info.roi.height)):
        raise ValueError('Cropped input is not supported')
    return k, d


def depth_array(msg):
    if msg.encoding != '16UC1' or (msg.width, msg.height) != (640, 480):
        raise ValueError('Expected VGA 16UC1 millimetre depth')
    if msg.step < msg.width * 2 or len(msg.data) != msg.step * msg.height:
        raise ValueError('Invalid depth buffer layout')
    dtype = np.dtype('>u2' if msg.is_bigendian else '<u2')
    return np.ndarray((msg.height, msg.width), dtype=dtype,
                      buffer=msg.data, strides=(msg.step, 2)).astype(np.uint16)


def rectification_maps(k, d, size=(640, 480)):
    return cv2.initUndistortRectifyMap(k, d, np.eye(3), k, size, cv2.CV_32FC1)


def rectify_pair(rgb, depth, maps):
    if rgb.shape != (*depth.shape, 3):
        raise ValueError('RGB and depth dimensions differ')
    # Interpolating depth mixes surfaces/invalid zeros. Keep nearest samples.
    return (cv2.remap(rgb, *maps, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT),
            cv2.remap(depth, *maps, cv2.INTER_NEAREST, borderMode=cv2.BORDER_CONSTANT))


def cloud_array(rgb, depth_mm, k, stride=4, min_depth_m=0.3, max_depth_m=5.0):
    """Return little-endian x/y/z float32 + PCL packed RGB bytes; optical axes.

    RGB is uint8 R,G,B. Zero and out-of-range depth never become 3-D points.
    The output is an unorganized decimated preview, not the tracking input.
    """
    if stride < 1 or not 0 < min_depth_m < max_depth_m:
        raise ValueError('Invalid point-cloud sampling/range limits')
    if rgb.shape != (*depth_mm.shape, 3):
        raise ValueError('RGB and depth dimensions differ')
    yy, xx = np.mgrid[0:depth_mm.shape[0]:stride, 0:depth_mm.shape[1]:stride]
    z = depth_mm[::stride, ::stride].astype(np.float32) * 0.001
    valid = np.isfinite(z) & (z >= min_depth_m) & (z <= max_depth_m)
    z = z[valid]
    color = rgb[::stride, ::stride][valid].astype(np.uint32)
    result = np.empty(len(z), dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('rgb', '<u4')])
    result['x'] = (xx[valid] - k[0, 2]) * z / k[0, 0]
    result['y'] = (yy[valid] - k[1, 2]) * z / k[1, 1]
    result['z'] = z
    result['rgb'] = (color[:, 0] << 16) | (color[:, 1] << 8) | color[:, 2]
    return result


def nearest_depth(rgb_stamp, depths, max_delta_ns):
    """Inputs are (stamp_ns, message) pairs. Matching is headers, not exposure."""
    if not depths:
        return None
    stamp, msg = min(depths, key=lambda pair: abs(pair[0] - rgb_stamp))
    return (stamp, msg) if abs(stamp - rgb_stamp) <= max_delta_ns else None
