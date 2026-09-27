#!/usr/bin/env python3
"""Offline board-plane consistency check; not a stereo calibration solution."""
import argparse
import json
from pathlib import Path
import cv2
import numpy as np
from check_calibration_target import detect, object_points


def plane_depth_mm(pixels, k, d, rvec, tvec):
    """Intersect undistorted RGB rays with a board plane; no depth extrinsics."""
    normal = cv2.Rodrigues(rvec)[0][:, 2]
    normalized = cv2.undistortPoints(np.asarray(pixels, dtype=np.float64).reshape(-1, 1, 2), k, d)
    rays = np.column_stack([normalized.reshape(-1, 2), np.ones(len(normalized))])
    denominators = rays @ normal
    if np.any(np.abs(denominators) < 1e-9):
        raise ValueError('RGB ray is parallel to the board plane')
    z_mm = float(normal @ tvec.reshape(3)) / denominators * 1000.
    if not np.isfinite(z_mm).all() or np.any(z_mm <= 0):
        raise ValueError('Invalid or behind-camera plane intersection')
    return z_mm


def analyze(directory):
    metadata = json.loads((directory / 'pair.json').read_text())
    rgb = cv2.imread(str(directory / 'rgb_raw.png'))
    depth = cv2.imread(str(directory / 'depth_mm.png'), cv2.IMREAD_UNCHANGED)
    if rgb is None or depth is None or rgb.shape != (480, 640, 3) or depth.shape != (480, 640):
        raise ValueError('Expected the saved 640x480 RGB/depth pair')
    if depth.dtype != np.uint16:
        raise ValueError('Expected native uint16 millimetre depth')
    corners = detect(cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY), (7, 7))
    if corners is None:
        raise ValueError('Full 7x7 internal-corner grid was not detected')
    k = np.array(metadata['rgb_info']['k']).reshape(3, 3)
    d = np.array(metadata['rgb_info']['d'])
    obj = object_points(7, 7, 0.035)
    ok, rvec, tvec = cv2.solvePnP(obj, corners, k, d, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        raise ValueError('RGB board pose could not be estimated')
    rotation = cv2.Rodrigues(rvec)[0]
    predicted = cv2.projectPoints(obj, rvec, tvec, k, d)[0]
    center = rotation @ np.array([0.105, 0.105, 0.0]) + tvec.reshape(3)
    normal = rotation[:, 2]
    # Erode the detected inner-corner hull to avoid outer edges/disocclusions.
    roi = np.zeros(depth.shape, np.uint8)
    cv2.fillConvexPoly(roi, np.rint(cv2.convexHull(corners)).astype(np.int32), 255)
    roi = cv2.erode(roi, np.ones((7, 7), np.uint8)) > 0
    ys, xs = np.nonzero(roi & (depth > 0))
    if len(xs) < 500:
        raise ValueError('Insufficient valid board-interior depth')
    # This intentionally hypothesizes same-pixel registration for a consistency
    # test. It must not be interpreted as an independent depth calibration.
    rgb_plane_z = plane_depth_mm(np.column_stack([xs, ys]), k, d, rvec, tvec)
    observed = depth[ys, xs].astype(float)
    residual = observed - rgb_plane_z
    median = float(np.median(observed))
    mad = float(np.median(np.abs(observed-median)))
    result = {
        'detected_corners': 49, 'square_size_m': 0.035,
        'rgb_pose_reprojection_rms_px': float(np.sqrt(np.mean(np.sum((predicted-corners)**2, axis=2)))),
        'rgb_board_center_xyz_m': center.tolist(),
        'rgb_board_normal': normal.tolist(),
        'rgb_board_normal_tilt_deg': float(np.degrees(np.arccos(np.clip(abs(normal[2]), 0, 1)))),
        'internal_corner_bounds_px': [corners.reshape(-1, 2).min(axis=0).tolist(),
                                      corners.reshape(-1, 2).max(axis=0).tolist()],
        'depth_roi_valid_fraction': float(len(xs)/np.count_nonzero(roi)),
        'depth_roi_pixels': len(xs), 'depth_roi_median_mm': median, 'depth_roi_mad_mm': mad,
        'depth_roi_p05_p95_mm': np.percentile(observed, [5, 95]).tolist(),
        'depth_minus_rgb_plane_median_mm': float(np.median(residual)),
        'depth_minus_rgb_plane_p05_p95_mm': np.percentile(residual, [5, 95]).tolist(),
        'warning': 'Assumes same-pixel registration only for this consistency calculation. '
                   'RGB pose depends on RGB intrinsics and board dimensions/flatness. '
                   'A tape measurement and multiple poses/ranges are still needed; no settings changed.',
    }
    annotated = rgb.copy()
    cv2.drawChessboardCorners(annotated, (7, 7), corners, True)
    roi_boundary = cv2.findContours(roi.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
    cv2.drawContours(annotated, roi_boundary, -1, (0, 255, 255), 1)
    if not cv2.imwrite(str(directory / 'board_roi.png'), annotated):
        raise RuntimeError('Could not write annotated diagnostic')
    (directory / 'board_analysis.json').write_text(json.dumps(result, indent=2) + '\n')
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    args = parser.parse_args()
    cv2.setNumThreads(1)
    print(json.dumps(analyze(args.directory), indent=2))
