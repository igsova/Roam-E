#!/usr/bin/env python3
"""Offline RGB/IR candidate fitting and leave-one-view-out validation.

No ROS imports, USB access, TF publication, or live configuration writes.
Existing independently collected RGB intrinsics are fixed throughout.
Every saved view is used; none is discarded based on its fitting residual.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path

import cv2
import numpy as np

SIZE = (640, 480)
CRITERIA = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-9)
MODELS = {
    'k1': cv2.CALIB_ZERO_TANGENT_DIST | cv2.CALIB_FIX_K2 | cv2.CALIB_FIX_K3,
    'k1_k2': cv2.CALIB_ZERO_TANGENT_DIST | cv2.CALIB_FIX_K3,
    'k1_k2_p1_p2': cv2.CALIB_FIX_K3,
}


def object_grid(square):
    points = np.zeros((49, 3), np.float32)
    points[:, :2] = np.mgrid[:7, :7].T.reshape(-1, 2) * square
    return points


def load_dataset(path):
    raw = Path(path).read_bytes()
    data = json.loads(raw)
    if data.get('status') != 'capture_finished' or tuple(data['size']) != SIZE:
        raise ValueError('Require a completed 640x480 capture')
    if data['pattern'] != [7, 7] or not 0.005 < data['square_m'] < 0.2:
        raise ValueError('Unexpected board geometry')
    pairs = data['pairs']
    if len(pairs) < 6 or len({p['index'] for p in pairs}) != len(pairs):
        raise ValueError('Need at least six uniquely indexed views')
    first = pairs[0]['rgb_camera_info']
    for pair in pairs:
        for kind in ('rgb', 'ir'):
            xy = np.asarray(pair[kind+'_corners_px'], np.float64)
            info = pair[kind+'_camera_info']
            if xy.shape != (49, 2) or not np.isfinite(xy).all():
                raise ValueError('Invalid corner array')
            if (xy < 0).any() or (xy[:, 0] >= 640).any() or (xy[:, 1] >= 480).any():
                raise ValueError('Corners outside image')
            if info['stamp_ns'] != pair[kind+'_stamp_ns']:
                raise ValueError('Image/CameraInfo timestamp mismatch')
            if (info['width'], info['height']) != SIZE:
                raise ValueError('CameraInfo resolution mismatch')
        if abs(pair['ir_stamp_ns']-pair['rgb_stamp_ns']) > 25_000_000:
            raise ValueError('Header pair exceeds capture tolerance')
        for key in ('k', 'd', 'distortion_model'):
            if pair['rgb_camera_info'][key] != first[key]:
                raise ValueError('RGB calibration changed within dataset')
    matrix = np.asarray(first['k'], np.float64).reshape(3, 3)
    distortion = np.asarray(first['d'], np.float64).reshape(-1, 1)
    if first['distortion_model'] != 'plumb_bob' or distortion.size != 5:
        raise ValueError('Require five-coefficient plumb_bob RGB calibration')
    if not np.isfinite(matrix).all() or not np.isfinite(distortion).all():
        raise ValueError('Invalid RGB calibration')
    return data, matrix, distortion, hashlib.sha256(raw).hexdigest()


def project(points, rotation, translation, matrix, distortion):
    return cv2.projectPoints(points, cv2.Rodrigues(rotation)[0], translation,
                             matrix, distortion)[0].reshape(-1, 2)


def pose(points, corners, matrix, distortion):
    ok, rvec, translation = cv2.solvePnP(points, corners, matrix, distortion,
                                       flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        raise ValueError('PnP failed')
    rotation = cv2.Rodrigues(rvec)[0]
    if (rotation @ points.T + translation)[2].min() <= 0:
        raise ValueError('Board pose behind camera')
    return rotation, translation


def stats(errors):
    values = np.asarray(errors, np.float64).reshape(-1)
    return {'rms_px': float(np.sqrt(np.mean(values**2))),
            'median_px': float(np.median(values)),
            'p95_px': float(np.percentile(values, 95)),
            'max_px': float(values.max())}


def radial_check(matrix, distortion):
    # Require positive radial scale and dr_distorted/dr throughout the sensor
    # radius. This checks radial invertibility, not tangential-map bijectivity.
    corners = np.array([[0,0], [639,0], [639,479], [0,479]], np.float64)
    radii = np.linalg.norm((corners-matrix[:2, 2])/np.diag(matrix)[:2], axis=1)
    r = np.linspace(0, radii.max()*1.05, 250)
    k1, k2, _, _, k3 = distortion.reshape(-1)[:5]
    scale = 1+k1*r**2+k2*r**4+k3*r**6
    derivative = 1+3*k1*r**2+5*k2*r**4+7*k3*r**6
    return {'passes': bool((scale > 0).all() and (derivative > 0).all()),
            'tested_radius': float(r[-1]), 'minimum_scale': float(scale.min()),
            'minimum_derivative': float(derivative.min())}


def fit(data, indices, model, rgb_k, rgb_d):
    obj = object_grid(data['square_m'])
    objects = [obj.copy() for i in indices]
    rgb = [np.asarray(data['pairs'][i]['rgb_corners_px'], np.float32) for i in indices]
    ir = [np.asarray(data['pairs'][i]['ir_corners_px'], np.float32) for i in indices]
    ir_k = np.asarray(data['pairs'][0]['ir_camera_info']['k'], np.float64).reshape(3,3)
    ir_d = np.zeros((5,1), np.float64)
    mono = None
    if model != 'driver_metadata_baseline':
        mono = cv2.calibrateCameraExtended(objects, ir, SIZE, ir_k, ir_d,
            flags=cv2.CALIB_USE_INTRINSIC_GUESS | MODELS[model], criteria=CRITERIA)
        ir_k, ir_d = mono[1:3]
    result = cv2.stereoCalibrate(objects, rgb, ir, rgb_k.copy(), rgb_d.copy(),
                               ir_k.copy(), ir_d.copy(), SIZE,
                               flags=cv2.CALIB_FIX_INTRINSIC, criteria=CRITERIA)
    rms, kr, dr, ki, di, rotation, translation, essential, fundamental = result
    if not np.array_equal(kr, rgb_k) or not np.array_equal(dr.reshape(-1), rgb_d.reshape(-1)):
        raise ValueError('Stereo solver unexpectedly changed RGB intrinsics')
    if not all(np.isfinite(x).all() for x in (ki, di, rotation, translation)):
        raise ValueError('Non-finite calibration result')
    return {'model': model, 'stereo_training_rms_px': float(rms),
            'ir_mono_training_rms_px': None if mono is None else float(mono[0]),
            'ir_intrinsic_std_deviations': None if mono is None else mono[5].reshape(-1).tolist(),
            'rgb_k': kr.tolist(), 'rgb_d': dr.reshape(-1).tolist(),
            'ir_k': ki.tolist(), 'ir_d': di.reshape(-1).tolist(),
            'r_rgb_to_ir': rotation.tolist(), 't_rgb_to_ir_m': translation.reshape(3).tolist(),
            'ir_optical_center_in_rgb_m': (-rotation.T @ translation).reshape(3).tolist(),
            'baseline_m': float(np.linalg.norm(translation)),
            'radial_mapping': radial_check(ki, di)}


def evaluate(data, index, solution):
    obj = object_grid(data['square_m'])
    pair = data['pairs'][index]
    rgb = np.asarray(pair['rgb_corners_px'], np.float64)
    ir = np.asarray(pair['ir_corners_px'], np.float64)
    kr, ki = (np.asarray(solution[k]) for k in ('rgb_k', 'ir_k'))
    dr, di = (np.asarray(solution[k]) for k in ('rgb_d', 'ir_d'))
    rotation = np.asarray(solution['r_rgb_to_ir'])
    translation = np.asarray(solution['t_rgb_to_ir_m']).reshape(3,1)
    rr, tr = pose(obj, rgb, kr, dr)
    ri, ti = pose(obj, ir, ki, di)
    predicted_ir = project(obj, rotation @ rr, rotation @ tr + translation, ki, di)
    predicted_rgb = project(obj, rotation.T @ ri, rotation.T @ (ti-translation), kr, dr)
    forward = np.linalg.norm(predicted_ir-ir, axis=1)
    reverse = np.linalg.norm(predicted_rgb-rgb, axis=1)
    return {'index': pair['index'], 'rgb_to_ir': stats(forward), 'ir_to_rgb': stats(reverse),
            'rgb_pose_tilt_deg': float(np.degrees(np.arccos(np.clip(abs(rr[2,2]),0,1)))),
            'rgb_pose_origin_z_m': float(tr[2,0]),
            'predicted_ir_corners_px': predicted_ir.tolist(),
            'ir_observed_corners_px': ir.tolist()}


def fit_rgb_anchored(data, indices, model, rgb_k, rgb_d):
    """Estimate IR model and rigid transform using RGB-derived 3D corners.

    This is a conditional estimate, NOT full bundle adjustment: uncertainty in
    RGB intrinsics and per-view PnP poses is ignored. Keep it a separate method.
    All 3D coordinates share the RGB camera frame, including across board poses.
    """
    obj = object_grid(data['square_m'])
    xyz, observations = [], []
    for i in indices:
        pair = data['pairs'][i]
        rgb = np.asarray(pair['rgb_corners_px'], np.float64)
        rotation, translation = pose(obj, rgb, rgb_k, rgb_d)
        xyz.append((rotation @ obj.T + translation).T)
        observations.append(np.asarray(pair['ir_corners_px'], np.float32))
    points = np.concatenate(xyz).astype(np.float32)
    corners = np.concatenate(observations).astype(np.float32)
    ki = np.asarray(data['pairs'][0]['ir_camera_info']['k'], np.float64).reshape(3,3)
    di = np.zeros((5,1), np.float64)
    std = None
    if model == 'driver_metadata_baseline':
        rotation, translation = pose(points, corners, ki, di)
        errors = np.linalg.norm(project(points, rotation, translation, ki, di)-corners, axis=1)
        rms = stats(errors)['rms_px']
    else:
        result = cv2.calibrateCameraExtended([points], [corners], SIZE, ki, di,
            flags=cv2.CALIB_USE_INTRINSIC_GUESS | MODELS[model], criteria=CRITERIA)
        rms, ki, di = result[:3]
        rotation = cv2.Rodrigues(result[3][0])[0]
        translation = result[4][0]
        std = result[5].reshape(-1).tolist()
    return {'model':model, 'anchored_ir_training_rms_px':float(rms),
        'stereo_training_rms_px':None, 'ir_mono_training_rms_px':None,
        'ir_intrinsic_std_deviations':std,
        'rgb_k':rgb_k.tolist(), 'rgb_d':rgb_d.reshape(-1).tolist(),
        'ir_k':ki.tolist(), 'ir_d':di.reshape(-1).tolist(),
        'r_rgb_to_ir':rotation.tolist(), 't_rgb_to_ir_m':translation.reshape(3).tolist(),
        'ir_optical_center_in_rgb_m':(-rotation.T @ translation).reshape(3).tolist(),
        'baseline_m':float(np.linalg.norm(translation)), 'radial_mapping':radial_check(ki,di)}


def difference(reference, other):
    r1, r2 = (np.asarray(x['r_rgb_to_ir']) for x in (reference, other))
    angle = np.linalg.norm(cv2.Rodrigues(r2 @ r1.T)[0]) * 180/math.pi
    delta = np.asarray(other['t_rgb_to_ir_m'])-reference['t_rgb_to_ir_m']
    return {'translation_change_mm': float(np.linalg.norm(delta)*1000),
            'rotation_change_deg': float(angle)}


def summarize_validation(full, folds):
    kr = np.asarray(full['ir_k'])
    ks = np.asarray([v['fit']['ir_k'] for v in folds])
    forward = [f['held_out']['rgb_to_ir']['rms_px'] for f in folds]
    reverse = [f['held_out']['ir_to_rgb']['rms_px'] for f in folds]
    return {
        'held_out_rgb_to_ir_pooled_rms_px': float(np.sqrt(np.mean(np.square(forward)))),
        'held_out_rgb_to_ir_worst_view_rms_px': max(forward),
        'held_out_ir_to_rgb_pooled_rms_px': float(np.sqrt(np.mean(np.square(reverse)))),
        'held_out_ir_to_rgb_worst_view_rms_px': max(reverse),
        'max_loo_translation_change_mm': max(f['change']['translation_change_mm'] for f in folds),
        'max_loo_rotation_change_deg': max(f['change']['rotation_change_deg'] for f in folds),
        'ir_fx_range_px': [float(ks[:,0,0].min()), float(ks[:,0,0].max())],
        'ir_fy_range_px': [float(ks[:,1,1].min()), float(ks[:,1,1].max())],
        'ir_cx_range_px': [float(ks[:,0,2].min()), float(ks[:,0,2].max())],
        'ir_cy_range_px': [float(ks[:,1,2].min()), float(ks[:,1,2].max())],
        'max_loo_focal_relative_change': float(max(
            np.max(np.abs(ks[:,0,0]/kr[0,0]-1)), np.max(np.abs(ks[:,1,1]/kr[1,1]-1)))),
        'all_loo_radial_checks_pass': all(f['fit']['radial_mapping']['passes'] for f in folds),
    }


def validate_output(path):
    path = Path(path).expanduser().resolve()
    repo = Path(__file__).resolve().parents[2]
    if path == repo or repo in path.parents or path.exists():
        raise ValueError('Output must be a NEW directory outside the repository')
    return path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('manifest')
    parser.add_argument('--output-dir', required=True)
    parser.add_argument('--method', choices=['separate', 'rgb-anchored'], default='separate')
    args = parser.parse_args()
    data, rgb_k, rgb_d, digest = load_dataset(args.manifest)
    output = validate_output(args.output_dir)
    output.mkdir(mode=0o700, parents=True)
    cv2.setNumThreads(1)
    report = {'opencv_version': cv2.__version__, 'manifest_sha256': digest,
        'method':args.method,
        'dataset': str(Path(args.manifest).resolve()), 'pair_count': len(data['pairs']),
        'calibration_applied': False, 'approved_for_deployment': False,
        'transform_convention': 'X_ir = R_rgb_to_ir * X_rgb + t_rgb_to_ir_m',
        'models': {}, 'limitations': [
            'No native-depth-to-IR or firmware-registered-depth validation.',
            'No hardware synchronization proof; still-board header pairs only.',
            'Nine mostly front-facing views; limited geometric excitation.',
            'Cross-validation uses the same capture session, not an independent validation set.',
            'RGB candidate uncertainty and chessboard manufacturing error are not propagated.',
            'RGB-anchored method additionally treats per-view RGB PnP coordinates as exact; not full joint bundle adjustment.',
            'Radial invertibility check alone does not prove full distortion-map bijectivity.'],
        'reference': 'https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html'}

    def save():
        temporary = output/'report.json.tmp'
        temporary.write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
        temporary.replace(output/'report.json')

    indices = list(range(len(data['pairs'])))
    fitter = fit if args.method == 'separate' else fit_rgb_anchored
    for model in ['driver_metadata_baseline'] + list(MODELS):
        full = fitter(data, indices, model, rgb_k, rgb_d)
        full['training_cross_projection'] = [evaluate(data, i, full) for i in indices]
        report['models'][model] = {'full_fit': full, 'leave_one_out': []}
        save()
        print(json.dumps({'event':'full_fit', 'model':model, 'rms_px':full.get('anchored_ir_training_rms_px', full['stereo_training_rms_px']),
                          'baseline_m':full['baseline_m'], 'ir_k':full['ir_k']}), flush=True)
        if model == 'driver_metadata_baseline':
            continue
        for omitted in indices:
            candidate = fitter(data, [i for i in indices if i != omitted], model, rgb_k, rgb_d)
            fold = {'omitted_index':data['pairs'][omitted]['index'], 'fit':candidate,
                    'held_out':evaluate(data, omitted, candidate), 'change':difference(full, candidate)}
            report['models'][model]['leave_one_out'].append(fold)
            save()
            print(json.dumps({'event':'held_out', 'model':model, 'index':fold['omitted_index'],
                  'rgb_to_ir_rms_px':fold['held_out']['rgb_to_ir']['rms_px'], **fold['change']}), flush=True)
        summary = summarize_validation(full, report['models'][model]['leave_one_out'])
        report['models'][model]['validation_summary'] = summary
        save()
        print(json.dumps({'event':'model_summary', 'model':model, **summary}), flush=True)
    report['status'] = 'offline_fitting_complete_not_applied'
    save()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
