#!/usr/bin/env python3
"""Offline native-depth geometry hypotheses, NOT depth calibration/deployment.

Uses measured RGB/IR candidate without fitting to these depth samples. A planar
board cannot uniquely determine depth pixel offset, distortion or depth bias.
Hypotheses are reported separately, never selected or installed automatically.
"""
import argparse
import hashlib
import json
from pathlib import Path
import cv2
import numpy as np

from check_calibration_target import detect, object_points
from capture_rgb_ir_calibration import infrared_preview, align_ir_order
from analyze_board_pair import plane_depth_mm
from inspect_rgbd_pair import depth_edges


def detect_offline(gray):
    corners=detect(gray,(7,7))
    if corners is not None:
        return corners
    ok,corners=cv2.findChessboardCornersSB(gray,(7,7),flags=cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY)
    if not ok or corners is None or len(corners)!=49 or not np.isfinite(corners).all():
        return None
    return corners


def residual_stats(values):
    values = np.asarray(values)
    return {'median_mm': float(np.median(values)),
            'p05_p95_mm': np.percentile(values,[5,95]).tolist(),
            'rms_mm': float(np.sqrt(np.mean(values**2)))}


def rgb_to_ir_board_pose(rvec, tvec, rotation, translation):
    return cv2.Rodrigues(rotation @ cv2.Rodrigues(rvec)[0])[0], rotation @ tvec + translation.reshape(3,1)


def software_register(depth, kd, dd, kr, dr, rotation, translation):
    """Saved pixels only. X_ir = R * X_rgb + t; inverse maps native Z to RGB.

    Assumes native depth is optical-axis Z in the IR coordinate system. Nearest
    projected pixel and nearest RGB Z win; holes are intentionally NOT filled.
    """
    ys,xs = np.nonzero(depth)
    rays = cv2.undistortPoints(np.column_stack([xs,ys]).astype(float).reshape(-1,1,2),kd,dd).reshape(-1,2)
    xyz_ir = np.column_stack([rays,np.ones(len(xs))]) * (depth[ys,xs,None]/1000.)
    xyz_rgb = (xyz_ir-translation) @ rotation
    valid = np.isfinite(xyz_rgb).all(axis=1) & (xyz_rgb[:,2]>0)
    xyz_rgb = xyz_rgb[valid]
    pixels = cv2.projectPoints(xyz_rgb,np.zeros(3),np.zeros(3),kr,dr)[0].reshape(-1,2)
    valid = np.isfinite(pixels).all(axis=1) & (pixels[:,0]>=0) & (pixels[:,0]<639.5) & (pixels[:,1]>=0) & (pixels[:,1]<479.5)
    pixels = np.rint(pixels[valid]).astype(int)
    z = xyz_rgb[valid,2]*1000.
    out = np.full(640*480, np.inf)
    np.minimum.at(out,pixels[:,1]*640+pixels[:,0],z)
    out[~np.isfinite(out)] = 0
    return np.rint(out.reshape(480,640)).astype(np.uint16)


def analyze(directory, candidate_path, output):
    metadata = json.loads((directory/'capture.json').read_text())
    candidate = json.loads(candidate_path.read_text())
    if metadata['status'] != 'complete' or metadata['depth_registration'] is not False:
        raise ValueError('Require complete native-depth capture')
    if output.exists():
        raise ValueError('Output must be new')
    repo = Path(__file__).resolve().parents[2]
    if output == repo or repo in output.parents:
        raise ValueError('Private outputs must be outside repository')
    kr, dr = np.array(candidate['rgb_k']),np.array(candidate['rgb_d'])
    ki, di = np.array(candidate['ir_k']),np.array(candidate['ir_d'])
    rotation, translation = np.array(candidate['r_rgb_to_ir']),np.array(candidate['t_rgb_to_ir_m'])
    output.mkdir(mode=0o700, parents=True, exist_ok=False)
    report = {'candidate_sha256':hashlib.sha256(candidate_path.read_bytes()).hexdigest(),
              'capture_sha256':hashlib.sha256((directory/'capture.json').read_bytes()).hexdigest(),
              'corner_detection':'Full-frame standard detector then SB NORMALIZE_IMAGE|EXHAUSTIVE|ACCURACY fallback; no crop or manually placed corners. IR normalized at percentiles 1 and 99.',
              'samples':[], 'calibration_applied':False,'approved_for_slam':False,
              'limitations':['Single pose, three time-separated samples; not independent pose coverage.',
                  'Depth assumed optical-axis Z in IR coordinates, not independently verified.',
                  'Board plane cannot uniquely validate depth/IR pixel offset or distortion.',
                  'RGB pose, square size, flatness and prior calibration uncertainty affect these residuals.',
                  'No transform, CameraInfo, firmware or live topic changed by this analysis.']}
    obj = object_points(7,7,.035)
    for entry in metadata['samples']:
        idx=entry['index']
        rgb=cv2.imread(str(directory/entry['files']['rgb']))
        ir=cv2.imread(str(directory/entry['files']['ir']),cv2.IMREAD_UNCHANGED)
        depth=cv2.imread(str(directory/entry['files']['depth']),cv2.IMREAD_UNCHANGED)
        if rgb is None or rgb.shape!=(480,640,3) or ir is None or ir.shape!=(480,640) or depth is None or depth.shape!=(480,640) or depth.dtype!=np.uint16:
            raise ValueError('Invalid saved images')
        for key in ('rgb','ir','depth'):
            if entry['camera_info'][key]['stamp_ns'] != entry['image_headers'][key]['stamp_ns']:
                raise ValueError('CameraInfo stamp mismatch')
        if not np.allclose(np.array(entry['camera_info']['rgb']['k']).reshape(3,3),kr,rtol=0,atol=1e-6) or not np.allclose(entry['camera_info']['rgb']['d'],dr,rtol=0,atol=1e-9):
            raise ValueError('RGB calibration changed')
        rc=detect_offline(cv2.cvtColor(rgb,cv2.COLOR_BGR2GRAY))
        ig=infrared_preview(ir)[0]
        ic=detect_offline(ig)
        if rc is None or ic is None:
            raise ValueError(f'Sample {idx}: board not found in both RGB and IR')
        ic, ordering=align_ir_order(rc,ic)
        ok,rr,tr=cv2.solvePnP(obj,rc,kr,dr)
        if not ok:
            raise ValueError('RGB pose failed')
        ri,ti=rgb_to_ir_board_pose(rr,tr,rotation,translation)
        predicted=cv2.projectPoints(obj,ri,ti,ki,di)[0]
        rgb_pred=cv2.projectPoints(obj,rr,tr,kr,dr)[0]
        normal=cv2.Rodrigues(rr)[0][:,2]
        sample={'index':idx,'rgb_pose_rms_px':float(np.sqrt(np.mean(np.sum((rgb_pred-rc)**2,axis=2)))),
                'rgb_to_ir_prediction_rms_px':float(np.sqrt(np.mean(np.sum((predicted-ic)**2,axis=2)))),
                'rgb_board_origin_m':tr.reshape(3).tolist(),
                'board_tilt_deg':float(np.degrees(np.arccos(np.clip(abs(normal[2]),0,1)))),
                'ir_minus_rgb_ms':entry['ir_minus_rgb_ms'],'depth_minus_rgb_ms':entry['depth_minus_rgb_ms'],
                'hypotheses':{},'ordering':ordering}
        for label,offset in [('ir_pixels_no_offset',(0,0)),('driver_minus_5_minus_4',(-5,-4))]:
            shifted=ic+np.array(offset).reshape(1,1,2)
            roi=np.zeros((480,640),np.uint8)
            cv2.fillConvexPoly(roi,np.rint(cv2.convexHull(shifted.astype(np.float32))).astype(np.int32),255)
            roi=cv2.erode(roi,np.ones((9,9),np.uint8))>0
            ys,xs=np.nonzero(roi & (depth>0))
            if len(xs)<500:
                raise ValueError('Insufficient board-interior depth')
            kd=ki.copy()
            kd[0,2]+=offset[0];kd[1,2]+=offset[1]
            for distortion_label,dd in [('ir_distortion',di),('zero_distortion',np.zeros(5))]:
                expected=plane_depth_mm(np.column_stack([xs,ys]),kd,dd,ri,ti)
                sample['hypotheses'][label+'_'+distortion_label]={
                    **residual_stats(depth[ys,xs].astype(float)-expected),
                    'roi_valid_fraction':float(len(xs)/np.count_nonzero(roi)),
                    'roi_pixels':len(xs),'observed_depth_median_mm':float(np.median(depth[ys,xs]))}
            registered=software_register(depth,kd,di,kr,dr,rotation,translation)
            overlay=rgb.copy()
            jumps,_=depth_edges(registered)
            overlay[jumps]=(255,255,0)
            cv2.drawChessboardCorners(overlay,(7,7),rc,True)
            cv2.putText(overlay,'OFFLINE HYPOTHESIS: '+label,(8,18),cv2.FONT_HERSHEY_SIMPLEX,.43,(0,0,255),1,cv2.LINE_AA)
            for name,pixels in [(f'{idx:02d}_{label}_registered_mm.png',registered),
                                (f'{idx:02d}_{label}_overlay.png',overlay)]:
                if not cv2.imwrite(str(output/name),pixels):
                    raise RuntimeError('Diagnostic image write failed')
        review=cv2.cvtColor(ig,cv2.COLOR_GRAY2BGR)
        for actual,pred in zip(ic.reshape(-1,2),predicted.reshape(-1,2)):
            cv2.circle(review,tuple(np.rint(actual).astype(int)),3,(0,255,0),1)
            cv2.drawMarker(review,tuple(np.rint(pred).astype(int)),(0,0,255),cv2.MARKER_CROSS,6,1)
        if not cv2.imwrite(str(output/f'{idx:02d}_ir_prediction.png'),review):
            raise RuntimeError('Review image write failed')
        report['samples'].append(sample)
    (output/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    return report


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory',type=Path)
    parser.add_argument('candidate',type=Path)
    parser.add_argument('--output-dir',type=Path,required=True)
    args=parser.parse_args()
    cv2.setNumThreads(1)
    print(json.dumps(analyze(args.directory.resolve(),args.candidate.resolve(),args.output_dir.resolve()),indent=2))
