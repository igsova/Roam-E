#!/usr/bin/env python3
"""Validate an offline fit on an unused legacy RGB/IR visibility capture.

No capture, recalibration, parameter tuning, live updates, or ROS communication.
Input sample directory contains rgb.png, ir_native.png, and report.json.
"""
import argparse
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np

from capture_rgb_ir_calibration import align_ir_order, infrared_preview
from check_calibration_target import detect
from fit_rgb_ir_calibration import evaluate, validate_output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('fit_report')
    parser.add_argument('sample_dir')
    parser.add_argument('--model', default='k1_k2_p1_p2')
    parser.add_argument('--output-dir', required=True)
    args = parser.parse_args()
    report = json.loads(Path(args.fit_report).read_text())
    full = report['models'][args.model]['full_fit']
    sample = Path(args.sample_dir).resolve()
    metadata = json.loads((sample/'report.json').read_text())
    np.testing.assert_allclose(metadata['camera_info']['rgb']['k'], np.asarray(full['rgb_k']).reshape(-1), rtol=0, atol=1e-9)
    np.testing.assert_allclose(metadata['camera_info']['rgb']['d'], full['rgb_d'], rtol=0, atol=1e-9)
    if abs(metadata['header_delta_ir_minus_rgb_ms']) > 25:
        raise ValueError('Legacy sample header mismatch exceeds 25 ms')
    cv2.setNumThreads(1)
    rgb = cv2.imread(str(sample/'rgb.png'), cv2.IMREAD_GRAYSCALE)
    native = cv2.imread(str(sample/'ir_native.png'), cv2.IMREAD_UNCHANGED)
    if rgb.shape != (480,640) or native.shape != (480,640) or native.dtype != np.uint16:
        raise ValueError('Unexpected saved image format')
    ir, bounds = infrared_preview(native)
    rc, ic = detect(rgb,(7,7)), detect(ir,(7,7))
    if rc is None or ic is None:
        raise ValueError('Full grid not found in both saved images')
    aligned, ordering = align_ir_order(rc,ic)
    dataset = {'square_m':.035, 'pairs':[{'index':1,
        'rgb_corners_px':rc.reshape(-1,2).tolist(), 'ir_corners_px':aligned.reshape(-1,2).tolist()}]}
    result = evaluate(dataset,0,full)
    result.update(sample_dir=str(sample), method=report['method'], model=args.model,
        independent_of_nine_view_fit=True, calibration_applied=False,
        header_delta_ms=metadata['header_delta_ir_minus_rgb_ms'],
        ir_normalization_bounds=bounds, corner_ordering=ordering,
        file_sha256={name:hashlib.sha256((sample/name).read_bytes()).hexdigest()
                     for name in ('rgb.png','ir_native.png','report.json')},
        limitation='One saved view at one range; not broad or native-depth validation.')
    output = validate_output(args.output_dir)
    output.mkdir(mode=0o700,parents=True)
    (output/'validation.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    candidate = {key:full[key] for key in ('rgb_k','rgb_d','ir_k','ir_d','r_rgb_to_ir',
        't_rgb_to_ir_m','ir_optical_center_in_rgb_m','baseline_m','radial_mapping')}
    candidate.update(status='experimental_offline_candidate_not_approved',
        method=report['method'], model=args.model, size=[640,480], square_m=.035,
        transform_convention=report['transform_convention'],
        training_manifest_sha256=report['manifest_sha256'],
        cross_validation=report['models'][args.model]['validation_summary'],
        external_validation_file='validation.json', calibration_applied=False,
        approved_for_deployment=False, limitations=report['limitations'])
    (output/'candidate.json').write_text(json.dumps(candidate,indent=2,allow_nan=False)+'\n')
    display = cv2.cvtColor(ir, cv2.COLOR_GRAY2BGR)
    for measured,predicted in zip(aligned.reshape(-1,2),result['predicted_ir_corners_px']):
        cv2.circle(display,tuple(np.rint(measured).astype(int)),4,(0,255,0),1)
        cv2.drawMarker(display,tuple(np.rint(predicted).astype(int)),(0,0,255),
                       cv2.MARKER_CROSS,7,1)
    cv2.putText(display,'Green: observed IR; red: predicted from RGB', (10,25),
                cv2.FONT_HERSHEY_SIMPLEX,.5,(255,255,255),1)
    cv2.imwrite(str(output/'validation_overlay.png'),display)
    print(json.dumps({key:value for key,value in result.items() if not key.endswith('corners_px')},allow_nan=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
