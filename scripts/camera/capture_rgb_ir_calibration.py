#!/usr/bin/env python3
"""Bounded stationary RGB/IR dataset recorder; never solve/apply calibration.

Consumes existing ROS image topics, saves native images outside Git. IR CameraInfo
subscriptions are released before Image subscriptions to stop the legacy stream.
"""
import argparse
from collections import Counter, deque
from datetime import datetime, timezone
import json
from pathlib import Path
import signal
import time

import cv2
import numpy as np

from check_calibration_target import detect, image_to_gray
from capture_rgb_calibration import board_quality, grid_distance


def stamp(m):
    return m.header.stamp.sec * 1000000000 + m.header.stamp.nanosec


def infrared_array(m):
    if (m.width, m.height) != (640, 480) or m.encoding not in ('mono16', '16UC1'):
        raise ValueError('Expected native 640x480 mono16 infrared')
    if m.step < m.width*2 or len(m.data) != m.step*m.height:
        raise ValueError('Invalid IR stride or data length')
    return np.ndarray((m.height, m.width), '>u2' if m.is_bigendian else '<u2',
                      buffer=m.data, strides=(m.step, 2)).astype(np.uint16)


def infrared_preview(native):
    lo, hi = np.percentile(native, [1, 99])
    if hi-lo < 20:
        raise ValueError('Insufficient infrared contrast')
    return np.clip((native.astype(float)-lo)*255/(hi-lo), 0, 255).astype(np.uint8), [float(lo), float(hi)]


def align_ir_order(rgb, infrared):
    """Resolve square-grid D4 order for the nearly parallel fixed camera pair.

    Keep the RGB ordering and compare centered shapes, independent of disparity.
    Reject large shape disagreement or ambiguous order; retain raw detections too.
    """
    target = np.asarray(rgb).reshape(49, 2)
    source = np.asarray(infrared).reshape(7, 7, 2)
    variants = [np.rot90(source, k, axes=(0, 1)) for k in range(4)]
    variants += [v[::-1] for v in variants[:]]
    centered = target-target.mean(axis=0)
    scored = []
    for i, variant in enumerate(variants):
        flat = variant.reshape(49, 2)
        centered_ir = flat-flat.mean(axis=0)
        score = float(np.sqrt(np.mean(np.sum((centered-centered_ir)**2, axis=1))))
        scored.append((score, i, flat))
    scored.sort(key=lambda v: v[0])
    best, second = scored[:2]
    if best[0] > 12 or second[0]-best[0] < 8:
        raise ValueError('Ambiguous or inconsistent RGB/IR corner correspondence')
    return best[2].reshape(49, 1, 2).copy(), {'ir_order_variant': best[1],
        'centered_shape_rms_px': best[0], 'ordering_margin_px': second[0]-best[0]}


def quality(gray, corners, infrared=False):
    reason, details = board_quality(gray, corners)
    # The projected dots disrupt this visible-light sharpness metric. Keep a
    # bounded looser IR guard, preserve native data, and document the exception.
    if infrared and reason == 'blurred_board':
        widths = np.asarray(details['edge_transition_px'])
        if np.isfinite(widths).all() and (widths > 0).all() and widths.max() <= 7 \
                and widths.max()/details['min_spacing_px'] <= 0.35:
            details['ir_sharpness_guard_relaxed_for_projector'] = True
            reason = None
    return reason, details


def info_dict(m):
    return {'stamp_ns': stamp(m), 'frame_id': m.header.frame_id,
            'width': m.width, 'height': m.height, 'distortion_model': m.distortion_model,
            'k': list(m.k), 'd': list(m.d), 'r': list(m.r), 'p': list(m.p)}


def validate_output(path):
    path = Path(path).expanduser().resolve()
    repo = Path(__file__).resolve().parents[2]
    if path == repo or repo in path.parents:
        raise ValueError('Private images must be outside the repository')
    if path.exists():
        raise ValueError('Output directory must be new')
    return path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', required=True)
    parser.add_argument('--seconds', type=int, default=300)
    parser.add_argument('--max-pairs', type=int, default=16)
    args = parser.parse_args()
    if not 15 <= args.seconds <= 300 or not 1 <= args.max_pairs <= 30:
        parser.error('Use 15..300 seconds and 1..30 pairs')
    try:
        output = validate_output(args.output_dir)
    except ValueError as error:
        parser.error(str(error))
    output.mkdir(mode=0o700, parents=True, exist_ok=False)
    manifest = {'format_version': 1, 'created_utc': datetime.now(timezone.utc).isoformat(),
        'size': [640,480], 'pattern': [7,7], 'square_m': 0.035, 'pairs': [],
        'calibration_applied': False, 'status': 'recording',
        'limitations': ['Stationary near-header pairs are not hardware synchronized.',
            'IR CameraInfo is not assumed calibrated.',
            'IR sharpness guard is relaxed for projected dots; validate fit/held-out views.',
            'D4 correspondence assumes roughly parallel camera images; inspect before fitting.']}
    def persist():
        temporary = output/'session.json.tmp'
        temporary.write_text(json.dumps(manifest, indent=2, allow_nan=False)+'\n')
        temporary.replace(output/'session.json')
    def emit(**values):
        print(json.dumps(values, allow_nan=False), flush=True)
    persist()
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image, CameraInfo
    cv2.setNumThreads(1)
    # Keep the context alive through ordered teardown, including Ctrl-C/SIGTERM.
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    stop_requested = [False]
    old_handlers = {}
    for signum in (signal.SIGINT, signal.SIGTERM):
        old_handlers[signum] = signal.getsignal(signum)
        signal.signal(signum, lambda signum, frame: stop_requested.__setitem__(0, True))
    node = rclpy.create_node('roam_rgb_ir_calibration_capture')
    frames = {'rgb': deque(maxlen=8), 'ir': deque(maxlen=70)}
    infos = {'rgb': deque(maxlen=60), 'ir': deque(maxlen=100)}
    received = Counter()
    last_arrival = {k: time.monotonic() for k in frames}
    def receive(k, m):
        received[k] += 1
        last_arrival[k] = time.monotonic()
        frames[k].append((m, last_arrival[k]))
    image_subs, info_subs = [], []
    for k, topic in [('rgb','/camera/color/camera_info'), ('ir','/camera/ir/camera_info')]:
        info_subs.append(node.create_subscription(CameraInfo, topic, lambda m,k=k: infos[k].append(m), qos_profile_sensor_data))
    # Legacy irConnectCb counts CameraInfo subscribers but is triggered by an
    # Image matched event. Let metadata discovery settle BEFORE Image connects.
    discovery_end = time.monotonic()+2
    while time.monotonic() < discovery_end:
        rclpy.spin_once(node, timeout_sec=0.1)
    for k, topic in [('rgb','/camera/color/image_raw'), ('ir','/camera/ir/image_raw')]:
        image_subs.append(node.create_subscription(Image, topic, lambda m,k=k: receive(k,m), qos_profile_sensor_data))
    started = next_check = heartbeat = time.monotonic()
    previous = None
    previous_at = last_save = 0.
    accepted = []
    rejected = Counter()
    last_stamp = None
    reason = 'waiting_for_images'
    emit(event='started', seconds=args.seconds, max_pairs=args.max_pairs, output_dir=str(output))
    try:
        while rclpy.ok() and not stop_requested[0] and time.monotonic()-started < args.seconds and len(accepted) < args.max_pairs:
            rclpy.spin_once(node, timeout_sec=0.05)
            now = time.monotonic()
            if now-started > 15 and any(now-v > 8 for v in last_arrival.values()):
                raise RuntimeError('RGB or IR stream stale for more than 8 seconds')
            if now-heartbeat > 5:
                emit(event='progress', elapsed_s=round(now-started), saved=len(accepted), last_status=reason,
                     received=dict(received))
                heartbeat = now
            if now < next_check or not all(frames.values()):
                continue
            selected = None
            for rgb, arrived in reversed(frames['rgb']):
                if stamp(rgb) == last_stamp or now-arrived > 1:
                    continue
                if not stamp(frames['ir'][0][0]) <= stamp(rgb) <= stamp(frames['ir'][-1][0]):
                    continue
                ir = min(frames['ir'], key=lambda item: abs(stamp(item[0])-stamp(rgb)))[0]
                if abs(stamp(ir)-stamp(rgb)) > 25000000:
                    continue
                ri = next((m for m in infos['rgb'] if stamp(m)==stamp(rgb)), None)
                ii = next((m for m in infos['ir'] if stamp(m)==stamp(ir)), None)
                if ri is not None and ii is not None:
                    selected = rgb, ir, ri, ii
                    break
            if selected is None:
                reason = 'waiting_for_matching_images_and_metadata'
                continue
            rgb, ir, ri, ii = selected
            last_stamp = stamp(rgb)
            next_check = now+0.85
            try:
                if (rgb.width,rgb.height) != (640,480):
                    raise ValueError('RGB resolution changed')
                rgb_gray = image_to_gray(rgb)
                ir_native = infrared_array(ir)
                ir_gray, bounds = infrared_preview(ir_native)
                rc, ic = detect(rgb_gray,(7,7)), detect(ir_gray,(7,7))
                reason = 'board_not_found_rgb' if rc is None else ('board_not_found_ir' if ic is None else None)
                if not reason:
                    r_reason, r_quality = quality(rgb_gray,rc)
                    i_reason, i_quality = quality(ir_gray,ic,infrared=True)
                    reason = ('rgb_'+r_reason) if r_reason else ('ir_'+i_reason if i_reason else None)
                if reason:
                    rejected[reason] += 1
                    previous = None
                    continue
                aligned, alignment = align_ir_order(rc,ic)
            except (ValueError, cv2.error) as error:
                reason = 'image_or_correspondence_error'
                rejected[reason] += 1
                previous = None
                emit(event=reason, detail=str(error))
                continue
            stable = previous is not None and now-previous_at < 3 \
                and max(grid_distance(rc,previous[0]),grid_distance(ic,previous[1])) <= 1.5
            previous, previous_at = (rc,ic), now
            if not stable:
                reason = 'hold_still'
            elif now-last_save < 4:
                reason = 'saved_move_to_new_pose'
            elif any(grid_distance(rc,v) < 12 for v in accepted):
                reason = 'duplicate_move_or_tilt_more'
            else:
                number = len(accepted)+1
                prefix = f'pair_{number:03d}'
                for name, pixels in [('rgb',rgb_gray), ('ir_native',ir_native), ('ir_preview',ir_gray)]:
                    if not cv2.imwrite(str(output/f'{prefix}_{name}.png'), pixels):
                        raise RuntimeError('Image write failed')
                entry = {'index':number, 'rgb_file':prefix+'_rgb.png',
                    'ir_file':prefix+'_ir_native.png', 'ir_preview_file':prefix+'_ir_preview.png',
                    'rgb_stamp_ns':stamp(rgb), 'ir_stamp_ns':stamp(ir),
                    'ir_minus_rgb_header_ms':(stamp(ir)-stamp(rgb))/1e6,
                    'rgb_corners_px':rc.reshape(-1,2).tolist(),
                    'ir_corners_px':aligned.reshape(-1,2).tolist(),
                    'ir_raw_detection_corners_px':ic.reshape(-1,2).tolist(),
                    'rgb_camera_info':info_dict(ri), 'ir_camera_info':info_dict(ii),
                    'rgb_quality':r_quality, 'ir_quality':i_quality,
                    'ir_normalization_bounds':bounds, **alignment}
                manifest['pairs'].append(entry)
                accepted.append(rc.copy())
                last_save = now
                persist()
                reason = 'saved_move_to_new_pose'
                emit(event='saved', index=number, rgb_bounds_px=r_quality['bounds_px'],
                     ir_bounds_px=i_quality['bounds_px'], header_delta_ms=entry['ir_minus_rgb_header_ms'])
                continue
            rejected[reason] += 1
        manifest['status'] = 'interrupted' if stop_requested[0] else 'capture_finished'
    except KeyboardInterrupt:
        manifest['status'] = 'interrupted'
    except Exception as error:
        manifest['status'] = 'failed'
        manifest['error'] = str(error)
        raise
    finally:
        manifest.update(received=dict(received), rejection_counts=dict(rejected), elapsed_s=time.monotonic()-started)
        persist()
        # Info must disappear from the graph before the Image disconnect callback.
        for sub in info_subs:
            node.destroy_subscription(sub)
        end = time.monotonic()+2
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
        for sub in image_subs:
            node.destroy_subscription(sub)
        end = time.monotonic()+2
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for signum, handler in old_handlers.items():
            signal.signal(signum, handler)
        emit(event='summary', saved=len(accepted), status=manifest['status'], rejection_counts=dict(rejected))
    return 0 if len(accepted) else 2


if __name__ == '__main__':
    raise SystemExit(main())
