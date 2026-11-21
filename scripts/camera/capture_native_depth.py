#!/usr/bin/env python3
"""Capture three stationary RGB/IR/native-depth samples; no fit or deployment.

Requires native_depth_check.launch.py and a still, fully visible 7x7/35mm board.
Reads images and the driver's registration parameter; never sets parameters.
Saves native PNGs plus exact-stamp CameraInfo to a new directory outside the repository.
"""
import argparse
from collections import Counter, deque
from datetime import datetime, timezone
import json
import time

import cv2

from capture_rgb_ir_calibration import infrared_array, info_dict, stamp, validate_output
from inspect_rgbd_pair import image_array


def nearest_complete(frames, infos, max_delta_ns=25000000):
    if not all(frames.values()):
        return None
    for rgb in reversed(frames['rgb']):
        picked = {'rgb': rgb}
        for key in ('ir', 'depth'):
            if not stamp(frames[key][0]) <= stamp(rgb) <= stamp(frames[key][-1]):
                break
            m = min(frames[key], key=lambda m: abs(stamp(m)-stamp(rgb)))
            if abs(stamp(m)-stamp(rgb)) > max_delta_ns:
                break
            picked[key] = m
        if len(picked) != 3:
            continue
        meta = {key: next((m for m in infos[key] if stamp(m)==stamp(value)), None)
                for key, value in picked.items()}
        if all(m is not None for m in meta.values()):
            return picked, meta
    return None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', required=True)
    args = parser.parse_args()
    output = validate_output(args.output_dir)
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, Image
    from rcl_interfaces.srv import GetParameters
    from rcl_interfaces.msg import ParameterType
    cv2.setNumThreads(1)
    rclpy.init(args=[])
    node = rclpy.create_node('roam_native_depth_check_capture')
    image_subs, info_subs = [], []
    frames = {key: deque(maxlen=100) for key in ('rgb', 'ir', 'depth')}
    infos = {key: deque(maxlen=120) for key in frames}
    counts = Counter()
    manifest = {'created_utc': datetime.now(timezone.utc).isoformat(),
                'status': 'recording', 'depth_registration': False,
                'calibration_applied': False, 'samples': [],
                'limitations': ['Header proximity is not hardware synchronization.',
                                'Three repetitions at one stationary pose, not three independent poses.',
                                'Native depth/IR pixel relationship remains a hypothesis.']}
    def spin_for(seconds):
        end = time.monotonic()+seconds
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
    def receive(key, m):
        frames[key].append(m)
        counts[key] += 1
    def persist():
        (output/'capture.json').write_text(json.dumps(manifest, indent=2, allow_nan=False)+'\n')
    try:
        client = node.create_client(GetParameters, '/camera/driver/get_parameters')
        if not client.wait_for_service(timeout_sec=5):
            raise RuntimeError('No camera driver parameter service')
        request = GetParameters.Request()
        request.names = ['depth_registration']
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5)
        if not future.done() or future.result() is None:
            raise RuntimeError('Could not verify registration parameter')
        values = future.result().values
        if len(values) != 1 or values[0].type != ParameterType.PARAMETER_BOOL or values[0].bool_value:
            raise RuntimeError('Refusing capture: driver must explicitly have depth_registration=false')
        output.mkdir(mode=0o700, parents=True, exist_ok=False)
        persist()
        topics = {'rgb': '/camera/color', 'ir': '/camera/ir', 'depth': '/camera/depth_raw'}
        for key, prefix in topics.items():
            info_subs.append(node.create_subscription(CameraInfo, prefix+'/camera_info',
                lambda m,key=key: infos[key].append(m), qos_profile_sensor_data))
        spin_for(2)  # legacy driver discovers metadata before image-match callback
        for key, prefix in topics.items():
            suffix = '/image' if key == 'depth' else '/image_raw'
            image_subs.append(node.create_subscription(Image, prefix+suffix,
                lambda m,key=key: receive(key,m), qos_profile_sensor_data))
        start, last_save, last_stamp = time.monotonic(), 0., None
        while rclpy.ok() and time.monotonic()-start < 25 and len(manifest['samples']) < 3:
            rclpy.spin_once(node, timeout_sec=0.05)
            now = time.monotonic()
            if now-start < 4 or now-last_save < 2:
                continue
            selected = nearest_complete(frames, infos)
            if selected is None:
                continue
            messages, meta = selected
            if stamp(messages['rgb']) == last_stamp:
                continue
            if messages['depth'].header.frame_id != 'camera_depth_optical_frame':
                raise RuntimeError('Unexpected depth frame: not native-depth configuration')
            pixels = {'rgb': cv2.cvtColor(image_array(messages['rgb']), cv2.COLOR_RGB2BGR),
                      'ir': infrared_array(messages['ir']),
                      'depth': image_array(messages['depth'])}
            index = len(manifest['samples'])+1
            entry = {'index': index, 'files': {}, 'camera_info': {}, 'image_headers': {}}
            for key, array in pixels.items():
                name = f'sample_{index:02d}_{key}.png'
                if not cv2.imwrite(str(output/name), array):
                    raise RuntimeError('Image write failed')
                entry['files'][key] = name
                entry['camera_info'][key] = info_dict(meta[key])
                entry['image_headers'][key] = {'stamp_ns': stamp(messages[key]),
                    'frame_id': messages[key].header.frame_id, 'encoding': messages[key].encoding}
            entry['ir_minus_rgb_ms'] = (stamp(messages['ir'])-stamp(messages['rgb']))/1e6
            entry['depth_minus_rgb_ms'] = (stamp(messages['depth'])-stamp(messages['rgb']))/1e6
            manifest['samples'].append(entry)
            persist()
            print(json.dumps({'saved': index, 'counts': dict(counts)}), flush=True)
            last_stamp, last_save = stamp(messages['rgb']), now
        manifest['status'] = 'complete' if len(manifest['samples'])==3 else 'incomplete'
        manifest['received'] = dict(counts)
        persist()
        print(json.dumps({'status':manifest['status'], 'output':str(output), 'received':dict(counts)}), flush=True)
        return 0 if manifest['status']=='complete' else 2
    finally:
        for sub in info_subs:
            node.destroy_subscription(sub)
        spin_for(2)
        for sub in image_subs:
            node.destroy_subscription(sub)
        spin_for(2)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
