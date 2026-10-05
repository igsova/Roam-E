#!/usr/bin/env python3
"""Subscribe to isolated candidate static TF; no sensor/motor/settings access.

Use only with a separate ROS_DOMAIN_ID and description-only state publisher.
Checks model axes, NOT actual LiDAR heading or physical metric calibration.
"""
import argparse
import json
import math
import time


def quaternion_rotation(q):
    x, y, z, w = q
    norm = math.sqrt(sum(v*v for v in q))
    if norm < 1e-10:
        raise ValueError('Zero quaternion')
    x, y, z, w = (v/norm for v in q)
    return [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
            [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
            [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]


def compose(a, b):
    return [[sum(a[i][k]*b[k][j] for k in range(3)) for j in range(3)]
            for i in range(3)]


def to_base(frames, child):
    result = [[1,0,0], [0,1,0], [0,0,1]]
    seen = set()
    while child != 'base_link':
        if child in seen:
            raise ValueError('TF cycle')
        seen.add(child)
        entry = frames[child]
        result = compose(quaternion_rotation(entry['quaternion_xyzw']), result)
        child = entry['parent']
    return result


def check(frames):
    expected = [[0,1,0], [-1,0,0], [0,0,1]]
    imu = to_base(frames, 'imu_link')
    assert max(abs(a-b) for aa,bb in zip(imu,expected)
               for a,b in zip(aa,bb)) < 1e-6, 'IMU native axes mismatch'
    camera = to_base(frames, 'camera_link')
    assert max(abs(camera[i][0]-v) for i,v in enumerate((1,0,0))) < 1e-6
    assert not any(name.endswith('_optical_frame') for name in frames), 'Optics should be disabled'
    assert len([name for name in frames if name.startswith('tof_')]) == 8
    assert frames['base_link']['parent'] == 'base_footprint'
    return {'status':'PASS', 'static_transforms':len(frames),
            'imu_native_to_base_rotation':imu, 'camera_body_forward_in_base':
            [camera[i][0] for i in range(3)], 'camera_optical_preview_frames':False,
            'hardware_or_motor_access':False}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout-s',type=float,default=8)
    args = parser.parse_args()
    if not math.isfinite(args.timeout_s) or not 0 < args.timeout_s <= 30:
        parser.error('timeout must be in (0,30] seconds')
    import rclpy
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from tf2_msgs.msg import TFMessage
    rclpy.init(args=[])
    node = rclpy.create_node('r19_candidate_tf_check')
    frames = {}
    def receive(message):
        for t in message.transforms:
            q = t.transform.rotation
            p = t.transform.translation
            frames[t.child_frame_id] = {'parent':t.header.frame_id,
                'xyz':[p.x,p.y,p.z], 'quaternion_xyzw':[q.x,q.y,q.z,q.w]}
    qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
    node.create_subscription(TFMessage,'/tf_static',receive,qos)
    try:
        deadline = time.monotonic()+args.timeout_s
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.1)
            if len(frames)>=20:
                result = check(frames)
                owners = node.get_publishers_info_by_topic('/tf_static')
                assert len(owners)==1, 'Isolated test must have one static TF publisher'
                result['publishers'] = [owner.node_name for owner in owners]
                print(json.dumps(result,indent=2))
                return
        raise RuntimeError('Timeout waiting for complete candidate static TF')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':
    main()
