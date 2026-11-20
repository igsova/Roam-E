#!/usr/bin/env python3
"""Publish a PCL binary PLY (x y z ... red green blue) once as a latched
PointCloud2, e.g. an RTAB-Map dense export, so Foxglove can show it.

Usage (Pi, ROS sourced):
  python3 publish_ply_cloud.py office_dense_cloud.ply --voxel 0.02 \
      --topic /rtabmap/dense_cloud --frame map
"""
import argparse

import numpy as np

TYPES = {'float': '<f4', 'float32': '<f4', 'double': '<f8', 'uchar': 'u1',
         'uint8': 'u1', 'int': '<i4', 'uint': '<u4', 'short': '<i2', 'ushort': '<u2'}


def read_ply(path):
    with open(path, 'rb') as f:
        fields, count, element = [], 0, None
        while True:
            line = f.readline().decode('ascii').strip()
            if line == 'end_header':
                break
            parts = line.split()
            if parts[0] == 'format' and parts[1] != 'binary_little_endian':
                raise ValueError('only binary_little_endian PLY is supported')
            if parts[0] == 'element':
                element = parts[1]
                if element == 'vertex':
                    count = int(parts[2])
            elif parts[0] == 'property' and element == 'vertex':
                fields.append((parts[2], TYPES[parts[1]]))
        return np.frombuffer(f.read(count * np.dtype(fields).itemsize),
                             np.dtype(fields), count=count)


def voxel_downsample(xyz, rgb, voxel):
    if voxel <= 0:
        return xyz, rgb
    keys = np.floor(xyz / voxel).astype(np.int64)
    _, first = np.unique(keys, axis=0, return_index=True)
    return xyz[first], rgb[first]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('ply')
    parser.add_argument('--voxel', type=float, default=0.02)
    parser.add_argument('--topic', default='/rtabmap/dense_cloud')
    parser.add_argument('--frame', default='map')
    args = parser.parse_args()

    vertices = read_ply(args.ply)
    xyz = np.column_stack([vertices['x'], vertices['y'], vertices['z']]).astype('<f4')
    rgb = np.column_stack([vertices['red'], vertices['green'], vertices['blue']])
    xyz, rgb = voxel_downsample(xyz, rgb, args.voxel)

    import rclpy
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import PointCloud2, PointField

    packed = (rgb[:, 0].astype(np.uint32) << 16) | (rgb[:, 1].astype(np.uint32) << 8) | rgb[:, 2]
    data = np.zeros(len(xyz), np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('rgb', '<u4')]))
    data['x'], data['y'], data['z'], data['rgb'] = xyz[:, 0], xyz[:, 1], xyz[:, 2], packed

    rclpy.init()
    node = rclpy.create_node('ply_cloud_publisher')
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
    pub = node.create_publisher(PointCloud2, args.topic, qos)
    msg = PointCloud2()
    msg.header.frame_id = args.frame
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.height, msg.width = 1, len(data)
    msg.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32 if n != 'rgb'
                             else PointField.UINT32, count=1)
                  for i, n in enumerate(('x', 'y', 'z', 'rgb'))]
    msg.is_bigendian, msg.point_step, msg.is_dense = False, 16, True
    msg.row_step = 16 * len(data)
    msg.data = data.tobytes()
    pub.publish(msg)
    node.get_logger().info(f'Published {len(data)} points on {args.topic} ({args.frame}); latched.')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
