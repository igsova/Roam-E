#!/usr/bin/env python3
"""Learned-feature visual relocalizer for RTAB-Map localization (R19).

The Pi's RTAB-Map build cannot run learned features, and its own ORB/SIFT
place recognition fails with the Astra Pro colour camera. This node matches
the current colour frame against a keyframe cache built from the map
(tools/r19_visual_keyframes.py, XFeat by default), solves PnP with the
keyframes' depth, and publishes the robot pose as an initial pose. It never
commands motion.

- Retrieval: fast mutual-nearest-neighbour matching + PnP against every
  keyframe; the best `fine_candidates` are re-matched with the learned
  matcher (LighterGlue for XFeat) for the final pose.
- Pose: one PnP over the learned matches of the best candidates combined;
  per-keyframe PnPs must agree with it within agree_m / agree_deg (one
  keyframe, or >= strong_inliers, when relocalizing; two keyframes for a
  drift correction).
- The pose is computed for the image time and carried to "now" with the
  odom->base transform, so it stays valid while the robot drives.
- Triggers: automatically while RTAB-Map reports not localized (covariance
  >= 1000, at most every retry_period s); every `period` s when period > 0
  (drift correction while driving: published only if it differs from the
  current pose by more than min_correction_m/deg, and corrections above
  max_correction_m/deg need two consecutive agreeing results;
  publish_corrections:=false only logs the camera-vs-current difference); and on
  demand via ~/relocalize (std_srvs/Trigger).

Run with the LightGlue venv and ROS sourced:
  ~/venvs/lightglue/bin/python r19_visual_relocalizer.py \
      --ros-args -p keyframes:=/path/keyframes.npz
"""
import math
import threading
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger
import tf2_ros

from r19_visual_features import VisualFeatures


def transform_to_matrix(t):
    q, p = t.transform.rotation, t.transform.translation
    x, y, z, w = q.x, q.y, q.z, q.w
    m = np.eye(4)
    m[:3, :3] = [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
    m[:3, 3] = [p.x, p.y, p.z]
    return m


def planar_pose(t_map_base):
    yaw = math.atan2(t_map_base[1, 0], t_map_base[0, 0])
    return float(t_map_base[0, 3]), float(t_map_base[1, 3]), yaw


def planar_matrix(x, y, yaw):
    m = np.eye(4)
    m[:2, :2] = [[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]]
    m[:2, 3] = [x, y]
    return m


def pose_difference(a, b):
    d_yaw = math.atan2(math.sin(a[2] - b[2]), math.cos(a[2] - b[2]))
    return math.hypot(a[0] - b[0], a[1] - b[1]), abs(d_yaw)


class Relocalizer(Node):
    def __init__(self):
        super().__init__('visual_relocalizer')
        defaults = {
            'keyframes': '',
            'features': 'xfeat',
            'max_keypoints': 1024,
            'threads': 2,
            'image_topic': '/camera/color/image_rect',
            'localization_topic': '/rtabmap/localization_pose',
            'output_topics': ['/rtabmap/initialpose'],
            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_frame': 'base_footprint',
            'fine_candidates': 3,
            'min_inliers': 30,
            'strong_inliers': 60,
            'agree_m': 0.25,
            'agree_deg': 10.0,
            'retry_period': 20.0,
            'period': 0.0,
            'min_correction_m': 0.10,
            'min_correction_deg': 5.0,
            'max_correction_m': 0.5,
            'max_correction_deg': 20.0,
            'auto': True,
            'publish_corrections': True,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self._p = lambda name: self.get_parameter(name).value  # noqa: E731

        self._features = VisualFeatures(self._p('features'), self._p('max_keypoints'),
                                        self._p('threads'))
        self._load_keyframes(self._p('keyframes'))

        self._latest = None
        self._lock = threading.Lock()
        self._busy = threading.Lock()
        self._last_attempt = 0.0
        self._not_localized = False
        self._pending_large = None
        self._tf = tf2_ros.Buffer(cache_time=Duration(seconds=30.0))
        tf2_ros.TransformListener(self._tf, self)
        self._pubs = [self.create_publisher(PoseWithCovarianceStamped, t, 1)
                      for t in self._p('output_topics')]
        cb = ReentrantCallbackGroup()
        self.create_subscription(Image, self._p('image_topic'), self._on_image,
                                 qos_profile_sensor_data, callback_group=cb)
        self.create_subscription(PoseWithCovarianceStamped, self._p('localization_topic'),
                                 self._on_localization, 10, callback_group=cb)
        work = MutuallyExclusiveCallbackGroup()
        self.create_service(Trigger, '~/relocalize', self._on_service, callback_group=work)
        self.create_timer(1.0, self._maybe_auto, callback_group=work)
        self.get_logger().info(
            f'Loaded {len(self._ids)} {self._features.kind} keyframes; '
            f'auto={self._p("auto")}, period={self._p("period")} s.')

    def _load_keyframes(self, path):
        data = np.load(path)
        kind = str(data['features']) if 'features' in data else 'superpoint'
        if kind != self._features.kind:
            raise RuntimeError(f'{path} holds {kind} features, node uses {self._features.kind}')
        torch = self._features._torch
        self._k = data['camera_matrix']
        w, h = (int(v) for v in data['image_size'])
        self._ids = [int(i) for i in data['ids']]
        self._kf = {}
        for i in self._ids:
            self._kf[i] = {
                'keypoints': torch.from_numpy(data[f'{i}_keypoints']),
                'descriptors': torch.from_numpy(data[f'{i}_descriptors']),
                'image_size': (w, h),
                'points': data[f'{i}_points'],
                'valid': data[f'{i}_valid'],
            }

    def _on_image(self, msg):
        with self._lock:
            self._latest = msg

    def _on_localization(self, msg):
        self._not_localized = msg.pose.covariance[0] >= 1000.0

    def _maybe_auto(self):
        if not self._p('auto'):
            return
        since = time.monotonic() - self._last_attempt
        if self._not_localized and since >= self._p('retry_period'):
            ok, text = self.relocalize(correction=False)
        elif not self._not_localized and self._p('period') > 0 and since >= self._p('period'):
            ok, text = self.relocalize(correction=True)
        else:
            return
        if not ok:
            self.get_logger().info(f'attempt failed: {text}')

    def _on_service(self, request, response):
        response.success, response.message = self.relocalize(correction=False)
        return response

    def _pnp(self, query_kp, matches, kf):
        matches = matches[kf['valid'][matches[:, 1]]]
        if len(matches) < 8:
            return 0, None
        ok, rvec, tvec, inliers = cv2.solvePnPRansac(
            kf['points'][matches[:, 1]].astype(np.float64),
            query_kp[matches[:, 0]].astype(np.float64), self._k, None,
            reprojectionError=4.0, iterationsCount=300, flags=cv2.SOLVEPNP_EPNP)
        if not ok or inliers is None:
            return 0, None
        r, _ = cv2.Rodrigues(rvec)
        t_cam_map = np.eye(4)
        t_cam_map[:3, :3], t_cam_map[:3, 3] = r, tvec.ravel()
        return len(inliers), np.linalg.inv(t_cam_map)

    def _pnp_points(self, points, pixels):
        if len(points) < 8:
            return 0, None
        ok, rvec, tvec, inliers = cv2.solvePnPRansac(
            points.astype(np.float64), pixels.astype(np.float64), self._k, None,
            reprojectionError=4.0, iterationsCount=500, flags=cv2.SOLVEPNP_EPNP)
        if not ok or inliers is None:
            return 0, None
        idx = inliers.ravel()
        rvec, tvec = cv2.solvePnPRefineLM(points[idx].astype(np.float64),
                                          pixels[idx].astype(np.float64), self._k, None,
                                          rvec, tvec)
        r, _ = cv2.Rodrigues(rvec)
        t_cam_map = np.eye(4)
        t_cam_map[:3, :3], t_cam_map[:3, 3] = r, tvec.ravel()
        return len(idx), np.linalg.inv(t_cam_map)

    def relocalize(self, correction):
        if not self._busy.acquire(blocking=False):
            return False, 'already running'
        try:
            self._last_attempt = time.monotonic()
            with self._lock:
                msg = self._latest
            if msg is None:
                return False, 'no colour image yet'
            t0 = time.monotonic()
            rgb = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)
            query = self._features.extract(cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY))
            query_kp = query['keypoints'].numpy()
            coarse = []
            for i in self._ids:
                n, _ = self._pnp(query_kp, self._features.match_fast(query, self._kf[i]),
                                 self._kf[i])
                coarse.append((n, i))
            coarse.sort(reverse=True)
            # Fine matching on the best candidates; one PnP over all of their
            # matches gives the pose, per-keyframe PnPs check agreement.
            results, all_points, all_pixels = [], [], []
            for _, i in coarse[:int(self._p('fine_candidates'))]:
                kf = self._kf[i]
                matches = self._features.match_fine(query, kf)
                n, t_map_cam = self._pnp(query_kp, matches, kf)
                if t_map_cam is not None:
                    results.append((n, i, t_map_cam))
                    valid = matches[kf['valid'][matches[:, 1]]]
                    all_points.append(kf['points'][valid[:, 1]])
                    all_pixels.append(query_kp[valid[:, 0]])
            results.sort(key=lambda r: -r[0])
            elapsed = time.monotonic() - t0
            if not results or results[0][0] < self._p('min_inliers'):
                best = results[0][0] if results else coarse[0][0]
                return False, f'no match: best {best} inliers ({elapsed:.1f}s)'
            combined_n, combined = self._pnp_points(np.vstack(all_points), np.vstack(all_pixels))
            if combined is None:
                return False, f'combined PnP failed ({elapsed:.1f}s)'

            stamp = Time.from_msg(msg.header.stamp)
            t_base_cam = transform_to_matrix(self._tf.lookup_transform(
                self._p('base_frame'), msg.header.frame_id, Time()))
            to_base = np.linalg.inv(t_base_cam)
            best_pose = planar_pose(combined @ to_base)
            best_n, best_id = results[0][0], results[0][1]
            agreeing = [i for n, i, t in results if n >= self._p('min_inliers') // 2 and
                        self._agrees(best_pose, planar_pose(t @ to_base))]
            needed = 2 if correction else 1
            if len(agreeing) < needed and not (not correction and best_n >= self._p('strong_inliers')):
                return False, (f'unconfirmed: {len(agreeing)} of {len(results)} keyframes '
                               f'agree, best {best_n} inliers ({elapsed:.1f}s)')

            # Carry the pose from the image time to now with odometry.
            odom, base = self._p('odom_frame'), self._p('base_frame')
            try:
                then = self._tf.lookup_transform(odom, base, stamp)
            except tf2_ros.TransformException:
                then = self._tf.lookup_transform(odom, base, Time())
                self.get_logger().warning('no odometry at image time; using latest')
            t_odom_base_then = transform_to_matrix(then)
            t_odom_base_now = transform_to_matrix(self._tf.lookup_transform(odom, base, Time()))
            t_map_odom = planar_matrix(*best_pose) @ np.linalg.inv(t_odom_base_then)
            now_pose = planar_pose(t_map_odom @ t_odom_base_now)

            if correction and not self._accept_correction(now_pose):
                text = (f'pose consistent ({best_n} inliers, {elapsed:.1f}s); '
                        'no correction published')
                self.get_logger().info(text)
                return True, text
            self._publish(now_pose)
            text = (f'relocalized at x={now_pose[0]:.2f} y={now_pose[1]:.2f} '
                    f'yaw={math.degrees(now_pose[2]):.0f} deg (best keyframe {best_id}, '
                    f'{combined_n} combined inliers, {len(agreeing)} of {len(results)} '
                    f'keyframes agree, {elapsed:.1f}s)')
            self.get_logger().info(text)
            return True, text
        except Exception as error:  # report and keep running
            self.get_logger().error(f'relocalization failed: {error}')
            return False, str(error)
        finally:
            self._busy.release()

    def _agrees(self, a, b):
        dist, d_yaw = pose_difference(a, b)
        return dist <= self._p('agree_m') and d_yaw <= math.radians(self._p('agree_deg'))

    def _accept_correction(self, new_pose):
        """Drift correction policy while localized."""
        current = planar_pose(transform_to_matrix(self._tf.lookup_transform(
            self._p('map_frame'), self._p('base_frame'), Time())))
        dist, d_yaw = pose_difference(new_pose, current)
        self.get_logger().info(
            f'camera vs current pose: {dist:.2f} m, {math.degrees(d_yaw):.1f} deg')
        if not self._p('publish_corrections'):
            return False
        if dist < self._p('min_correction_m') and d_yaw < math.radians(self._p('min_correction_deg')):
            self._pending_large = None
            return False
        if dist <= self._p('max_correction_m') and d_yaw <= math.radians(self._p('max_correction_deg')):
            self._pending_large = None
            self.get_logger().info(f'correcting drift: {dist:.2f} m, {math.degrees(d_yaw):.0f} deg')
            return True
        previous, self._pending_large = self._pending_large, new_pose
        if previous is not None and self._agrees(previous, new_pose):
            self._pending_large = None
            self.get_logger().warning(
                f'large correction confirmed twice: {dist:.2f} m, {math.degrees(d_yaw):.0f} deg')
            return True
        self.get_logger().warning(
            f'large correction {dist:.2f} m, {math.degrees(d_yaw):.0f} deg held until confirmed')
        return False

    def _publish(self, pose):
        x, y, yaw = pose
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self._p('map_frame')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
        msg.pose.pose.orientation.z = math.sin(yaw / 2)
        msg.pose.pose.orientation.w = math.cos(yaw / 2)
        cov = [0.0] * 36
        cov[0] = cov[7] = 0.05 ** 2
        cov[35] = math.radians(5) ** 2
        msg.pose.covariance = cov
        for pub in self._pubs:
            pub.publish(msg)


def main():
    rclpy.init()
    node = Relocalizer()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
