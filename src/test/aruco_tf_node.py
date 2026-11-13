#!/usr/bin/env python3
"""用 OpenCV 5 的 ArucoDetector 自己检测标记并发布 TF。

替代 aruco_ros 的 single 节点：后者用的是自带的旧检测器，对这块 4x4 板
（画面里 ~78 px）经常检不出来；而 cv2.aruco 能稳定检出。

发布:
  TF:  <camera_frame> -> <marker_frame>   （默认 default_cam -> aruco_marker）
  topic: /aruco_marker/pose (geometry_msgs/PoseStamped)

用法:
  pixi run python src/test/aruco_tf_node.py --dict DICT_4X4_50 --marker-id 0 --marker-size 0.05
"""
import argparse
import math
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import TransformBroadcaster


def quat_from_R(R):
    tr = np.trace(R)
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        return ((R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, 0.25 * s)
    i = int(np.argmax(np.diag(R)))
    if i == 0:
        s = math.sqrt(1 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        return (0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s, (R[2, 1] - R[1, 2]) / s)
    if i == 1:
        s = math.sqrt(1 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        return ((R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s, (R[0, 2] - R[2, 0]) / s)
    s = math.sqrt(1 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
    return ((R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s, (R[1, 0] - R[0, 1]) / s)


class ArucoTf(Node):
    def __init__(self, args):
        super().__init__('aruco_tf_node')
        self.args = args
        self.bridge = CvBridge()
        self.img = None
        self.K = None
        self.D = None
        self.det = cv2.aruco.ArucoDetector(
            cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, args.dict)),
            cv2.aruco.DetectorParameters())
        self.tf = TransformBroadcaster(self)
        self.pub = self.create_publisher(PoseStamped, args.pose_topic, 10)
        self.create_subscription(Image, args.image_topic, self.on_img, 10)
        self.create_subscription(CameraInfo, args.info_topic, self.on_info, 10)
        self.create_timer(1.0 / args.rate, self.tick)
        self.last_log = 0.0

    def on_img(self, msg):
        try:
            self.img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception:
            pass

    def on_info(self, msg):
        if msg.k is not None and any(msg.k):
            self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            self.D = np.array(msg.d, dtype=np.float64).reshape(-1, 1)

    def tick(self):
        if self.img is None or self.K is None:
            return
        gray = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.det.detectMarkers(gray)
        if ids is None or self.args.marker_id not in ids.ravel():
            if time.time() - self.last_log > 5:
                self.last_log = time.time()
                self.get_logger().warning('没检测到标记')
            return
        idx = int(np.where(ids.ravel() == self.args.marker_id)[0][0])
        c = cv2.cornerSubPix(gray, corners[idx].reshape(-1, 1, 2).astype(np.float32),
                             (5, 5), (-1, -1),
                             (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 0.01))
        s = self.args.marker_size / 2.0
        obj = np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]], dtype=np.float64)
        ok, rvec, tvec = cv2.solvePnP(obj, c.reshape(-1, 2).astype(np.float64), self.K, self.D,
                                      flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok:
            return
        R, _ = cv2.Rodrigues(rvec)
        tv = np.asarray(tvec).ravel()
        x, y, z, w = quat_from_R(R)

        ts = TransformStamped()
        ts.header.stamp = self.get_clock().now().to_msg()
        ts.header.frame_id = self.args.camera_frame
        ts.child_frame_id = self.args.marker_frame
        ts.transform.translation.x = float(tv[0])
        ts.transform.translation.y = float(tv[1])
        ts.transform.translation.z = float(tv[2])
        ts.transform.rotation.x, ts.transform.rotation.y = float(x), float(y)
        ts.transform.rotation.z, ts.transform.rotation.w = float(z), float(w)
        self.tf.sendTransform(ts)

        ps = PoseStamped()
        ps.header = ts.header
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = map(float, tv)
        ps.pose.orientation.x, ps.pose.orientation.y = float(x), float(y)
        ps.pose.orientation.z, ps.pose.orientation.w = float(z), float(w)
        self.pub.publish(ps)
        self.last_log = 0.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--dict', default='DICT_4X4_50')
    ap.add_argument('--marker-id', type=int, default=0)
    ap.add_argument('--marker-size', type=float, default=0.05)
    ap.add_argument('--camera-frame', default='default_cam')
    ap.add_argument('--marker-frame', default='aruco_marker')
    ap.add_argument('--image-topic', default='/image_raw')
    ap.add_argument('--info-topic', default='/camera_info')
    ap.add_argument('--pose-topic', default='/aruco_marker/pose')
    ap.add_argument('--rate', type=float, default=15.0)
    args = ap.parse_args()

    rclpy.init()
    node = ArucoTf(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
