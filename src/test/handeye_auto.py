#!/usr/bin/env python3
"""自动采集手眼标定样本并求解（eye-in-hand：相机装在末端）。

不需要点 GUI：直接订阅 /image_raw + /camera_info 自己检测 ArUco 板、自己算 PnP，
再用 tf 里的 base_link -> link6 组成 (T_base_ee, T_cam_marker) 样本对，
最后用 AX=XB 解出 T_ee_cam。

用法：
  pixi run python src/test/handeye_auto.py                  # 采 18 个样本后自动求解
  pixi run python src/test/handeye_auto.py --samples 25 --marker-size 0.05

特点：
  * 板子字典自动识别（4x4/5x5/6x6/7x7/ARUCO_ORIGINAL 都试）
  * 只有"机械臂静止 + 板子静止 + 姿态和已有样本差别够大"时才采，避免采废样
  * 采够后自动求解，打印外参、残差，并写成 easy_handeye2 的 .calib
"""
import argparse
import time
from collections import deque

import cv2
import numpy as np
import rclpy
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import StaticTransformBroadcaster

from handeye_solve import pairs_from_samples, solve_ax_xb

DICTS = ['DICT_4X4_50', 'DICT_5X5_50', 'DICT_6X6_50', 'DICT_6X6_250', 'DICT_7X7_50',
         'DICT_ARUCO_ORIGINAL', 'DICT_4X4_100', 'DICT_5X5_100', 'DICT_7X7_100']


def to_h(R, t):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(t).reshape(3)
    return T


def rot_angle_deg(R):
    return float(np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1))))


class Sampler(Node):
    def __init__(self, args):
        super().__init__('handeye_auto')
        self.args = args
        self.bridge = CvBridge()
        self.img = None
        self.K = None
        self.D = None
        self.dict_name = args.dict if args.dict != 'auto' else None
        self.detector = None
        self.marker_id = None
        self._candidate = None
        self._candidate_hits = 0
        self.samples = []
        self.history = deque(maxlen=30)          # (t, T_base_ee, T_cam_marker)
        self.last_status = 0.0
        self.last_sample_time = 0.0
        self.solve_done = False

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=2))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(Image, args.image_topic, self.on_image, 10)
        self.create_subscription(CameraInfo, args.camera_info_topic, self.on_info, 10)
        self.create_timer(0.1, self.tick)

    # ---------- 输入 ----------
    def on_image(self, msg):
        try:
            self.img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception:
            pass

    def on_info(self, msg):
        if msg.k is not None and any(msg.k):
            self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            self.D = np.array(msg.d, dtype=np.float64).reshape(-1, 1)

    def lookup(self, parent, child):
        try:
            t = self.tf_buffer.lookup_transform(parent, child, Time())
        except Exception:
            return None
        q = t.transform.rotation
        tr = t.transform.translation
        # quaternion -> rotation matrix
        x, y, z, w = q.x, q.y, q.z, q.w
        n = float(np.linalg.norm([x, y, z, w]))
        if n < 1e-9:
            return None
        x, y, z, w = x / n, y / n, z / n, w / n
        R = np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
        T = to_h(R, [tr.x, tr.y, tr.z])
        return T if np.all(np.isfinite(T)) else None

    # ---------- 检测 ----------
    MIN_SIDE_PX = 25.0

    def detect(self, gray):
        if self.detector is None:
            # 自动识别字典：连续 5 帧都稳定检出同一个 (字典, id) 才锁定，避免背景误检
            for name in DICTS:
                if not hasattr(cv2.aruco, name):
                    continue
                d = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, name)),
                                            cv2.aruco.DetectorParameters())
                corners, ids, _ = d.detectMarkers(gray)
                if ids is None:
                    continue
                c = corners[0].reshape(-1, 2)
                if np.linalg.norm(c[0] - c[1]) < self.MIN_SIDE_PX:
                    continue
                cand = (name, int(ids.ravel()[0]))
                if cand == self._candidate:
                    self._candidate_hits += 1
                else:
                    self._candidate, self._candidate_hits = cand, 1
                if self._candidate_hits >= 5:
                    self.dict_name, self.marker_id = cand
                    self.detector = d
                    print(f'[aruco] 字典锁定 {name}, id={self.marker_id}', flush=True)
                return None
            self._candidate, self._candidate_hits = None, 0
            return None
        corners, ids, _ = self.detector.detectMarkers(gray)
        if ids is None:
            return None
        idx = 0
        if self.marker_id is not None and self.marker_id in ids.ravel():
            idx = int(np.where(ids.ravel() == self.marker_id)[0][0])
        c = corners[idx].reshape(-1, 2)
        if np.linalg.norm(c[0] - c[1]) < self.MIN_SIDE_PX:
            return None
        c = cv2.cornerSubPix(gray, c.astype(np.float32).reshape(-1, 1, 2), (5, 5), (-1, -1),
                             (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 0.01))
        return c.reshape(-1, 2)

    def marker_pose(self, corners):
        s = self.args.marker_size / 2.0
        obj = np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]], dtype=np.float64)
        ok, rvec, tvec = cv2.solvePnP(obj, corners.astype(np.float64), self.K, self.D,
                                      flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok:
            return None
        R, _ = cv2.Rodrigues(rvec)
        T = to_h(R, tvec.ravel())
        if not np.all(np.isfinite(T)):
            return None
        z = float(T[2, 3])
        if not (0.03 < z < 3.0):      # 板子距离不可能离奇，挡掉误检
            return None
        return T

    # ---------- 主循环 ----------
    def steady(self):
        if len(self.history) < 5:
            return False
        window = [h for h in self.history if h[0] > time.time() - self.args.steady]
        if len(window) < 4:
            return False
        T0_ee, T0_m = window[0][1], window[0][2]
        for _, T_ee, T_m in window[1:]:
            if np.linalg.norm(T_ee[:3, 3] - T0_ee[:3, 3]) > 0.002:
                return False
            if rot_angle_deg(T_ee[:3, :3] @ T0_ee[:3, :3].T) > 0.8:
                return False
            if np.linalg.norm(T_m[:3, 3] - T0_m[:3, 3]) > 0.003:
                return False
            if rot_angle_deg(T_m[:3, :3] @ T0_m[:3, :3].T) > 1.0:
                return False
        return True

    def diverse(self, T_ee):
        for _, T_prev in self.samples:
            dt = np.linalg.norm(T_ee[:3, 3] - T_prev[:3, 3])
            dr = rot_angle_deg(T_ee[:3, :3] @ T_prev[:3, :3].T)
            if not np.isfinite(dt) or not np.isfinite(dr):
                return False
            # 平移差够大、或者姿态差够大，才认为是一个新样本
            if dt < self.args.min_trans and dr < self.args.min_rot:
                return False
        return True

    def status(self, msg):
        now = time.time()
        if now - self.last_status > 3.0:
            self.last_status = now
            print(f'[{time.strftime("%H:%M:%S")}] {msg}', flush=True)

    def tick(self):
        if self.solve_done:
            return
        if self.img is None:
            self.status('等待相机图像 (/image_raw)…')
            return
        if self.K is None:
            self.status('等待相机内参 (/camera_info)…')
            return
        T_ee = self.lookup(self.args.robot_base, self.args.robot_effector)
        gray = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)
        corners = self.detect(gray)
        if corners is None:
            self.history.clear()
            self.status(f'等待板子…（字典 {self.dict_name or "未锁定"}）')
            return
        T_m = self.marker_pose(corners)
        if T_m is None:
            self.status('板子检测到了但 PnP 失败')
            return
        if T_ee is None:
            self.history.clear()
            self.status(f'找不到 tf {self.args.robot_base} -> {self.args.robot_effector}')
            return

        self.history.append((time.time(), T_ee, T_m))

        if not self.steady():
            self.status('板子/机械臂还在动…（稳住等一下）')
            return
        if not self.diverse(T_ee):
            self.status(f'姿态和已有样本太像，换个姿态（已有 {len(self.samples)} 个）')
            return

        self.samples.append((T_ee, T_m))
        dts = [np.linalg.norm(T_ee[:3, 3] - p[0][:3, 3]) for p in self.samples[:-1]]
        drs = [rot_angle_deg(T_ee[:3, :3] @ p[0][:3, :3].T) for p in self.samples[:-1]]
        print(f'[sample {len(self.samples)}/{self.args.samples}] '
              f'最近样本差: {min(dts) * 1000 if dts else 0:.0f} mm / {min(drs) if drs else 0:.1f} deg', flush=True)

        if len(self.samples) >= self.args.samples:
            self.finish()

    # ---------- 求解 ----------
    def finish(self):
        self.solve_done = True
        print('\n=== 开始求解 ===', flush=True)
        X, info = solve_ax_xb(pairs_from_samples(self.samples))
        R, t = X[:3, :3], X[:3, 3]
        q = self.rot_to_quat(R)
        print(f'T_ee_cam 平移 (m): x={t[0]:.6f} y={t[1]:.6f} z={t[2]:.6f}', flush=True)
        print(f'T_ee_cam 四元数: x={q[0]:.6f} y={q[1]:.6f} z={q[2]:.6f} w={q[3]:.6f}', flush=True)
        print(f'板在 base 下的位置漂移(std): {info["translation_std_m"] * 1000:.2f} mm', flush=True)
        print(f'板姿态最大偏差: {info["rotation_spread_deg"]:.2f} deg', flush=True)

        base_marker = info['marker_pose_base']
        print('板在 base 下的平均位姿: x=%.4f y=%.4f z=%.4f' % tuple(base_marker[:3, 3]), flush=True)

        self.save_calib(X)
        if self.args.publish_tf:
            self.publish_tf(X)

    @staticmethod
    def rot_to_quat(R):
        tr = np.trace(R)
        if tr > 0:
            s = np.sqrt(tr + 1.0) * 2
            w = 0.25 * s
            x = (R[2, 1] - R[1, 2]) / s
            y = (R[0, 2] - R[2, 0]) / s
            z = (R[1, 0] - R[0, 1]) / s
        else:
            i = int(np.argmax(np.diag(R)))
            if i == 0:
                s = np.sqrt(1 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
                w = (R[2, 1] - R[1, 2]) / s
                x = 0.25 * s
                y = (R[0, 1] + R[1, 0]) / s
                z = (R[0, 2] + R[2, 0]) / s
            elif i == 1:
                s = np.sqrt(1 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
                w = (R[0, 2] - R[2, 0]) / s
                x = (R[0, 1] + R[1, 0]) / s
                y = 0.25 * s
                z = (R[1, 2] + R[2, 1]) / s
            else:
                s = np.sqrt(1 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
                w = (R[1, 0] - R[0, 1]) / s
                x = (R[0, 2] + R[2, 0]) / s
                y = (R[1, 2] + R[2, 1]) / s
                z = 0.25 * s
        return (x, y, z, w)

    def save_calib(self, X):
        try:
            from easy_handeye2.handeye_calibration import (HandeyeCalibration,
                                                           save_calibration)
            calib = HandeyeCalibration()
            calib.parameters.name = self.args.name
            calib.parameters.calibration_type = 'eye_in_hand'
            calib.parameters.robot_base_frame = self.args.robot_base
            calib.parameters.robot_effector_frame = self.args.robot_effector
            calib.parameters.tracking_base_frame = self.args.camera_frame
            calib.parameters.tracking_marker_frame = self.args.marker_frame
            calib.transform.translation.x = float(X[0, 3])
            calib.transform.translation.y = float(X[1, 3])
            calib.transform.translation.z = float(X[2, 3])
            q = self.rot_to_quat(X[:3, :3])
            calib.transform.rotation.x, calib.transform.rotation.y = float(q[0]), float(q[1])
            calib.transform.rotation.z, calib.transform.rotation.w = float(q[2]), float(q[3])
            path = save_calibration(calib)
            print(f'已写入标定文件: {path}', flush=True)
            print(f'可以用: ros2 launch easy_handeye2 publish.launch.py name:={self.args.name}', flush=True)
        except Exception as e:
            print(f'[warn] 写 .calib 失败: {e}', flush=True)

    def publish_tf(self, X):
        q = self.rot_to_quat(X[:3, :3])
        ts = TransformStamped()
        ts.header.stamp = self.get_clock().now().to_msg()
        ts.header.frame_id = self.args.robot_effector
        ts.child_frame_id = self.args.camera_frame
        ts.transform.translation.x, ts.transform.translation.y, ts.transform.translation.z = map(float, X[:3, 3])
        ts.transform.rotation.x, ts.transform.rotation.y = float(q[0]), float(q[1])
        ts.transform.rotation.z, ts.transform.rotation.w = float(q[2]), float(q[3])
        self.create_static = StaticTransformBroadcaster(self).sendTransform(ts)
        print('已发布静态 TF:', self.args.robot_effector, '->', self.args.camera_frame, flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--samples', type=int, default=18)
    ap.add_argument('--marker-size', type=float, default=0.05, help='黑框外边长，米')
    ap.add_argument('--dict', default='auto')
    ap.add_argument('--image-topic', default='/image_raw')
    ap.add_argument('--camera-info-topic', default='/camera_info')
    ap.add_argument('--robot-base', default='base_link')
    ap.add_argument('--robot-effector', default='link6')
    ap.add_argument('--camera-frame', default='default_cam')
    ap.add_argument('--marker-frame', default='aruco_marker')
    ap.add_argument('--name', default='dm_armx_handeye')
    ap.add_argument('--min-trans', type=float, default=0.03, help='新样本和旧样本的最小平移差(m)')
    ap.add_argument('--min-rot', type=float, default=12.0, help='新样本和旧样本的最小旋转差(deg)')
    ap.add_argument('--steady', type=float, default=0.7, help='静止判定窗口(s)')
    ap.add_argument('--publish-tf', action='store_true')
    args = ap.parse_args()

    rclpy.init()
    node = Sampler(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    except Exception as e:
        print(f'[shutdown] {type(e).__name__}: {e}')
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
