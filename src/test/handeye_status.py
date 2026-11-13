#!/usr/bin/env python3
"""一边拖机械臂一边看手眼标定"到了什么程度"。

它不采样本、不改任何东西，只做两件事：
  1. 从 easy_handeye2 的 handeye_server 读当前已采的样本（/easy_handeye2/calibration/get_sample_list）
  2. 用这些样本实时重算 T_ee_cam，并把"质量指标"打出来

输出的指标怎么看：
  样本数       ≥3 才能解；15~20 个比较稳
  板位置漂移   同一个板在 base 下反算出来的位置标准差，越小越好（<5 mm 可用）
  板姿态偏差   同上，姿态方向（<2° 可用）
  旋转覆盖     样本之间机器人姿态的最大夹角，越大越好（>60° 才够激励）
  平移跨度     样本之间末端位置的最大距离，>0.1 m 比较好
  相比上次     解出的外参变化量；连续几次都 <2 mm / <1° 就说明收敛了

用法（另开一个终端，标定时一直挂着）:
    pixi run python src/test/handeye_status.py
    pixi run python src/test/handeye_status.py --name dm_armx_handeye
"""
import argparse
import math
import time

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import Transform
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time

from handeye_solve import pairs_from_samples, solve_ax_xb

try:
    from easy_handeye2_msgs.srv import TakeSample
except ImportError:  # 没 source 到 easy_handeye2 时给个清晰提示
    raise SystemExit('找不到 easy_handeye2_msgs，请确认在 pixi 环境里且 realsense_ws 已构建')

GET_SAMPLE_LIST = '/easy_handeye2/calibration/get_sample_list'


def tf_to_h(t: Transform) -> np.ndarray:
    q = t.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    R = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [t.translation.x, t.translation.y, t.translation.z]
    return T


def rot_deg(R):
    return math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(R) - 1) / 2))))


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


class Status(Node):
    def __init__(self, args):
        super().__init__('handeye_status')
        self.args = args
        self.last = None
        self.client = self.create_client(TakeSample, GET_SAMPLE_LIST)
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=2))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.create_timer(2.0, self.tick)

    def tf_ok(self, parent, child):
        try:
            self.tf_buffer.lookup_transform(parent, child, Time())
            return True
        except Exception:
            return False

    def tick(self):
        robot_ok = self.tf_ok(self.args.robot_base, self.args.robot_effector)
        cam_ok = self.tf_ok(self.args.camera_frame, self.args.marker_frame)
        stamp = time.strftime('%H:%M:%S')

        if not self.client.service_is_ready():
            print(f'[{stamp}] 标定服务还没起来（等板子进画面） | TF: 机器人{"✓" if robot_ok else "✗"} '
                  f'板子{"✓" if cam_ok else "✗"}', flush=True)
            return

        try:
            samples = self.client.call(TakeSample.Request()).samples.samples
        except Exception as e:
            print(f'[{stamp}] 读样本失败: {e}', flush=True)
            return

        n = len(samples)
        if n < 3:
            print(f'[{stamp}] 样本 {n} 个（≥3 才能解） | TF: 机器人{"✓" if robot_ok else "✗"} '
                  f'板子{"✓" if cam_ok else "✗"}', flush=True)
            return

        pairs = []
        for s in samples:
            pairs.append((tf_to_h(s.robot), tf_to_h(s.tracking)))
        try:
            X, info = solve_ax_xb(pairs_from_samples(pairs))
        except Exception as e:
            print(f'[{stamp}] 求解失败: {e}', flush=True)
            return

        t = X[:3, 3]
        q = quat_from_R(X[:3, :3])
        rots = [p[0][:3, :3] for p in pairs]
        poss = np.array([p[0][:3, 3] for p in pairs])
        rot_cover = max(rot_deg(rots[i] @ rots[j].T) for i in range(len(rots)) for j in range(i + 1, len(rots)))
        span = float(np.max(np.linalg.norm(poss[:, None, :] - poss[None, :, :], axis=2)))

        line = (f'[{stamp}] 样本 {n} | t=({t[0]:+.3f},{t[1]:+.3f},{t[2]:+.3f}) m  '
                f'q=({q[0]:+.3f},{q[1]:+.3f},{q[2]:+.3f},{q[3]:+.3f})')
        print(line, flush=True)
        print(f'           残差: 板位置 {info["translation_std_m"] * 1000:.1f} mm, '
              f'姿态 {info["rotation_spread_deg"]:.1f}° | '
              f'旋转覆盖 {rot_cover:.0f}°, 平移跨度 {span * 1000:.0f} mm', flush=True)

        if self.last is not None:
            dt = float(np.linalg.norm(t - self.last[0]))
            dr = rot_deg(X[:3, :3] @ self.last[1].T)
            print(f'           相比上次: Δt {dt * 1000:.1f} mm, Δr {dr:.2f}°', flush=True)
        self.last = (t.copy(), X[:3, :3].copy())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--name', default='dm_armx_handeye')
    ap.add_argument('--robot-base', default='base_link')
    ap.add_argument('--robot-effector', default='link6')
    ap.add_argument('--camera-frame', default='default_cam')
    ap.add_argument('--marker-frame', default='aruco_marker')
    args = ap.parse_args()

    rclpy.init()
    node = Status(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception):
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
