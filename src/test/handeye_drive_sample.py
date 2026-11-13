#!/usr/bin/env python3
"""自动走位 + 采样：joint1 往左摆，joint5/joint6 带角度，每个姿态停稳后采一个样本。

流程：移动(JTC) → 等 1 s 停稳 → 检查 default_cam->aruco_marker 的 TF 是否新鲜 →
      调 /easy_handeye2/calibration/take_sample → 下一个姿态。
板子跑出视野的姿态会自动跳过。

用法:
    pixi run python src/test/handeye_drive_sample.py --samples 18
    pixi run python src/test/handeye_drive_sample.py --samples 18 --dry-run   # 只走位不采样
"""
import argparse
import time

import rclpy
import tf2_ros
from easy_handeye2_msgs.srv import TakeSample
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time

from arm_move import ARM_JOINTS, ArmMover

CANDIDATES = []
for dj1 in (0.0, -0.15, -0.30, -0.45, -0.60, -0.75, -0.90):
    for dj5, dj6 in ((0.0, 0.0), (0.25, 0.5), (-0.25, -0.5), (0.25, -0.5), (-0.25, 0.5)):
        CANDIDATES.append((dj1, dj5, dj6))


class Driver(ArmMover):
    def __init__(self, args):
        super().__init__()
        self.args = args
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=2))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, spin_thread=False)
        self.take = self.create_client(TakeSample, '/easy_handeye2/calibration/take_sample')
        self.list_ = self.create_client(TakeSample, '/easy_handeye2/calibration/get_sample_list')

    def marker_fresh(self, max_age=0.4):
        try:
            t = self.tf_buffer.lookup_transform(self.args.camera_frame, self.args.marker_frame, Time())
        except Exception:
            return False
        stamp = t.header.stamp
        age = (self.get_clock().now().nanoseconds - (stamp.sec * 10**9 + stamp.nanosec)) / 1e9
        return 0.0 <= age <= max_age

    def wait_marker(self, timeout=2.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.marker_fresh():
                return True
        return False

    def _call(self, client, timeout=5.0):
        fut = client.call_async(TakeSample.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        res = fut.result()
        return res

    def sample(self):
        if not self.take.service_is_ready():
            return None
        res = self._call(self.take)
        return None if res is None else len(res.samples.samples)

    def count(self):
        if not self.list_.service_is_ready():
            return None
        res = self._call(self.list_)
        return None if res is None else len(res.samples.samples)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--samples', type=int, default=18)
    ap.add_argument('--settle', type=float, default=1.0)
    ap.add_argument('--camera-frame', default='default_cam')
    ap.add_argument('--marker-frame', default='aruco_marker')
    ap.add_argument('--dry-run', action='store_true')
    args = ap.parse_args()

    rclpy.init()
    node = Driver(args)
    st = node.wait_state()
    if st is None:
        return
    base = {j: st[j] for j in ARM_JOINTS}
    print('基准姿态:', {k: round(v, 3) for k, v in base.items()}, flush=True)

    taken = 0
    for dj1, dj5, dj6 in CANDIDATES:
        if taken >= args.samples:
            break
        targets = {
            'joint1': base['joint1'] + dj1,
            'joint5': base['joint5'] + dj5,
            'joint6': base['joint6'] + dj6,
        }
        if not node.move_jtc(targets):
            print(f'移动失败 {targets}, 跳过', flush=True)
            continue
        time.sleep(args.settle)
        if not node.wait_marker():
            print(f'姿态 j1{dj1:+.2f} j5{dj5:+.2f} j6{dj6:+.2f}: 板子不可见，跳过', flush=True)
            continue
        if args.dry_run:
            print(f'[dry] j1{dj1:+.2f} j5{dj5:+.2f} j6{dj6:+.2f}: 板子可见', flush=True)
            taken += 1
            continue
        n = node.sample()
        if n is None:
            print('采样服务不可用', flush=True)
            break
        taken = n
        print(f'[样本 {n}] j1={targets["joint1"]:+.2f} j5={targets["joint5"]:+.2f} '
              f'j6={targets["joint6"]:+.2f}', flush=True)

    print(f'结束，共 {node.count()} 个样本', flush=True)


if __name__ == '__main__':
    main()
