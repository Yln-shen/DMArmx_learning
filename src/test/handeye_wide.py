#!/usr/bin/env python3
"""大范围自动手眼标定：走位 → 截图确认码在画面里 → 采样。

每个姿态都抓帧检测（相当于自己截图看码在哪）：
  - 码不在画面里 / 贴边 / 太小 → 跳过这个姿态
  - 码在画面里 → 调 /easy_handeye2/calibration/take_sample 采一个样本
跑完自动 compute + save，打印外参和质量指标。
"""
import sys
import time

import cv2
import numpy as np
import rclpy
from easy_handeye2_msgs.srv import ComputeCalibration, SaveCalibration, TakeSample
from rclpy.node import Node

sys.path.insert(0, '/home/ylnn/DMArmx_learning/src/test')
from arm_move import ARM_JOINTS, ArmMover
from marker_probe import detect

POSES = []
for j1 in (-1.60, -1.30, -1.00, -0.70):
    for j5, j6 in ((1.00, -1.0), (1.00, 1.0), (1.30, -0.6),
                   (1.30, 0.0), (1.30, 0.6), (1.15, 0.0)):
        POSES.append((j1, j5, j6))


class Wide(ArmMover):
    def __init__(self):
        super().__init__()
        self.bridge = None
        import cv_bridge
        from sensor_msgs.msg import Image
        self.bridge = cv_bridge.CvBridge()
        self.img = None
        self.create_subscription(Image, '/image_raw', self.on_img, 10)
        self.take = self.create_client(TakeSample, '/easy_handeye2/calibration/take_sample')
        self.comp = self.create_client(ComputeCalibration, '/easy_handeye2/calibration/compute_calibration')
        self.save = self.create_client(SaveCalibration, '/easy_handeye2/calibration/save_calibration')

    def on_img(self, msg):
        try:
            self.img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception:
            pass

    def look(self, timeout=3.0):
        self.img = None
        t0 = time.time()
        while self.img is None and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.img is None:
            return None, None
        return self.img, detect(cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY))

    def call(self, client, req, timeout=10.0):
        f = client.call_async(req)
        rclpy.spin_until_future_complete(self, f, timeout_sec=timeout)
        return f.result()


def main():
    rclpy.init()
    node = Wide()
    st = node.wait_state()
    if st is None:
        return
    print('起始:', {k: round(st[k], 3) for k in ARM_JOINTS}, flush=True)

    # 先回到已知能看见码的区域
    node.move_jtc({'joint1': -1.30, 'joint5': 1.30, 'joint6': 0.0})
    time.sleep(1.0)

    taken = 0
    for i, (j1, j5, j6) in enumerate(POSES, 1):
        if not node.move_jtc({'joint1': j1, 'joint5': j5, 'joint6': j6}):
            print(f'[{i}/{len(POSES)}] 移动失败 j1={j1:+.2f}', flush=True)
            continue
        time.sleep(1.0)
        img, hit = node.look()
        if hit is None:
            print(f'[{i}/{len(POSES)}] j1={j1:+.2f} j5={j5:+.2f} j6={j6:+.2f}: 码不在画面，跳过', flush=True)
            continue
        u, v, side, inside = hit
        if not inside or side < 40:
            print(f'[{i}/{len(POSES)}] j1={j1:+.2f} j5={j5:+.2f} j6={j6:+.2f}: 码贴边/太小 '
                  f'(u={u:.0f} v={v:.0f} side={side:.0f}) 跳过', flush=True)
            continue
        if taken % 5 == 0:
            vis = img.copy()
            cv2.putText(vis, f'u={u:.0f} v={v:.0f} s={side:.0f}', (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
            cv2.imwrite(f'/tmp/handeye_view_{taken:02d}.jpg', vis)
        res = node.call(node.take, TakeSample.Request())
        n = None if res is None else len(res.samples.samples)
        print(f'[{i}/{len(POSES)}] j1={j1:+.2f} j5={j5:+.2f} j6={j6:+.2f}: 码在 ({u:.0f},{v:.0f}) '
              f'side={side:.0f} -> 样本 {n}', flush=True)
        taken = n or taken

    print(f'\n采样结束，共 {taken} 个样本，开始计算…', flush=True)
    res = node.call(node.comp, ComputeCalibration.Request(), timeout=30)
    if res is None or not res.valid:
        print('计算失败/无效', flush=True)
        return
    tr = res.calibration.transform.translation
    q = res.calibration.transform.rotation
    print(f'外参 t=({tr.x:+.6f},{tr.y:+.6f},{tr.z:+.6f}) m', flush=True)
    print(f'     q=({q.x:+.6f},{q.y:+.6f},{q.z:+.6f},{q.w:+.6f})', flush=True)
    r2 = node.call(node.save, SaveCalibration.Request())
    print('保存:', None if r2 is None else (r2.success, r2.filepath.data), flush=True)


if __name__ == '__main__':
    main()
