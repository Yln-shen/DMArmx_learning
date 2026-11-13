#!/usr/bin/env python3
"""让 joint1 往一个方向分段扫，边扫边用相机找 ArUco 板。

用法:
    # 俯视顺时针（左摆）= joint1 负方向
    pixi run python src/test/sweep_joint1.py --to -1.2
    # 反方向
    pixi run python src/test/sweep_joint1.py --to 1.2

每走一步会抓 3 帧，用 9 种字典试检测，找到就停在那并打印关节角。
"""
import argparse
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

from arm_move import ARM_JOINTS, ArmMover

DICTS = ['DICT_4X4_50', 'DICT_5X5_50', 'DICT_6X6_50', 'DICT_6X6_250', 'DICT_7X7_50',
         'DICT_ARUCO_ORIGINAL', 'DICT_4X4_100', 'DICT_5X5_100', 'DICT_7X7_100']
DETECTORS = [(n, cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, n)),
                                         cv2.aruco.DetectorParameters()))
             for n in DICTS if hasattr(cv2.aruco, n)]


class Sweeper(ArmMover):
    def __init__(self):
        super().__init__()
        self.bridge = CvBridge()
        self.img = None
        self.create_subscription(Image, '/image_raw', self.on_img, 10)

    def on_img(self, msg):
        try:
            self.img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception:
            pass

    def grab(self, n=3, timeout=3.0):
        self.img = None
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.img is not None:
                n -= 1
                if n <= 0:
                    break
        return self.img

    def detect(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        for name, det in DETECTORS:
            corners, ids, _ = det.detectMarkers(gray)
            if ids is None:
                continue
            for c, i in zip(corners, ids.ravel()):
                c = c.reshape(-1, 2)
                side = float(np.linalg.norm(c[0] - c[1]))
                if side < 25:
                    continue
                return name, int(i), side, c
        return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--joint', default='joint1')
    ap.add_argument('--to', type=float, default=-1.2, help='目标关节角（绝对，rad）')
    ap.add_argument('--step', type=float, default=0.15)
    args = ap.parse_args()

    rclpy.init()
    node = Sweeper()
    st = node.wait_state()
    if st is None:
        print('没有 /joint_states')
        return
    start = st[args.joint]
    print(f'{args.joint} 从 {start:+.3f} 扫到 {args.to:+.3f}，步长 {args.step}')

    n = int(abs(args.to - start) / args.step) + 1
    for k in range(1, n + 1):
        target = start + np.sign(args.to - start) * args.step * k
        if (args.to - start) * (target - start) > (args.to - start) ** 2:
            target = args.to
        if not node.move_jtc({args.joint: float(target)}):
            print(f'移动失败，停在 {args.joint}={target:+.3f}')
            break
        time.sleep(0.4)
        img = node.grab()
        if img is None:
            print(f'{args.joint}={target:+.3f}: 没图像')
            continue
        hit = node.detect(img)
        if hit:
            name, mid, side, c = hit
            print(f'>>> 找到板子! {args.joint}={target:+.3f} 字典={name} id={mid} 边长={side:.0f}px')
            cv2.imwrite('/tmp/sweep_found.jpg', img)
            return
        print(f'{args.joint}={target:+.3f}: 没看到板子')

    print('扫完没找到；把 --to 再往前放，或换 joint5/joint6 调俯仰')


if __name__ == '__main__':
    main()
