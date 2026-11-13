#!/usr/bin/env python3
"""控制机械臂扫一圈，用相机找 ArUco 板。

策略：保持其它关节不动，先扫 joint6（腕部滚转），再配 joint5（俯仰）扫几个角度，
每到一处抓几帧试检测；找到就打印关节角并停在那里。

用法: pixi run python src/test/find_marker.py
"""
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

from arm_move import ARM_JOINTS, ArmMover

DICTS = ['DICT_4X4_50', 'DICT_5X5_50', 'DICT_6X6_50', 'DICT_6X6_250', 'DICT_7X7_50',
         'DICT_ARUCO_ORIGINAL', 'DICT_4X4_100', 'DICT_5X5_100', 'DICT_7X7_100']

DETECTORS = []
for name in DICTS:
    if hasattr(cv2.aruco, name):
        d = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, name)),
                                    cv2.aruco.DetectorParameters())
        DETECTORS.append((name, d))


class Finder(ArmMover):
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
            if self.img is not None and n <= 0:
                break
            if self.img is not None:
                n -= 1
            time.sleep(0.1)
        return self.img

    def detect(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        hits = []
        for name, det in DETECTORS:
            corners, ids, _ = det.detectMarkers(gray)
            if ids is None:
                continue
            for c, i in zip(corners, ids.ravel()):
                c = c.reshape(-1, 2)
                side = float(np.linalg.norm(c[0] - c[1]))
                hits.append((name, int(i), side, c.mean(0)))
        return hits


def main():
    rclpy.init()
    node = Finder()
    state = node.wait_state()
    if state is None:
        print('没有 /joint_states')
        return
    base = {j: state[j] for j in ARM_JOINTS}
    print('起始姿态:', {k: round(v, 3) for k, v in base.items()})

    rolls = [0.0, 0.8, -0.8, 1.6, -1.6, 2.4, -2.4, 3.0, -3.0]
    pitches = [base['joint5'], -0.5, 0.6]

    shot = 0
    for pitch in pitches:
        for roll in rolls:
            targets = {'joint5': pitch, 'joint6': roll}
            if not node.move_jtc(targets):
                print(f'移动失败 j5={pitch:+.2f} j6={roll:+.2f}，跳过')
                continue
            img = node.grab()
            if img is None:
                print('没图像')
                continue
            hits = node.detect(img)
            shot += 1
            if hits:
                img2 = img.copy()
                for name, i, side, c in hits:
                    print(f'>>> 找到! j5={pitch:+.2f} j6={roll:+.2f} 字典={name} id={i} '
                          f'边长={side:.0f}px 中心=({c[0]:.0f},{c[1]:.0f})')
                    cv2.circle(img2, tuple(c.astype(int)), 6, (0, 0, 255), 2)
                cv2.imwrite(f'/tmp/found_{shot}.jpg', img2)
                print('标注图: /tmp/found_%d.jpg' % shot)
                return
            print(f'j5={pitch:+.2f} j6={roll:+.2f}: 没看到板子')

    print('扫完了，整个扫描范围里都没找到板子')
    node.move_jtc({k: v for k, v in base.items()})


if __name__ == '__main__':
    main()
