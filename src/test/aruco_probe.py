#!/usr/bin/env python3
"""抓一帧 /image_raw，用各个 ArUco 字典试检测，打印结果并存一张标注图。

用法: pixi run python src/test/aruco_probe.py [输出图片路径]
"""
import sys
import time
import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

DICTS = ['DICT_4X4_50', 'DICT_4X4_100', 'DICT_4X4_250', 'DICT_5X5_50', 'DICT_5X5_100',
         'DICT_5X5_250', 'DICT_6X6_50', 'DICT_6X6_100', 'DICT_6X6_250', 'DICT_7X7_50',
         'DICT_7X7_100', 'DICT_7X7_250', 'DICT_ARUCO_ORIGINAL', 'DICT_APRILTAG_36h11']

out = sys.argv[1] if len(sys.argv) > 1 else '/tmp/aruco_probe.jpg'

rclpy.init()
node = Node('aruco_probe')
bridge = CvBridge()
frame = {}
node.create_subscription(Image, '/image_raw', lambda m: frame.setdefault('img', bridge.imgmsg_to_cv2(m, 'bgr8')), 10)

t0 = time.time()
while not frame and time.time() - t0 < 10:
    rclpy.spin_once(node, timeout_sec=0.1)

if not frame:
    print('没有收到 /image_raw，相机节点没跑？')
    sys.exit(1)

img = frame['img']
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
found = []
for name in DICTS:
    if not hasattr(cv2.aruco, name):
        continue
    det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, name)),
                                  cv2.aruco.DetectorParameters())
    corners, ids, _ = det.detectMarkers(gray)
    if ids is None:
        continue
    for c, i in zip(corners, ids.ravel()):
        c = c.reshape(-1, 2)
        found.append((name, int(i), float(cv2.norm(c[0], c[1]))))
        cv2.polylines(img, [c.astype(int)], True, (0, 0, 255), 2)
        cv2.putText(img, f'{name} id={i}', tuple(c.mean(0).astype(int)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

if found:
    for name, i, px in sorted(found):
        print(f'检测到: 字典={name} id={i} 边长≈{px:.1f}px')
else:
    print('所有字典都没检测到标记 —— 板子要么没完整进画面，要么太小/太糊/反光')
cv2.imwrite(out, img)
print('标注图已存:', out)
