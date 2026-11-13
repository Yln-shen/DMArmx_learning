#!/usr/bin/env python3
"""抓一帧、检测 ArUco、报告它相对画面中心的位置（板子贴边也能检出）。"""
import cv2
import numpy as np

DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
DET = cv2.aruco.ArucoDetector(DICT, cv2.aruco.DetectorParameters())


def detect(u8, pad=40):
    """u8: 灰度图。返回 (u, v, side, inside) 或 None。

    先直接检测；失败就四周补白边再检测（板子贴边时 quiet zone 被裁掉会检不出）。
    inside=False 表示是靠补边才检到的，说明板子贴边了。
    """
    for inside in (True, False):
        img = u8 if inside else cv2.copyMakeBorder(u8, pad, pad, pad, pad, cv2.BORDER_CONSTANT, value=255)
        corners, ids, _ = DET.detectMarkers(img)
        if ids is None or 0 not in ids.ravel():
            continue
        idx = int(np.where(ids.ravel() == 0)[0][0])
        c = corners[idx].reshape(-1, 2).astype(np.float32)
        if not inside:
            c -= pad
        side = float(np.linalg.norm(c[0] - c[1]))
        if side < 20:
            continue
        u, v = c.mean(axis=0)
        return float(u), float(v), side, inside
    return None
