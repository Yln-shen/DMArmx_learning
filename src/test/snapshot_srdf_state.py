#!/usr/bin/env python3
"""snapshot_srdf_state.py —— 把当前 /joint_states 打印成可粘贴的 SRDF <group_state>。

用途
    设计命名状态时不用手抄 6 个角度：机械臂（手拖/普通控制都行）摆到目标姿态后跑一次，
    把 stdout 贴进 src/dm_armx_config/config/DM_Armx.srdf 即可。

用法
    pixi run python src/test/snapshot_srdf_state.py --name transport
    pixi run python src/test/snapshot_srdf_state.py --name ready --group arm

说明
    - 只订阅 /joint_states，不发任何控制指令、不碰串口；
    - 默认取 joint1..joint6；--joints finger_left 可以单独抓夹爪；
    - 超时（默认 10 s）拿不到完整关节就退出，退出码 1。
"""
from __future__ import annotations

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

DEFAULT_JOINTS = "joint1,joint2,joint3,joint4,joint5,joint6"


class Snapshot(Node):
    def __init__(self, names):
        super().__init__("srdf_state_snapshot")
        self.names = list(names)
        self.msg = None
        self.create_subscription(JointState, "/joint_states", self._cb, 10)

    def _cb(self, msg: JointState) -> None:
        if self.msg is None and set(self.names).issubset(set(msg.name)):
            self.msg = msg


def main() -> int:
    ap = argparse.ArgumentParser(description="把当前关节角打印成 SRDF group_state")
    ap.add_argument("--name", required=True, help="状态名，如 transport / ready / place")
    ap.add_argument("--group", default="arm")
    ap.add_argument("--joints", default=DEFAULT_JOINTS, help="逗号分隔的关节名")
    ap.add_argument("--timeout", type=float, default=10.0, help="等待 /joint_states 的秒数")
    args = ap.parse_args()

    names = [j.strip() for j in args.joints.split(",") if j.strip()]
    if not names:
        print("--joints 不能为空", file=sys.stderr)
        return 2

    rclpy.init()
    node = Snapshot(names)
    try:
        deadline = time.monotonic() + args.timeout
        while node.msg is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)

        if node.msg is None:
            print(f"等不到 /joint_states（需要 {', '.join(names)}）——bringup 起来了吗？",
                  file=sys.stderr)
            return 1

        msg = node.msg
        index = {n: i for i, n in enumerate(msg.name)}
        missing = [n for n in names if n not in index]
        if missing:
            print(f"/joint_states 里没有：{', '.join(missing)}", file=sys.stderr)
            return 1

        print(f'<group_state name="{args.name}" group="{args.group}">')
        for n in names:
            print(f'    <joint name="{n}" value="{msg.position[index[n]]:.5f}"/>')
        print("</group_state>")

        # 夹爪当前值只作参考，不写进 arm 状态
        if "finger_left" in index:
            print(f"[i] 当前 finger_left = {msg.position[index['finger_left']]:.5f} m"
                  "（夹爪状态单独定义，不要写进 arm）", file=sys.stderr)
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
