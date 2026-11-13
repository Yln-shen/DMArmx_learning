#!/usr/bin/env python3
"""check_hold_drift.py —— 定点悬停时量一下各关节到底漂了多少。

用途
----
调 `--hold-kp` / `--scales` / `kd` 时，光靠手感很难判断「1°/s 的慢漂」和
「已经停住」。这个脚本只订阅 `/joint_states`、**不发任何命令**，采 N 秒后
给出每个关节的起始角、结束角、漂移速率（°/s）、峰峰值，以及实测 effort
的均值（能看出电机是不是一直在顶着一个固定偏差）。

判读
----
- `rate` 是负的 = 关节往负方向走（j2/j3 通常是「往下沉」）。
- 稳态悬停的目标：10 s 内 |rate| < 0.5 °/s、峰峰值 < 1°；60 s 累积 < 2°。
- `eff` 均值明显非零且稳定 ⇒ 有一个恒定力矩偏差（模型/摩擦），
  这种偏差靠加大 `kd` 只能变慢、不能停住，要靠 `--hold-kp`。
- 采的时候**不要碰手臂**，否则量到的是你的手，不是漂移。

用法
----
    # 悬停中，另开一个终端，量 10 秒
    pixi run python src/test/check_hold_drift.py --duration 10

    # 只看 j4（注意 joint 名是全名 joint4）
    pixi run python src/test/check_hold_drift.py --duration 15 --joints joint4

    # 阈值收紧到 0.2 °/s，超了返回码 1（方便脚本判定）
    pixi run python src/test/check_hold_drift.py --max-rate 0.2
"""

from __future__ import annotations

import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

ARM_JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]


class DriftChecker(Node):
    def __init__(self, joints: list[str]):
        super().__init__("hold_drift_checker")
        self.joints = joints
        self.t0 = None
        self.t_first = None
        self.t_last = None
        self.q: dict[str, list[float]] = {j: [] for j in joints}
        self.eff: dict[str, list[float]] = {j: [] for j in joints}
        self.create_subscription(JointState, "/joint_states", self.on_state, 50)

    def on_state(self, msg: JointState) -> None:
        idx = {name: i for i, name in enumerate(msg.name)}
        if not all(j in idx for j in self.joints):
            return
        now = time.monotonic()
        if self.t0 is None:
            self.t0 = now
            self.t_first = now
        self.t_last = now
        for j in self.joints:
            i = idx[j]
            if i < len(msg.position):
                self.q[j].append(float(msg.position[i]))
            if i < len(msg.effort):
                self.eff[j].append(float(msg.effort[i]))


def stats(values: list[float]) -> tuple[float, float]:
    if not values:
        return 0.0, 0.0
    mean = sum(values) / len(values)
    var = sum((v - mean) ** 2 for v in values) / len(values)
    return mean, math.sqrt(var)


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description="定点悬停漂移测量（只读，不发命令）")
    p.add_argument("--duration", type=float, default=10.0, help="采样秒数")
    p.add_argument("--joints", default=",".join(ARM_JOINTS),
                   help="逗号分隔的关节名，默认六轴全测")
    p.add_argument("--max-rate", type=float, default=0.5,
                   help="判定阈值 °/s；超过就判 FAIL")
    p.add_argument("--min-samples", type=int, default=20,
                   help="少于此样本数视为没数据")
    args = p.parse_args(argv)

    joints = [j.strip() for j in args.joints.split(",") if j.strip()]
    unknown = [j for j in joints if j not in ARM_JOINTS]
    if unknown:
        print(f"未知关节名：{unknown}（应为 {ARM_JOINTS}）", file=sys.stderr)
        return 2

    rclpy.init()
    node = DriftChecker(joints)
    deadline = time.monotonic() + args.duration
    try:
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass

    if not node.q[joints[0]]:
        print("没收到 /joint_states —— 先确认 bringup 在跑、ROS_DOMAIN_ID 一致。",
              file=sys.stderr)
        node.destroy_node()
        rclpy.shutdown()
        return 2

    n = min(len(node.q[j]) for j in joints)
    if n < args.min_samples:
        print(f"样本太少（{n} < {args.min_samples}），加长 --duration。", file=sys.stderr)
        node.destroy_node()
        rclpy.shutdown()
        return 2

    t_span = args.duration
    if node.t_first is not None and node.t_last is not None:
        t_span = max(1e-6, node.t_last - node.t_first)

    print(f"采样 {n} 点 / {args.duration:g} s，关节：{', '.join(joints)}")
    print(f"{'关节':<8}{'起始°':>9}{'结束°':>9}{'漂移°':>9}"
          f"{'速率°/s':>10}{'峰峰°':>8}{'eff均值':>9}{'eff标准差':>10}  判定")

    failed = []
    for j in joints:
        q = node.q[j]
        e = node.eff[j]
        drift = math.degrees(q[-1] - q[0])
        rate = drift / t_span
        pp = math.degrees(max(q) - min(q))
        em, es = stats(e)
        verdict = "OK" if abs(rate) <= args.max_rate else "FAIL"
        if verdict == "FAIL":
            failed.append(j)
        print(f"{j:<8}{math.degrees(q[0]):>9.2f}{math.degrees(q[-1]):>9.2f}"
              f"{drift:>9.2f}{rate:>10.2f}{pp:>8.2f}{em:>9.3f}{es:>10.3f}  {verdict}")

    print()
    if failed:
        print(f"判定：{', '.join(failed)} 超过 {args.max_rate:g} °/s —— "
              "先看 eff 均值是否恒定非零；是则用 --hold-kp，不是则查模型/方向。")
    else:
        print(f"判定：全部在 {args.max_rate:g} °/s 以内。")

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
