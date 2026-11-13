#!/usr/bin/env python3
"""gravity_feedforward_test.py —— 用 URDF 重力项做 MIT 前馈悬浮测试。

前提
----
`test_mit.launch.xml` 已经起来（六轴 CTRL_MODE=1，`forward_effort_controller` active）。
手臂必须有支撑/手扶；先 `--scale 0.3` 确认方向，再逐步加到 0.5 / 0.7 / 1.0。

它做什么
--------
1. 用 `urdf_parser_py` 读 `arm.urdf`，按 `base_link -> end_link` 建 KDL 链；
2. 100 Hz 读 `/joint_states` 的六轴位置，`ChainDynParam::JntToGravity` 算 τ_g；
3. 发布 `sign × scale × gain × τ_g` 到 `/forward_effort_controller/commands`；
4. 启动 1 s 斜坡，Ctrl-C 时 0.5 s 斜坡到 0 再退出。

为什么这样建链
--------------
PyKDL 的 `Joint` 只暴露“平移 origin + axis”的构造，没有 URDF 那种带 rpy 的 origin。
对转动关节用：joint origin = xyz，axis = R(rpy)·axis_urdf，f_tip = R(rpy)：
    T(xyz)·R_{R·a}(q)·R(rpy) = T(xyz)·R(rpy)·R_a(q)
和 URDF 的 `T(xyz)·R(rpy)·R_a(q)` 等价。固定关节则把 xyz+rpy 全放进 f_tip。

注意
----
- 这是**前馈**，没有闭环；URDF 质量比实机大（少装填充件），τ_g 偏大，scale 要试；
- 方向不对（手臂被往下压）就加 `--sign -1`；
- 脚本被强杀时 controller 会保持最后一条命令；Ctrl-C 会先斜坡到 0。
- `--record` = 边发边记；`--record-only` = 纯被动记录（不发力，辨识采集用）。
- `--hold-kp` > 0 时启用"牵绳"保持：弱弹簧把臂拴在锚点上，拖出 `--hold-leash`
  就把锚点弹到当前位置。纯力矩模式没有任何位置约束，残留 0.1 N·m 在 j4（摩擦仅
  0.06）上就足以让它一直滑 —— 这是该机制存在的原因。

用法
----
    # 先离线看 q=0 的 τ_g（不连 ROS/不发力）
    pixi run python src/test/gravity_feedforward_test.py --dry-run

    # 0.3 倍前馈，确认方向
    pixi run python src/test/gravity_feedforward_test.py --scale 0.3

    # 方向反了（往下压）
    pixi run python src/test/gravity_feedforward_test.py --scale 0.3 --sign -1

    # 逐步加
    pixi run python src/test/gravity_feedforward_test.py --scale 0.7
"""

from __future__ import annotations

import argparse
import contextlib
import csv
import io
import logging
import os
import math
import signal
import sys
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

import PyKDL
from urdf_parser_py.urdf import URDF

# urdf_parser_py 会为每个 visual 的 name 属性刷一屏 warning，这里关掉
logging.disable(logging.WARNING)


def build_chain(urdf_path: str, base_link: str, tip_link: str, gravity: float,
                inertia_override=None):
    """URDF -> KDL Chain（只保留重力项需要的信息：质量 + 质心）。

    inertia_override: 可选 {link_name: (mass, [cx,cy,cz], RotationalInertia)}；
    给了就完全替代 URDF 的惯性（辨识时用来构造“只有一根连杆”的基链）。
    """
    with open(urdf_path, "rb") as f:
        urdf_xml = f.read()
    # urdf_parser_py 会为每个 visual 的 name 属性打印一屏 warning（不影响解析）
    with contextlib.redirect_stderr(io.StringIO()):
        robot = URDF.from_xml_string(urdf_xml)

    by_child = {j.child: j for j in robot.joints}
    links = {l.name: l for l in robot.links}

    path = []
    link = tip_link
    while link != base_link:
        if link not in by_child:
            raise ValueError(f"{link} 没有父关节，{base_link}->{tip_link} 不连通")
        joint = by_child[link]
        path.append(joint)
        link = joint.parent
    path.reverse()

    chain = PyKDL.Chain()
    joint_names = []
    for joint in path:
        rpy = joint.origin.rpy if joint.origin else [0.0, 0.0, 0.0]
        xyz = joint.origin.xyz if joint.origin else [0.0, 0.0, 0.0]
        rot = PyKDL.Rotation.RPY(*rpy)

        link_info = links[joint.child]
        if inertia_override is not None:
            mass, com, rot_inertia = inertia_override.get(
                joint.child, (0.0, [0.0, 0.0, 0.0], PyKDL.RotationalInertia.Zero()))
        elif link_info.inertial is not None:
            mass = link_info.inertial.mass
            com = link_info.inertial.origin.xyz if link_info.inertial.origin else [0.0, 0.0, 0.0]
            inertia = link_info.inertial.inertia
            rot_inertia = PyKDL.RotationalInertia(
                inertia.ixx, inertia.iyy, inertia.izz,
                inertia.ixy, inertia.ixz, inertia.iyz,
            ) if inertia is not None else PyKDL.RotationalInertia.Zero()
        else:
            mass, com, rot_inertia = 0.0, [0.0, 0.0, 0.0], PyKDL.RotationalInertia.Zero()

        body_inertia = PyKDL.RigidBodyInertia(mass, PyKDL.Vector(*com), rot_inertia)

        if joint.type in ("revolute", "continuous"):
            axis = rot * PyKDL.Vector(*joint.axis)
            kdl_joint = PyKDL.Joint(
                joint.name, PyKDL.Vector(*xyz), axis, PyKDL.Joint.RotAxis)
            f_tip = PyKDL.Frame(rot)        # 平移在 joint origin，旋转留给 f_tip
            joint_names.append(joint.name)
        elif joint.type == "fixed":
            kdl_joint = PyKDL.Joint(joint.name, PyKDL.Joint.Fixed)
            f_tip = PyKDL.Frame(rot, PyKDL.Vector(*xyz))
        else:
            raise ValueError(f"关节 {joint.name} 类型 {joint.type} 不支持")

        chain.addSegment(PyKDL.Segment(joint.child, kdl_joint, f_tip, body_inertia))

    if chain.getNrOfJoints() != len(joint_names):
        raise RuntimeError(
            f"KDL 链关节数 {chain.getNrOfJoints()} != 转动关节数 {len(joint_names)}")

    dyn = PyKDL.ChainDynParam(chain, PyKDL.Vector(0.0, 0.0, -gravity))
    return chain, joint_names, dyn


class GravityFeedforward(Node):
    def __init__(self, args, chain, joint_names, dyn):
        super().__init__("gravity_feedforward_test")
        self.args = args
        self.chain = chain
        self.joint_names = joint_names
        self.dyn = dyn
        self.n = len(joint_names)

        self.q = PyKDL.JntArray(self.n)
        self.tau = PyKDL.JntArray(self.n)
        self.positions: dict[str, float] = {}
        self.velocities: dict[str, float] = {}
        self.efforts: dict[str, float] = {}
        self.ready = False
        self.gain = 0.0
        self.last_time = time.monotonic()
        self.last_print = 0.0
        self.stop_requested = False
        self.stopping = False

        # 松手保持：静止时把锚点冻结在当前位置，用弱刚度顶住慢漂
        self.anchor = None
        self.holding = False

        self.record_path = args.record or args.record_only
        self.record = bool(self.record_path)
        self.passive = bool(args.record_only)
        self.csv_file = None
        self.csv_writer = None
        self.csv_rows = 0

        # 每轴比例（可带符号）：全局 sign × 全局 scale × per-joint scales
        self.factors = [args.sign * args.scale * v for v in args.scales]

        self.sub = self.create_subscription(JointState, "/joint_states", self.on_joint_state, 10)
        if self.record:
            path = os.path.abspath(self.record_path)
            os.makedirs(os.path.dirname(path), exist_ok=True)
            self.csv_file = open(path, "w", newline="", encoding="utf-8")
            self.csv_writer = csv.writer(self.csv_file)
            header = ["t"]
            header += [f"q{i}" for i in range(1, self.n + 1)]
            header += [f"qd{i}" for i in range(1, self.n + 1)]
            header += [f"tau{i}" for i in range(1, self.n + 1)]
            header += [f"eff{i}" for i in range(1, self.n + 1)]
            self.csv_writer.writerow(header)
            self.tick = 0
            self.record_every = 1 if self.passive else max(1, int(round(args.rate / args.record_rate)))

        if not self.passive:
            self.pub = self.create_publisher(Float64MultiArray, args.topic, 10)

        timer_rate = args.record_rate if self.passive else args.rate
        self.timer = self.create_timer(1.0 / timer_rate, self.on_timer)

        if self.passive:
            self.get_logger().info(
                f"纯被动记录到 {path}（不发力、不建 publisher）；rate={args.record_rate}Hz")
        else:
            self.get_logger().info(
                f"URDF={args.urdf}  joints={joint_names}  "
                f"scale={args.scale} sign={args.sign} per-joint={args.scales}  "
                f"rate={args.rate}Hz"
                + (f"  记录到 {path}" if self.record else ""))

    def on_joint_state(self, msg: JointState):
        for i, name in enumerate(msg.name):
            if name not in self.joint_names:
                continue
            self.positions[name] = msg.position[i]
            if i < len(msg.velocity):
                self.velocities[name] = msg.velocity[i]
            if i < len(msg.effort):
                self.efforts[name] = msg.effort[i]
        self.ready = all(
            name in self.positions and name in self.velocities and name in self.efforts
            for name in self.joint_names)

    def publish(self, values):
        msg = Float64MultiArray()
        msg.data = [float(v) for v in values]
        self.pub.publish(msg)

    def compute_tau(self):
        if not self.ready:
            return None
        for i, name in enumerate(self.joint_names):
            self.q[i] = self.positions[name]
        if self.dyn.JntToGravity(self.q, self.tau) < 0:
            return None
        return [self.tau[i] for i in range(self.n)]

    def on_timer(self):
        if self.stop_requested:
            if not self.stopping:
                if self.passive:
                    self.stop_record()
                else:
                    self.ramp_down_and_stop()
            return
        if self.stopping:
            return
        if self.passive:
            self.record_row()
            return

        now = time.monotonic()
        dt = max(0.0, now - self.last_time)
        self.last_time = now

        tau = self.compute_tau()
        if tau is None:
            self.publish([0.0] * self.n)
            return

        out = [self.factors[i] * self.gain * tau[i] for i in range(self.n)]

        # 松手保持（牵绳）：始终有一个弱弹簧，把臂拴在锚点上；拖出 leash 就把锚点
        # 弹到当前位置（拖动时不吃力）。纯力矩模式（hold_kp=0）没有任何位置约束，
        # 残留 0.1 N·m 在摩擦只有 0.06 的 j4 上就足以让它一直滑 —— 这是本机制存在的原因。
        if self.args.hold_kp > 0.0:
            pos = [self.positions[n] for n in self.joint_names]
            if self.anchor is None:
                self.anchor = list(pos)
            if any(abs(pos[i] - self.anchor[i]) > self.args.hold_leash
                   for i in range(self.n)):
                self.anchor = list(pos)      # 被拖出绳长 ⇒ 认这个新位置
                self.holding = False
            else:
                self.holding = True
            for i in range(self.n):
                v = self.args.hold_kp * (self.anchor[i] - pos[i])
                v = max(-4.0, min(4.0, v))   # 保持力别失控
                out[i] += self.gain * v

        self.publish(out)

        if self.record:
            self.tick += 1
            if self.tick % self.record_every == 0:
                self.record_row()
        self.gain = min(1.0, self.gain + dt / max(self.args.ramp, 1e-6))

        if now - self.last_print >= 1.0:
            self.last_print = now
            q_deg = [math.degrees(self.positions[n]) for n in self.joint_names]
            print(
                "q(deg)=[" + ", ".join(f"{v:+.1f}" for v in q_deg) + "]  "
                "tau_g=[" + ", ".join(f"{v:+.2f}" for v in tau) + "]  "
                f"gain={self.gain:.2f}  "
                + (f"HOLD(kp={self.args.hold_kp:g})" if self.holding else
                   ("free" if self.args.hold_kp > 0.0 else "torque-only")),
                flush=True)

    def record_row(self):
        if not self.ready:
            return
        tau = self.compute_tau()
        if tau is None:
            return
        t = self.get_clock().now().nanoseconds * 1e-9
        row = [t]
        row += [self.positions[n] for n in self.joint_names]
        row += [self.velocities[n] for n in self.joint_names]
        row += tau
        row += [self.efforts[n] for n in self.joint_names]
        self.csv_writer.writerow(row)
        self.csv_rows += 1
        if self.csv_rows % 20 == 0:
            self.csv_file.flush()

    def stop_record(self):
        self.stopping = True
        if self.csv_file is not None:
            self.csv_file.flush()
            self.csv_file.close()
            self.csv_file = None
        if rclpy.ok():
            try:
                self.get_logger().info(f"记录结束，共 {self.csv_rows} 行")
            except Exception:   # noqa: BLE001
                pass
            rclpy.shutdown()

    def ramp_down_and_stop(self):
        """Ctrl-C：把当前输出在 0.5 s 内斜坡到 0，再关 rclpy。"""
        self.stopping = True
        tau = self.compute_tau()
        if tau is None:
            self.publish([0.0] * self.n)
            rclpy.shutdown()
            return

        base = [self.factors[i] * self.gain * tau[i] for i in range(self.n)]
        steps = 10
        for k in range(steps, -1, -1):
            self.publish([v * k / steps for v in base])
            time.sleep(self.args.stop_ramp / steps)
        self.publish([0.0] * self.n)
        if self.record:
            self.stop_record()
        self.get_logger().info("已斜坡到 0，退出")
        rclpy.shutdown()


def parse_args(argv=None):
    p = argparse.ArgumentParser(description="URDF 重力项 MIT 前馈悬浮测试")
    p.add_argument("--urdf", default="src/dm_armx_description/urdf/arm.urdf",
                   help="真机 arm.urdf 路径")
    p.add_argument("--base", default="base_link")
    p.add_argument("--tip", default="end_link")
    p.add_argument("--gravity", type=float, default=9.81)
    p.add_argument("--scale", type=float, default=0.3,
                   help="前馈比例；URDF 质量偏大，先 0.3 再往上加")
    p.add_argument("--sign", type=float, default=1.0,
                   help="全局方向；整条手臂都被往下压时改成 -1")
    p.add_argument("--scales", default="1,1,1,1,1,1",
                   help="每轴比例（可带符号），逗号分隔；只有 joint4 往下压就设 "
                        "第 4 个为 -1（或 -0.5 再压小一点）")
    p.add_argument("--rate", type=float, default=100.0, help="发布频率 Hz")
    p.add_argument("--ramp", type=float, default=1.0, help="启动斜坡秒数")
    p.add_argument("--stop-ramp", type=float, default=0.5, help="退出斜坡秒数")
    p.add_argument("--topic", default="/forward_effort_controller/commands")
    p.add_argument("--record", default=None, metavar="CSV",
                   help="边发前馈边记录 q/qd/tau_g/effort 到 CSV（不改变发力）")
    p.add_argument("--record-only", default=None, metavar="CSV",
                   help="纯被动记录：不建 publisher、不发力。配合位置模式做多姿态辨识用")
    p.add_argument("--record-rate", type=float, default=20.0, help="记录频率 Hz")
    p.add_argument("--hold-kp", type=float, default=0.0,
                   help="松手保持刚度 N·m/rad。0=关闭（纯力矩，会漂）。"
                        "建议 4~6：残留 0.2 N·m 时下垂仅 0.03~0.05 rad")
    p.add_argument("--hold-leash", type=float, default=0.15,
                   help="牵绳长度 rad：偏离锚点超过它就认为有人在拖，锚点弹到当前位置")
    p.add_argument("--dry-run", action="store_true",
                   help="只建链、算 q=0 的 τ_g，不连 ROS、不发力")
    args = p.parse_args(argv)
    try:
        args.scales = [float(x) for x in args.scales.split(",")]
    except ValueError:
        p.error("--scales 必须是逗号分隔的数字，例如 1,1,1,-1,1,1")
    if len(args.scales) != 6:
        p.error("--scales 必须是 6 个数字（joint1..joint6）")
    return args


def main(argv=None):
    args = parse_args(argv)
    try:
        chain, joint_names, dyn = build_chain(
            args.urdf, args.base, args.tip, args.gravity)
    except Exception as exc:   # noqa: BLE001
        print(f"建链失败：{exc}", file=sys.stderr)
        return 1

    print(f"KDL 链：{args.base} -> {args.tip}，{len(joint_names)} 个转动关节：{joint_names}")

    if args.dry_run:
        q = PyKDL.JntArray(len(joint_names))
        tau = PyKDL.JntArray(len(joint_names))
        dyn.JntToGravity(q, tau)
        print("q=0 时 τ_g = [" + ", ".join(f"{tau[i]:+.3f}" for i in range(len(joint_names))) + "] N·m")
        return 0

    rclpy.init()
    node = GravityFeedforward(args, chain, joint_names, dyn)

    def on_signal(signum, frame):
        node.stop_requested = True

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if not node.stopping:
            try:
                if node.record:
                    node.stop_record()
                else:
                    node.ramp_down_and_stop()
            except Exception:   # noqa: BLE001
                pass
        try:
            node.destroy_node()
        except Exception:   # noqa: BLE001
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:   # noqa: BLE001
            pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
