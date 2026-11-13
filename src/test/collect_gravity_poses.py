#!/usr/bin/env python3
"""collect_gravity_poses.py —— 自动走姿态 + 记录 + 退出前 park 的一条龙采集。

流程
----
1. 可选 `--launch`：自己起 `dm_armx.launch.xml`（位置模式，JTC）；
2. 等 `/joint_states` 和 `/arm_controller/follow_joint_trajectory` 就绪；
3. 记录当前姿态为 **home**（写 `--home-json`）；
4. 以 home 为基准，joint2/joint3/joint4 取小范围组合（joint1 全程不动，满足 ±30° 限制），
   每个目标用 JTC 走 3 s、停 2 s，同时把 q/qd/tau_g/effort 写到 CSV；
5. 结束时（正常/异常/Ctrl-C）：
   - **先只把 joint4 移回 home**（其余保持当前）；
   - **再把其余关节移回 home**；
   - 然后（如果是自己起的 launch）向 launch 发 SIGINT，让硬件 on_deactivate 失能。

注意
----
- JTC 不做碰撞检测，只保证关节限位；运行前工作空间必须清空、手边有急停；
- joint1 保持不变；joint2/joint3 的 URDF 上限是 0.02，所以正方向只取到限位内，
  实际组合以打印出来的目标为准；
- 如果 launch 是你自己起的（`--no-launch`），脚本只负责 park，最后需要你 Ctrl-C launch。
"""

from __future__ import annotations

import argparse
import contextlib
import csv
import io
import json
import math
import os
import signal
import subprocess
import sys
import threading
import time

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from trajectory_msgs.msg import JointTrajectoryPoint

import PyKDL
from urdf_parser_py.urdf import URDF

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gravity_feedforward_test import build_chain  # noqa: E402

JOINT_ORDER = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
NCOLS = 1 + 6 * 4


def parse_args(argv=None):
    p = argparse.ArgumentParser(description="自动走姿态、记录重力数据、退出前 park")
    p.add_argument("--csv", default="log/gravity_poses.csv")
    p.add_argument("--home-json", default="log/gravity_home.json")
    p.add_argument("--urdf", default="src/dm_armx_description/urdf/arm.urdf")
    p.add_argument("--base", default="base_link")
    p.add_argument("--tip", default="end_link")
    p.add_argument("--launch", dest="launch", action="store_true", default=True,
                   help="自己起 dm_armx.launch.xml（默认）")
    p.add_argument("--no-launch", dest="launch", action="store_false",
                   help="attach 到已经起好的 launch，只 park 不停止它")
    p.add_argument("--startup-timeout", type=float, default=60.0)
    p.add_argument("--move-time", type=float, default=3.0)
    p.add_argument("--hold", type=float, default=4.0,
                   help="每个姿态停多久再采（默认 4 s：等固件积分收敛，见 DM_Armx 陷阱 #45）")
    p.add_argument("--poses", default=None,
                   help="姿态表 JSON（每项 6 个数）。离线用 FK 校验过离桌余量的姿态集；"
                        "不给则用内置的 home±delta 小网格")
    p.add_argument("--park-time", type=float, default=3.0)
    p.add_argument("--rate", type=float, default=20.0)
    return p.parse_args(argv)


def load_limits(urdf_path):
    with open(urdf_path, "rb") as f:
        xml = f.read()
    with contextlib.redirect_stderr(io.StringIO()):
        robot = URDF.from_xml_string(xml)
    out = {}
    for j in robot.joints:
        if j.type in ("revolute", "continuous") and j.limit is not None:
            out[j.name] = (j.limit.lower, j.limit.upper)
    return out


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class Collector(Node):
    def __init__(self, args, chain, joint_names, dyn):
        super().__init__("collect_gravity_poses")
        self.args = args
        self.chain = chain
        self.joint_names = joint_names
        self.dyn = dyn
        self.n = len(joint_names)
        self.q = PyKDL.JntArray(self.n)
        self.tau = PyKDL.JntArray(self.n)

        self.positions = {}
        self.velocities = {}
        self.efforts = {}
        self.ready = False
        self.home = None
        self.parked = False
        self.stop = False
        self.executor_dead = False
        self.proc = None
        self.csv_file = None
        self.csv_writer = None
        self.csv_rows = 0
        self.recording_hold = False

        self.limits = load_limits(args.urdf)

        os.makedirs(os.path.dirname(os.path.abspath(args.csv)), exist_ok=True)
        self.csv_file = open(args.csv, "w", newline="", encoding="utf-8")
        self.csv_writer = csv.writer(self.csv_file)
        header = ["t"]
        header += [f"q{i}" for i in range(1, 7)]
        header += [f"qd{i}" for i in range(1, 7)]
        header += [f"tau{i}" for i in range(1, 7)]
        header += [f"eff{i}" for i in range(1, 7)]
        header += ["hold"]
        self.csv_writer.writerow(header)
        self.csv_file.flush()

        self.sub = self.create_subscription(JointState, "/joint_states", self.on_js, 10)
        self.timer = self.create_timer(1.0 / args.rate, self.on_timer)
        self.client = ActionClient(self, FollowJointTrajectory,
                                   "/arm_controller/follow_joint_trajectory")

    # ---------- 数据 ----------
    def on_js(self, msg: JointState):
        for i, name in enumerate(msg.name):
            if name not in self.joint_names:
                continue
            self.positions[name] = msg.position[i]
            if i < len(msg.velocity):
                self.velocities[name] = msg.velocity[i]
            if i < len(msg.effort):
                self.efforts[name] = msg.effort[i]
        self.ready = all(
            n in self.positions and n in self.velocities and n in self.efforts
            for n in self.joint_names)

    def latest(self):
        if not self.ready:
            return None
        return [self.positions[n] for n in self.joint_names]

    def compute_tau(self):
        if not self.ready:
            return None
        for i, n in enumerate(self.joint_names):
            self.q[i] = self.positions[n]
        if self.dyn.JntToGravity(self.q, self.tau) < 0:
            return None
        return [self.tau[i] for i in range(self.n)]

    def on_timer(self):
        try:
            if not self.ready:
                return
            tau = self.compute_tau()
            if tau is None:
                return
            t = time.time()   # rclpy 的 Time.nanoseconds 是属性，不是方法
            row = [t]
            row += [self.positions[n] for n in self.joint_names]
            row += [self.velocities[n] for n in self.joint_names]
            row += tau
            row += [self.efforts[n] for n in self.joint_names]
            row += [1.0 if self.recording_hold else 0.0]
            self.csv_writer.writerow(row)
            self.csv_rows += 1
            if self.csv_rows % 20 == 0:
                self.csv_file.flush()
        except Exception as exc:   # noqa: BLE001
            print(f"[collect] record error: {exc}", flush=True)

    def close_csv(self):
        if self.csv_file is not None:
            self.csv_file.flush()
            self.csv_file.close()
            self.csv_file = None

    # ---------- launch ----------
    def start_launch(self):
        env = os.environ.copy()
        env.setdefault("ROS_LOG_DIR", os.path.abspath("log/ros"))
        cmd = ["bash", "-c",
               "source install/setup.bash && ros2 launch dm_armx_bringup dm_armx.launch.xml"]
        print("[collect] starting:", " ".join(cmd), flush=True)
        self.proc = subprocess.Popen(
            cmd, cwd=os.getcwd(), env=env, stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT, text=True, bufsize=1,
            start_new_session=True)
        threading.Thread(target=self._pump_launch, daemon=True).start()

    def _pump_launch(self):
        try:
            for line in self.proc.stdout:
                print("[launch] " + line.rstrip(), flush=True)
        except Exception:   # noqa: BLE001
            pass

    def stop_launch(self):
        if self.proc is None or self.proc.poll() is not None:
            return
        print("[collect] park done, stopping launch (SIGINT to process group)", flush=True)
        try:
            pgid = os.getpgid(self.proc.pid)
            os.killpg(pgid, signal.SIGINT)
        except ProcessLookupError:
            return
        try:
            self.proc.wait(timeout=20)
        except subprocess.TimeoutExpired:
            print("[collect] launch did not exit, SIGTERM group", flush=True)
            try:
                os.killpg(pgid, signal.SIGTERM)
            except ProcessLookupError:
                return
            try:
                self.proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(pgid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                self.proc.kill()

    # ---------- JTC ----------
    def _wait_future(self, future, timeout, ignore_stop=False):
        deadline = time.time() + timeout
        while not future.done():
            if self.executor_dead:
                print("[collect] executor dead while waiting action", flush=True)
                return False
            if self.stop and not ignore_stop:
                return False
            if time.time() > deadline:
                print("[collect] action future timeout", flush=True)
                return False
            time.sleep(0.02)
        return True

    def travel_time(self, target, minimum=3.0):
        """按最大关节转角估时长。

        硬件 velocity_limit=0.5 rad/s（xacro），JTC 的 spline 若给得太短，臂会跟不上轨迹
        ⇒ 状态容差 0.3 rad 一超就 ABORT（2026-11-01 实测：joint5 差 0.0004 就失败）。
        留 1.8 倍余量，并给一个下限。
        """
        cur = self.latest()
        if cur is None:
            return minimum
        d = max(abs(t - c) for t, c in zip(target, cur))
        return max(minimum, d / 0.5 * 1.8)

    def move_to(self, target, seconds, ignore_stop=False):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(self.joint_names)
        pt = JointTrajectoryPoint()
        pt.positions = [float(v) for v in target]
        sec = int(seconds)
        pt.time_from_start = Duration(sec=sec, nanosec=int((seconds - sec) * 1e9))
        goal.trajectory.points = [pt]
        for name in self.joint_names:
            tol = JointTolerance()
            tol.name = name
            tol.position = 0.15
            tol.velocity = 0.5
            goal.goal_tolerance.append(tol)
            ptol = JointTolerance()
            ptol.name = name
            ptol.position = 0.30
            ptol.velocity = 0.5
            goal.path_tolerance.append(ptol)
        goal.goal_time_tolerance = Duration(sec=5, nanosec=0)

        send_future = self.client.send_goal_async(goal)
        if not self._wait_future(send_future, 5.0, ignore_stop):
            return False
        handle = send_future.result()
        if handle is None or not handle.accepted:
            print("[collect] goal rejected", flush=True)
            return False
        result_future = handle.get_result_async()
        if not self._wait_future(result_future, seconds + 10.0, ignore_stop):
            return False
        result = result_future.result()
        ok = result is not None and result.status == GoalStatus.STATUS_SUCCEEDED
        if not ok:
            print(f"[collect] goal failed: status={None if result is None else result.status}",
                  flush=True)
        return ok

    # ---------- 流程 ----------
    def wait_system(self):
        deadline = time.time() + self.args.startup_timeout
        while time.time() < deadline:
            if self.stop:
                return False
            if self.proc is not None and self.proc.poll() is not None:
                print("[collect] launch exited early", flush=True)
                return False
            if self.ready and self.client.server_is_ready():
                return True
            time.sleep(0.2)
        print("[collect] timeout waiting for /joint_states or JTC", flush=True)
        return False

    def make_poses(self):
        """joint1 固定；joint2..6 取 3 水平，从中选 20 个 maximin 姿态，再正反各走一遍。"""
        home = self.home
        margin = 0.02
        levels = [
            [0.0, -0.25, -0.5],    # joint2（URDF 上限 0.02，只能往负走）
            [0.0, -0.3, -0.6],     # joint3
            [-0.5, 0.0, 0.5],      # joint4
            [-0.4, 0.0, 0.4],      # joint5
            [-0.6, 0.0, 0.6],      # joint6
        ]
        cands = []
        for a in levels[0]:
            for b in levels[1]:
                for c in levels[2]:
                    for d in levels[3]:
                        for e in levels[4]:
                            t = list(home)
                            for idx, delta in ((1, a), (2, b), (3, c), (4, d), (5, e)):
                                lo, hi = self.limits[self.joint_names[idx]]
                                t[idx] = clamp(home[idx] + delta, lo + margin, hi - margin)
                            cands.append(t)
        c = np.array(cands)
        lo = c.min(axis=0)
        hi = c.max(axis=0)
        span = np.where(hi > lo, hi - lo, 1.0)
        cn = (c - lo) / span
        sel = [int(np.argmin(np.linalg.norm(cn - 0.5, axis=1)))]
        for _ in range(19):
            d = np.min(np.linalg.norm(cn[:, None, :] - cn[None, sel, :], axis=2), axis=1)
            sel.append(int(np.argmax(d)))
        picks = [cands[i] for i in sel]
        print("[collect] 20 poses (joint1 fixed):", flush=True)
        for i, t in enumerate(picks):
            print("  %2d: [%s]" % (i + 1, ", ".join(f"{v:+.3f}" for v in t)), flush=True)
        return picks + list(reversed(picks))

    def run(self):
        if self.args.launch:
            self.start_launch()
        if not self.wait_system():
            return False
        self.home = self.latest()
        if self.home is None:
            return False
        with open(self.args.home_json, "w", encoding="utf-8") as f:
            json.dump({"joint_names": self.joint_names, "home": self.home}, f, indent=2)
        print("[collect] home = [" + ", ".join(f"{v:+.4f}" for v in self.home) + "]",
              flush=True)

        if self.args.poses:
            with open(self.args.poses, encoding="utf-8") as f:
                poses = [list(map(float, q)) for q in json.load(f)]
            print(f"[collect] 载入姿态表 {self.args.poses}：{len(poses)} 个", flush=True)
        else:
            poses = self.make_poses()
        print(f"[collect] {len(poses)} poses:", flush=True)
        for i, t in enumerate(poses):
            print(f"  {i+1:2d}: [" + ", ".join(f"{v:+.3f}" for v in t) + "]", flush=True)

        for i, target in enumerate(poses):
            if self.stop:
                break
            dur = max(self.args.move_time, self.travel_time(target))
            print(f"[collect] pose {i+1}/{len(poses)}  (move {dur:.1f}s)", flush=True)
            if not self.move_to(target, dur):
                break
            self.recording_hold = True
            t0 = time.time()
            while time.time() - t0 < self.args.hold:
                if self.stop:
                    break
                time.sleep(0.05)
            self.recording_hold = False
        return True

    def park(self):
        if self.parked or self.home is None:
            return
        self.parked = True
        cur = self.latest() or list(self.home)
        j4 = self.joint_names.index("joint4")
        target = list(cur)
        target[j4] = self.home[j4]
        print("[collect] park: joint4 -> home", flush=True)
        if not self.move_to(target, max(self.args.park_time, self.travel_time(target)), ignore_stop=True):
            print("[collect] park step1 failed", flush=True)
            return
        print("[collect] park: other joints -> home", flush=True)
        if not self.move_to(list(self.home), max(self.args.park_time, self.travel_time(self.home)), ignore_stop=True):
            print("[collect] park step2 failed", flush=True)
            return
        print("[collect] park done", flush=True)


def main(argv=None):
    args = parse_args(argv)
    try:
        chain, joint_names, dyn = build_chain(args.urdf, args.base, args.tip, 9.81)
    except Exception as exc:   # noqa: BLE001
        print(f"建链失败：{exc}", file=sys.stderr)
        return 1
    if joint_names != JOINT_ORDER:
        print(f"KDL 关节顺序 {joint_names} != {JOINT_ORDER}", file=sys.stderr)
        return 1

    rclpy.init()
    node = Collector(args, chain, joint_names, dyn)
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    def spin_executor():
        try:
            executor.spin()
        except Exception as exc:   # noqa: BLE001
            print(f"[collect] executor stopped: {exc}", flush=True)
            node.executor_dead = True

    spin_thread = threading.Thread(target=spin_executor, daemon=True)
    spin_thread.start()

    def on_signal(signum, frame):
        print("[collect] signal -> stop requested", flush=True)
        node.stop = True

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    ok = False
    try:
        ok = node.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.park()
        except Exception as exc:   # noqa: BLE001
            print(f"[collect] park failed: {exc}", flush=True)
        node.close_csv()
        try:
            node.stop_launch()
        except Exception as exc:   # noqa: BLE001
            print(f"[collect] stop launch failed: {exc}", flush=True)
        executor.shutdown()
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:   # noqa: BLE001
            pass
    print(f"[collect] finished, rows={node.csv_rows}", flush=True)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
