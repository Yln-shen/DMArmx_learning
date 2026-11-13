#!/usr/bin/env python3
"""fit_gravity_params.py —— 用多姿态的 q / 电机保持力矩拟合连杆质量与质心。

数据来源
--------
`gravity_feedforward_test.py --record poses.csv`（位置模式下被动记录，不发力）：
    t, q1..q6, qd1..q6, tau_g1..6, eff1..6

模型
----
重力矩对每根连杆的 (m, m·cx, m·cy, m·cz) 是**线性**的。对每根连杆构造 4 条
“只有这根连杆、质量 1”的基链：

    τ_link = m·T0 + mx·(Tx−T0) + my·(Ty−T0) + mz·(Tz−T0)

把所有连杆的基列拼成 A，用最小二乘解
    A·x ≈ eff（电机保持力矩 ≈ 重力 + 摩擦）
x 就是每根连杆的 [m, mx, my, mz]，com = (mx,my,mz)/m。

注意
----
- 只用低速度行（|qd| < --max-velocity），运动段会引入摩擦/加速度噪声；
- 电机 effort 里有摩擦，残差不可能到 0；拟合结果主要看**预测 τ 的 RMS**；
- 24 个参数在有限姿态下可能秩亏，结果不一定和 URDF 一一对应；加 --ridge 会把解
  往 URDF 先验拉一点；
- 拟合出的质量如果为负，说明数据/姿态覆盖不够或符号有问题（先试 --flip-effort）。
"""

from __future__ import annotations

import argparse
import contextlib
import csv
import io
import json
import math
import os
import sys

import numpy as np
import PyKDL
from urdf_parser_py.urdf import URDF

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gravity_feedforward_test import build_chain  # noqa: E402

FIT_LINKS = ["link2", "link3", "link4", "link5", "link6", "end_link"]
NCOLS = 1 + 6 * 4


def parse_args(argv=None):
    p = argparse.ArgumentParser(description="从多姿态 q/effort 拟合重力模型参数")
    p.add_argument("--csv", nargs="+", required=True, help="--record 生成的 CSV（可多个）")
    p.add_argument("--urdf", default="src/dm_armx_description/urdf/arm.urdf")
    p.add_argument("--base", default="base_link")
    p.add_argument("--tip", default="end_link")
    p.add_argument("--gravity", type=float, default=9.81)
    p.add_argument("--max-velocity", type=float, default=0.05,
                   help="只保留 |qd| 都小于该值的行（rad/s）")
    p.add_argument("--min-rows", type=int, default=10)
    p.add_argument("--home-json", default="log/gravity_home.json",
                   help="collect 脚本写的 home JSON，用来排除 home 附近（可能靠桌）的行")
    p.add_argument("--home-radius", type=float, default=0.02)
    p.add_argument("--cluster-radius", type=float, default=0.015,
                   help="同一姿态正反两遍按 q 聚类平均的半径")
    p.add_argument("--ridge", type=float, default=1e-6,
                   help="往 URDF 先验的岭回归权重；0 = 纯最小二乘")
    p.add_argument("--flip-effort", action="store_true",
                   help="电机 effort 符号与 KDL 相反时取反")
    p.add_argument("--out", default=None, help="把拟合结果写成 JSON")
    return p.parse_args(argv)


def urdf_link_params(path):
    with open(path, "rb") as f:
        xml = f.read()
    with contextlib.redirect_stderr(io.StringIO()):
        robot = URDF.from_xml_string(xml)
    out = {}
    for link in robot.links:
        if link.inertial is None:
            continue
        com = link.inertial.origin.xyz if link.inertial.origin else [0.0, 0.0, 0.0]
        out[link.name] = (link.inertial.mass, list(com))
    return out


def load_rows(paths, max_velocity, flip, home=None, home_radius=0.02):
    rows = []
    for path in paths:
        with open(path, newline="", encoding="utf-8") as f:
            reader = csv.reader(f)
            header = next(reader)
            has_hold = len(header) == NCOLS + 1
            if len(header) not in (NCOLS, NCOLS + 1):
                raise ValueError(f"{path}: 表头 {len(header)} 列，期望 {NCOLS} 或 {NCOLS+1}")
            for raw in reader:
                if len(raw) not in (NCOLS, NCOLS + 1):
                    continue
                vals = [float(v) for v in raw]
                q = vals[1:7]
                qd = vals[7:13]
                eff = vals[19:25]
                hold = vals[25] if len(raw) == NCOLS + 1 else None
                if any(not math.isfinite(v) for v in q + qd + eff):
                    continue
                if hold is not None:
                    if hold < 0.5:
                        continue
                elif max(abs(v) for v in qd) > max_velocity:
                    continue
                if home is not None and max(abs(q[i] - home[i]) for i in range(6)) < home_radius:
                    continue   # home 附近可能靠桌/支撑，排除
                if flip:
                    eff = [-v for v in eff]
                rows.append((q, eff))
    return rows


def cluster_rows(rows, radius=0.015):
    """把同一姿态（正反两遍）的停稳行按 q 聚类，eff 取平均。"""
    groups = []
    for q, eff in rows:
        placed = False
        for g in groups:
            if max(abs(q[i] - g["q"][i]) for i in range(6)) < radius:
                g["members"].append((q, eff))
                placed = True
                break
        if not placed:
            groups.append({"q": list(q), "members": [(q, eff)]})
    out = []
    for g in groups:
        qm = np.mean([m[0] for m in g["members"]], axis=0)
        em = np.mean([m[1] for m in g["members"]], axis=0)
        out.append((list(qm), list(em)))
    return out


def eval_chain(urdf, base, tip, gravity, override, q_rows):
    _, names, dyn = build_chain(
        urdf, base, tip, gravity, inertia_override=override)
    n = len(names)
    out = np.zeros((len(q_rows), n))
    for r, qv in enumerate(q_rows):
        q = PyKDL.JntArray(n)
        tau = PyKDL.JntArray(n)
        for i in range(n):
            q[i] = qv[i]
        if dyn.JntToGravity(q, tau) < 0:
            raise RuntimeError("JntToGravity 失败")
        for i in range(n):
            out[r, i] = tau[i]
    return names, out


def build_regressor(urdf, base, tip, gravity, q_rows):
    zero = PyKDL.RotationalInertia.Zero()
    cols = []
    names = None
    for link in FIT_LINKS:
        names, t0 = eval_chain(
            urdf, base, tip, gravity, {link: (1.0, [0.0, 0.0, 0.0], zero)}, q_rows)
        _, tx = eval_chain(
            urdf, base, tip, gravity, {link: (1.0, [1.0, 0.0, 0.0], zero)}, q_rows)
        _, ty = eval_chain(
            urdf, base, tip, gravity, {link: (1.0, [0.0, 1.0, 0.0], zero)}, q_rows)
        _, tz = eval_chain(
            urdf, base, tip, gravity, {link: (1.0, [0.0, 0.0, 1.0], zero)}, q_rows)
        cols.extend([t0, tx - t0, ty - t0, tz - t0])
    A = np.stack([c.reshape(-1) for c in cols], axis=1)
    return A, names


def main(argv=None):
    args = parse_args(argv)
    urdf_params = urdf_link_params(args.urdf)
    for link in FIT_LINKS:
        if link not in urdf_params:
            print(f"URDF 里没有 {link} 的 inertial", file=sys.stderr)
            return 1

    home = None
    if args.home_json and os.path.exists(args.home_json):
        with open(args.home_json, encoding="utf-8") as f:
            home = json.load(f)["home"]

    rows = load_rows(args.csv, args.max_velocity, args.flip_effort,
                     home, args.home_radius)
    n_raw = len(rows)
    rows = cluster_rows(rows, args.cluster_radius)
    if len(rows) < args.min_rows:
        print(f"有效行 {len(rows)} < {args.min_rows}；多摆几个姿态、每个停稳再记",
              file=sys.stderr)
        return 1

    q_rows = np.array([q for q, _ in rows], dtype=float)
    b = np.array([eff for _, eff in rows], dtype=float).reshape(-1)

    A, names = build_regressor(
        args.urdf, args.base, args.tip, args.gravity, q_rows)
    if names != [f"joint{i}" for i in range(1, 7)]:
        print(f"注意：KDL 关节顺序是 {names}，期望 joint1..joint6", file=sys.stderr)

    # URDF 先验（x0）用于岭回归
    x0 = []
    for link in FIT_LINKS:
        m, com = urdf_params[link]
        x0.extend([m, m * com[0], m * com[1], m * com[2]])
    x0 = np.array(x0)

    if args.ridge > 0.0:
        lam = math.sqrt(args.ridge)
        A_fit = np.vstack([A, lam * np.eye(A.shape[1])])
        b_fit = np.concatenate([b, lam * x0])
    else:
        A_fit, b_fit = A, b

    x, _, rank, _ = np.linalg.lstsq(A_fit, b_fit, rcond=None)
    pred = (A @ x).reshape(-1, 6)
    meas = b.reshape(-1, 6)
    residual = pred - meas
    rms = np.sqrt(np.mean(residual ** 2, axis=0))

    print(f"停稳行 = {n_raw}，聚类后姿态 = {len(rows)}，"
          f"A = {A.shape[0]}x{A.shape[1]}，rank = {rank}")
    print(f"{'link':8s} {'URDF m':>8s} {'fit m':>8s}  "
          f"{'URDF com':>24s}  {'fit com':>24s}")
    for k, link in enumerate(FIT_LINKS):
        m = x[4 * k]
        mx, my, mz = x[4 * k + 1:4 * k + 4]
        com_fit = [mx / m, my / m, mz / m] if abs(m) > 1e-9 else [float("nan")] * 3
        um, ucom = urdf_params[link]
        print(f"{link:8s} {um:8.3f} {m:8.3f}  "
              f"[{ucom[0]:+.4f},{ucom[1]:+.4f},{ucom[2]:+.4f}]  "
              f"[{com_fit[0]:+.4f},{com_fit[1]:+.4f},{com_fit[2]:+.4f}]")
    print("残差 RMS (N·m) per joint = " +
          ", ".join(f"{v:.3f}" for v in rms) +
          f"  | 总 RMS = {math.sqrt(np.mean(residual ** 2)):.3f}")
    if any(x[4 * k] < 0 for k in range(len(FIT_LINKS))):
        print("⚠️ 有负质量：说明姿态覆盖/符号/摩擦让解不唯一；先看预测 RMS，"
              "或加姿态、试 --flip-effort、加 --ridge。", file=sys.stderr)

    if args.out:
        result = {}
        for k, link in enumerate(FIT_LINKS):
            m = float(x[4 * k])
            mx, my, mz = (float(v) for v in x[4 * k + 1:4 * k + 4])
            result[link] = {
                "mass": m,
                "com": [mx / m, my / m, mz / m] if abs(m) > 1e-9 else None,
                "first_moment": [mx, my, mz],
            }
        with open(args.out, "w", encoding="utf-8") as f:
            json.dump(result, f, ensure_ascii=False, indent=2)
        print(f"已写 {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
