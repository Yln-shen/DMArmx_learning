#!/usr/bin/env python3
"""fit_gravity_offsets.py —— 联合拟合 24 个质量/质心参数 + 6 个关节零位偏移。

模型
----
τ_g(q) ≈ A_mass(q + dq) · x_mass，x_mass = 每根连杆的 [m, m·cx, m·cy, m·cz]，
dq 是关节零位偏移（模型角 = 实测角 + dq）。

对固定的 dq，质量部分仍是线性的；dq 部分用数值雅可比。每轮解
    [J_mass, J_off] · [dx_mass, ddq] ≈ -r
的 Gauss-Newton（带 URDF 先验 ridge、dq 先验 0、dq 限幅）。joint1 的重力矩恒为 0，
其 dq 不可辨识，ridge 会把它留在 0。

用法
----
    pixi run python src/test/fit_gravity_offsets.py \
      --csv log/gravity_poses2.csv --home-json log/gravity_home.json \
      --init log/gravity_fit2b.json --out log/gravity_fit_offsets.json

输出每个连杆的质量/质心、6 个关节的 dq（rad/deg）、预测残差 RMS，以及 JSON。
注意：dq 是“重力模型角”的修正，不需要改 URDF 运动学；可以在前馈脚本/控制器里
      用 q_model = q_measured + dq 计算 τ_g。
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from fit_gravity_params import (  # noqa: E402
    FIT_LINKS,
    build_regressor,
    cluster_rows,
    load_rows,
    urdf_link_params,
)


def parse_args(argv=None):
    p = argparse.ArgumentParser(description="联合拟合重力参数与关节零位")
    p.add_argument("--csv", nargs="+", default=["log/gravity_poses2.csv"])
    p.add_argument("--urdf", default="src/dm_armx_description/urdf/arm.urdf")
    p.add_argument("--base", default="base_link")
    p.add_argument("--tip", default="end_link")
    p.add_argument("--gravity", type=float, default=9.81)
    p.add_argument("--home-json", default="log/gravity_home.json")
    p.add_argument("--home-radius", type=float, default=0.02)
    p.add_argument("--cluster-radius", type=float, default=0.06)
    p.add_argument("--max-velocity", type=float, default=0.05)
    p.add_argument("--ridge-mass", type=float, default=0.01)
    p.add_argument("--ridge-offset", type=float, default=0.01)
    p.add_argument("--max-iter", type=int, default=12)
    p.add_argument("--tol", type=float, default=1e-4)
    p.add_argument("--delta", type=float, default=1e-3, help="数值雅可比步长 rad")
    p.add_argument("--init", default="log/gravity_fit2b.json",
                   help="用之前 24 参数拟合的 JSON 作为初值（可选）")
    p.add_argument("--flip-effort", action="store_true")
    p.add_argument("--out", default="log/gravity_fit_offsets.json")
    return p.parse_args(argv)


def urdf_prior(urdf_path):
    params = urdf_link_params(urdf_path)
    x = []
    for link in FIT_LINKS:
        m, com = params[link]
        x.extend([m, m * com[0], m * com[1], m * com[2]])
    return np.array(x)


def load_init(path, prior):
    if not path or not os.path.exists(path):
        return prior.copy()
    with open(path, encoding="utf-8") as f:
        data = json.load(f)
    x = []
    for link in FIT_LINKS:
        d = data.get(link)
        if d is None or d.get("mass") is None:
            return prior.copy()
        m = float(d["mass"])
        com = d.get("com") or [0.0, 0.0, 0.0]
        x.extend([m, m * com[0], m * com[1], m * com[2]])
    return np.array(x)


def main(argv=None):
    args = parse_args(argv)
    home = None
    if args.home_json and os.path.exists(args.home_json):
        with open(args.home_json, encoding="utf-8") as f:
            home = json.load(f)["home"]

    rows = load_rows(args.csv, args.max_velocity, args.flip_effort,
                     home, args.home_radius)
    if len(rows) < 10:
        print(f"有效行 {len(rows)} < 10", file=sys.stderr)
        return 1

    q = np.array([r[0] for r in rows], dtype=float)
    eff = np.array([r[1] for r in rows], dtype=float)
    n = q.shape[0]

    prior = urdf_prior(args.urdf)
    x = load_init(args.init, prior)
    dq = np.zeros(6)
    prior_full = np.concatenate([prior, np.zeros(6)])
    lam = np.diag(np.concatenate([
        np.full(24, args.ridge_mass), np.full(6, args.ridge_offset)]))

    print(f"姿态数 = {n}，参数 = 24 质量/质心 + 6 零位，"
          f"ridge = {args.ridge_mass}/{args.ridge_offset}", flush=True)

    pred = None
    for it in range(args.max_iter):
        A, _ = build_regressor(
            args.urdf, args.base, args.tip, args.gravity, q + dq)
        pred = (A @ x).reshape(n, 6)
        r = (pred - eff).reshape(-1)

        # dq 的数值雅可比：对当前模型预测求 ∂/∂q
        A_off = np.zeros((n * 6, 6))
        for j in range(6):
            dq1 = dq.copy()
            dq1[j] += args.delta
            A1, _ = build_regressor(
                args.urdf, args.base, args.tip, args.gravity, q + dq1)
            dq2 = dq.copy()
            dq2[j] -= args.delta
            A2, _ = build_regressor(
                args.urdf, args.base, args.tip, args.gravity, q + dq2)
            A_off[:, j] = ((A1 @ x) - (A2 @ x)).reshape(-1) / (2.0 * args.delta)

        J = np.hstack([A, A_off])
        full = np.concatenate([x, dq])
        dx = np.linalg.solve(
            J.T @ J + lam,
            -(J.T @ r + lam @ (full - prior_full)))
        dx = np.clip(dx, -0.5, 0.5)
        x += dx[:24]
        dq += dx[24:]
        dq = np.clip(dq, -0.6, 0.6)
        rms = math.sqrt(float(np.mean(r ** 2)))
        print(f"iter {it:2d}: rms = {rms:.4f}  dq(deg) = "
              + ", ".join(f"{math.degrees(v):+.2f}" for v in dq), flush=True)
        if np.max(np.abs(dx)) < args.tol:
            break

    A, _ = build_regressor(
        args.urdf, args.base, args.tip, args.gravity, q + dq)
    pred = (A @ x).reshape(n, 6)
    r = pred - eff
    rms = np.sqrt(np.mean(r ** 2, axis=0))
    print(f"\n最终：{n} 个姿态，总 RMS = {math.sqrt(np.mean(r**2)):.4f} N·m")
    print(f"{'link':8s} {'URDF m':>8s} {'fit m':>8s}  "
          f"{'URDF com':>26s}  {'fit com':>26s}")
    for k, link in enumerate(FIT_LINKS):
        m = x[4 * k]
        mx, my, mz = x[4 * k + 1:4 * k + 4]
        com = [mx / m, my / m, mz / m] if abs(m) > 1e-9 else [float("nan")] * 3
        um, ucom = urdf_link_params(args.urdf)[link]
        print(f"{link:8s} {um:8.3f} {m:8.3f}  "
              f"[{ucom[0]:+.4f},{ucom[1]:+.4f},{ucom[2]:+.4f}]  "
              f"[{com[0]:+.4f},{com[1]:+.4f},{com[2]:+.4f}]")
    print("残差 RMS per joint = " + ", ".join(f"{v:.3f}" for v in rms))
    print("零位偏移 dq = [" + ", ".join(f"{v:+.4f} rad ({math.degrees(v):+.2f}°)"
                                       for v in dq) + "]")

    result = {"links": {}, "dq_rad": list(map(float, dq)),
              "joint_names": ["joint1", "joint2", "joint3",
                              "joint4", "joint5", "joint6"],
              "rms_per_joint": list(map(float, rms)),
              "rms_total": float(math.sqrt(np.mean(r ** 2)))}
    for k, link in enumerate(FIT_LINKS):
        m = float(x[4 * k])
        mx, my, mz = (float(v) for v in x[4 * k + 1:4 * k + 4])
        result["links"][link] = {
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
