#!/usr/bin/env python3
"""AX = XB 手眼标定求解（Park-Martin 旋转 + 最小二乘平移），纯 numpy。

OpenCV 5 的 Python 绑定里已经没有 cv2.calibrateHandEye，这里自己实现。
约定：样本是 (T_base_ee, T_cam_marker)，X = T_ee_cam（相机相对末端的外参），
满足 T_base_marker = T_base_ee @ X @ T_cam_marker（板固定不动）。

用法（自检）：pixi run python src/test/handeye_solve.py
"""
import numpy as np


def _to_h(R, t):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(t).reshape(3)
    return T


def _skew(v):
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])


def _exp_so3(w):
    th = np.linalg.norm(w)
    if th < 1e-12:
        return np.eye(3)
    k = w / th
    K = _skew(k)
    return np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * K @ K


def solve_ax_xb(pairs):
    """pairs: [(A_i, B_i)] 且 A_i @ X = X @ B_i。

    返回 (X(4x4), info dict)。
    """
    if len(pairs) < 2:
        raise ValueError('至少要 2 组样本')

    # R_A R_X = R_X R_B  ->  [(I (x) R_A) - (R_B^T (x) I)] vec_F(R_X) = 0
    # 取所有 pair 堆起来做 SVD，最小奇异值对应的右奇异向量就是 vec(R_X)
    I3 = np.eye(3)
    L = np.zeros((0, 9))
    for A, B in pairs:
        L = np.vstack([L, np.kron(I3, A[:3, :3]) - np.kron(B[:3, :3].T, I3)])
    _, _, Vt = np.linalg.svd(L)

    def project_so3(M):
        U, _, Vt_ = np.linalg.svd(M)
        R_ = U @ Vt_
        if np.linalg.det(R_) < 0:
            U[:, -1] *= -1
            R_ = U @ Vt_
        return R_

    # 零空间向量差一个符号，两种都试，取得旋转方程残差小的那个
    best_err, R = None, None
    for sgn in (1.0, -1.0):
        cand = project_so3((sgn * Vt[-1]).reshape(3, 3, order='F'))
        err = sum(np.linalg.norm(A[:3, :3] @ cand - cand @ B[:3, :3]) for A, B in pairs)
        if best_err is None or err < best_err:
            best_err, R = err, cand

    # (R_A - I) t = R t_B - t_A
    rows, rhs = [], []
    for A, B in pairs:
        rows.append(A[:3, :3] - np.eye(3))
        rhs.append(R @ B[:3, 3] - A[:3, 3])
    t, *_ = np.linalg.lstsq(np.vstack(rows), np.concatenate(rhs), rcond=None)

    X = _to_h(R, t)

    # 残差：用解出的 X 反推板在 base 下的位姿，看离散程度
    base_marker = np.array([p[0] @ X @ np.linalg.inv(p[1]) for p in pairs])
    ts = np.array([T[:3, 3] for T in base_marker])
    t_std = float(np.linalg.norm(ts.std(axis=0)))
    rot_dev = []
    for T in base_marker:
        dR = T[:3, :3] @ base_marker[0][:3, :3].T
        ang = np.degrees(np.arccos(np.clip((np.trace(dR) - 1) / 2, -1, 1)))
        rot_dev.append(ang)
    return X, {'translation_std_m': t_std,
               'rotation_spread_deg': float(np.max(rot_dev)),
               'marker_pose_base': base_marker.mean(axis=0)}


def pairs_from_samples(samples):
    """samples: [(T_base_ee, T_cam_marker)] -> [(A, B)]（所有组合）。"""
    out = []
    for i in range(len(samples)):
        for j in range(i + 1, len(samples)):
            Tee_i, Tcm_i = samples[i]
            Tee_j, Tcm_j = samples[j]
            A = np.linalg.inv(Tee_j) @ Tee_i
            B = Tcm_j @ np.linalg.inv(Tcm_i)
            out.append((A, B))
    return out


if __name__ == '__main__':
    rng = np.random.default_rng(0)
    X_true = _to_h(_exp_so3([0.3, -0.5, 0.8]), [0.03, -0.02, 0.10])
    T_base_marker = _to_h(_exp_so3([0.1, 0.2, -0.1]), [0.4, 0.1, 0.05])

    samples = []
    for _ in range(15):
        T_ee = _to_h(_exp_so3(rng.uniform(-0.8, 0.8, 3)), rng.uniform(-0.2, 0.2, 3))
        T_cm = np.linalg.inv(T_ee @ X_true) @ T_base_marker
        samples.append((T_ee, T_cm))

    X, info = solve_ax_xb(pairs_from_samples(samples))
    print('真实 X:\n', np.round(X_true, 4))
    print('求解 X:\n', np.round(X, 4))
    print('误差 t:', np.round(X[:3, 3] - X_true[:3, 3], 6))
    dR = X[:3, :3] @ X_true[:3, :3].T
    print('误差 rot(deg):', round(float(np.degrees(np.arccos(np.clip((np.trace(dR) - 1) / 2, -1, 1)))), 4))
    print('info:', {k: (np.round(v, 5) if isinstance(v, float) else '...') for k, v in info.items()})
