#!/usr/bin/env python3
"""把 camera_calibration 补丁到 OpenCV 5 / numpy 2 下能正常用（幂等，可重复执行）。

pixi 里的 ROS 2 Lyrical 带的是 OpenCV 5.0 + numpy 2.x，而 camera_calibration
还是按 OpenCV 4.x 的 API 写的，实测有 3 处会直接抛异常：

1. calibrator.py::_get_corners
   OpenCV 5 的 cv2.findChessboardCorners 返回 (N, 2)，旧代码按 (N, 1, 2) 索引，
   报 "IndexError: too many indices for array"。
   → 统一 reshape 成 (N, 1, 2)，后续所有 CV 4.x 风格的索引就都成立。

2. calibrator.py::_get_corners 的越界检查
   该处用的是 (N, 2) 的取法，和第 1 步的 (N, 1, 2) 不一致，
   会报 "ValueError: The truth value of an array with more than one element is ambiguous"。
   → 改成 corners[i][0][0] / corners[i][0][1]。

3. mono_calibrator.py / stereo_calibrator.py::do_tarfile_save
   ndarray.tostring() 在 numpy >= 2.0 已被删除，点 SAVE 时报 AttributeError。
   → 换成 .tobytes()。

注意：补丁改的是 .pixi/envs/default 里的 site-packages，pixi 升级 / 重装
ros-lyrical-camera-calibration 后会被覆盖，届时重跑一次本脚本即可：

    pixi run python src/test/patch_camera_calibration.py
"""

from pathlib import Path
import sys

try:
    import camera_calibration
except ImportError:
    sys.exit("找不到 camera_calibration，请在 pixi 环境里运行（pixi run python ...）")

PKG = Path(camera_calibration.__file__).parent


def apply(path: Path, old: str, new: str, label: str) -> None:
    text = path.read_text()
    if new in text:
        print(f"[skip] {label}（已经打过）")
        return
    if text.count(old) != 1:
        sys.exit(f"[FAIL] {label}：在 {path} 里找到 {text.count(old)} 处目标代码，预期 1 处")
    path.write_text(text.replace(old, new, 1))
    print(f"[ ok ] {label}")


# 1) OpenCV 5 的 (N, 2) 角点数组统一成 (N, 1, 2)
apply(
    PKG / "calibrator.py",
    """    if not ok:
        return (ok, corners)

    # If any corners are within BORDER pixels""",
    """    if not ok:
        return (ok, corners)

    # OpenCV >= 5.0 returns an (N, 2) array; the rest of this module assumes (N, 1, 2)
    if corners.ndim == 2:
        corners = corners.reshape(-1, 1, 2)

    # If any corners are within BORDER pixels""",
    "calibrator.py: 角点形状 (N,2) -> (N,1,2)",
)

# 2) 越界检查按 (N, 1, 2) 取坐标
apply(
    PKG / "calibrator.py",
    """        [(BORDER < corners[i][0] < (w - BORDER)) and (BORDER < corners[i][1] < (h - BORDER))""",
    """        [(BORDER < corners[i][0][0] < (w - BORDER)) and (BORDER < corners[i][0][1] < (h - BORDER))""",
    "calibrator.py: 边界检查索引",
)

# 3) numpy >= 2.0 删掉了 ndarray.tostring()
for name in ("mono_calibrator.py", "stereo_calibrator.py"):
    apply(
        PKG / name,
        'cv2.imencode(".png", im)[1].tostring()',
        'cv2.imencode(".png", im)[1].tobytes()',
        f"{name}: tostring() -> tobytes()",
    )

print("\n全部补丁就位。重开 cameracalibrator 即可。")
