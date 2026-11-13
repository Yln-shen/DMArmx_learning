#!/usr/bin/env python3
"""修 easy_handeye2 装到 install/ 里的 rqt 脚本的 shebang（幂等，可重复执行）。

现象：`ros2 run easy_handeye2 rqt_calibrator.py` 直接报
    FileNotFoundError: [Errno 2] No such file or directory: '.../rqt_calibrator.py'
（文件明明存在）。原因是 easy_handeye2 用 setup.py 的 scripts=[] 安装这两个脚本，
setuptools 把首行 shebang 写成了占位符 `#!python`（不是合法解释器路径），
内核 exec 时就找不到解释器。

修法：把首行换成当前 pixi 环境里 python 的绝对路径。每次 colcon build 之后
这两个文件会被重新覆盖，重跑一次本脚本即可：

    pixi run python src/test/fix_easy_handeye2_scripts.py
"""

from pathlib import Path
import sys

SCRIPTS_DIR = Path("/home/ylnn/realsense_ws/install/easy_handeye2/lib/easy_handeye2")
NAMES = ("rqt_calibrator.py", "rqt_evaluator.py")

python = sys.executable
if not python or not Path(python).exists():
    sys.exit("拿不到当前 python 路径，请在 pixi 环境里运行")
shebang = f"#!{python}\n"

if not SCRIPTS_DIR.is_dir():
    sys.exit(f"找不到 {SCRIPTS_DIR}，先在 ~/realsense_ws 里 colcon build easy_handeye2")

for name in NAMES:
    path = SCRIPTS_DIR / name
    if not path.exists():
        print(f"[skip] {name} 不存在（没 build？）")
        continue
    lines = path.read_text().splitlines(keepends=True)
    first = lines[0] if lines else ""
    if first == shebang:
        print(f"[skip] {name} 已经是对的")
        continue
    lines[0] = shebang
    path.write_text("".join(lines))
    path.chmod(0o775)
    print(f"[ ok ] {name}: {first.strip()} -> {shebang.strip()}")
