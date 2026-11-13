# 手眼补偿法 Runbook（单点平移修正）

> 目的：用**人眼确认的接触点**量出当前手眼标定在抓取点的实际偏差 Δ，写出修正后的
> `T_base_cam`。适合“旋转已经对、只差平移”的情况（本机当前就是：旧解与新解旋转
> 差 0.00°、平移差 26.75 mm）。
>
> 不是全自动：最后“笔尖是否真的在码中心”由人眼确认；相机/桌面晃动会直接进 Δ。

---

## 0. 原理（先看这段，避免循环论证）

当前标定 `C0`，相机 PnP 得到码中心 `p`（相机系）。如果只是让 MoveIt 按 `C0`
把笔尖送到 `C0·p`，记录到的 TCP `q ≈ C0·p`——这对数据天然满足 `q = C0·p`，
用它反解只会把 `C0` 原样解回来，修不了任何东西。

所以必须让 `q` 是**笔尖真正到达码中心时**的 TCP 位置，而“真正到达”由人眼确认。
拿到 Δ 后：

```
Δ = q_真实 − C0·p_marker
C_new = translate(Δ) · C0        （只改平移，旋转沿用 C0）
```

一个点只能估 3 个平移自由度；旋转/尺度误差需要 ≥3 个不共线点。本机误差主要是
常值平移，所以单点够用；换一个码位置后如果残差又变大，说明还有别的误差分量。

---

## 1. 前置条件

| 项 | 要求 |
|---|---|
| 控制模式 | `dm_armx.launch.xml`（MoveIt + `arm_controller`，pos_vel） |
| IK / JTC | `/compute_ik` 服务、`/arm_controller/follow_joint_trajectory` 动作 |
| 相机 | `/image_raw`、`/camera_info`，帧名 `camera_optical_frame` |
| 标定 | `dm_armx_handeye_fixed`（或你要修的 C） |
| TCP | `log/handeye/tcp.json`（笔尖） |
| 码 | id0 平贴桌面，在相机视野和臂可达范围内 |
| 笔 | 胶带绑紧，顶抓姿态；TCP 的常值误差会被 Δ 一起吸收 |

启动与检查：

```bash
cd /home/ylnn/DMArmx_learning
pixi run ros2 launch dm_armx_bringup dm_armx.launch.xml

# 三项都正常再继续
ros2 control list_controllers                       # arm_controller active
ros2 run tf2_ros tf2_echo base_link link6           # 连续输出
ros2 service list | grep compute_ik                 # /compute_ik
```

---

## 2. 运行步骤

### Step 1：只识别，不动臂

```bash
pixi run python /home/ylnn/realsense_ws/scripts/handeye/marker_pick_probe.py \
  --dry-run --calib dm_armx_handeye_fixed
```

看三样：`码中心(base)`、`法向 n`、`抓取/悬停位姿`。`中心抖动` 应 <1 mm；
`side` 建议 ≥60 px（太小就把码挪近）。

### Step 2：到码上方指定高度（会存目标缓存）

```bash
pixi run python /home/ylnn/realsense_ws/scripts/handeye/marker_pick_probe.py \
  --calib dm_armx_handeye_fixed \
  --move-only --approach 0.01 --speed 5
```

- `--approach 0.01`：笔尖停在码中心上方 **1 cm**；
- 到位后会打印 `到位 hover ... 误差 x.x mm`（脚本会等位姿稳定再读，误差 >5 mm 会警告）；
- 同时写 `log/handeye/pick_target.json`（码中心、目标位姿、t_tip、calib 名）。

> 到 1 cm 后笔会挡住码，之后**不要再重新识别**；用 Step 3 的 `--jog-only`。

### Step 3：进键盘微调（单独终端，需要 TTY）

```bash
pixi run python /home/ylnn/realsense_ws/scripts/handeye/marker_pick_probe.py \
  --jog-only --out-dir /home/ylnn/DMArmx_learning/log/handeye
```

键位（码面坐标系，z 朝上）：

| 键 | 动作 |
|---|---|
| `a` / `d` | −x / +x |
| `w` / `s` | +y / −y |
| `q` / `e` | +z / −z（下降） |
| `[` / `]` | 步长减半 / 加倍（0.1–2 mm） |
| `g` | 直接到抓取点（笔尖=码中心） |
| `h` | 回 1 cm 悬停点 |
| `r` | 记录当前偏差 Δ，写修正标定 |
| `ESC` | 退出 |

操作：先用 `[` 把步长降到 0.5 mm，`a/d/w/s` 把笔尖对到码中心，`e` 慢慢下降直到
笔尖轻触桌面，停稳，按 `r`。

### Step 4：记录

按 `r` 后输出：

```
记录：笔尖=(...)
Δ base   = (+x.x, +y.y, +z.z) mm
Δ 码面系 = (...)
修正后标定写入 ~/.ros2/easy_handeye2/calibrations/dm_armx_handeye_fixed_corrected.calib
报告写入 log/handeye/pick_probe.json
```

### Step 5：验证

先把臂移开（让相机看到码），再跑：

```bash
pixi run python /home/ylnn/realsense_ws/scripts/handeye/marker_pick_probe.py \
  --dry-run --calib dm_armx_handeye_fixed_corrected
```

或用新标定重新跑 probe（需要臂移开、码可见）：

```bash
pixi run python /home/ylnn/realsense_ws/scripts/handeye/handdrag_qc.py probe \
  --calib dm_armx_handeye_fixed_corrected \
  --tcp log/handeye/tcp.json --val-id 0 --marker-size 0.096 \
  --min-side 65 --out log/handeye/probe_results_corrected.json
```

判据：同一个抓取点，修正后应接近 0；换一个码位置再测，才能看出修正是否全局成立。

---

## 3. 参数表（`marker_pick_probe.py`）

| 参数 | 含义 | 默认 |
|---|---|---|
| `--calib` | 起始手眼标定名 | `dm_armx_handeye_fixed` |
| `--tcp` | 笔尖 TCP JSON | `log/handeye/tcp.json` |
| `--marker-id` | 码 id | 0 |
| `--marker-size` | 黑方块实测边长（m） | 0.096 |
| `--dict` | 字典 | `DICT_4X4_50` |
| `--base-frame` | 基座帧 | `base_link` |
| `--ee-frame` | IK/解算的末端帧 | `link6` |
| `--group` | MoveIt 组名 | `arm` |
| `--approach` | 悬停高度（沿码面法向，m） | 0.03 |
| `--step` | 键盘初始步长（m） | 0.0005 |
| `--speed` | JTC 到位时间（s） | 2.0 |
| `--dry-run` | 只识别+打印，不动臂 | 关 |
| `--move-only` | IK+JTC 到悬停点后退出，并存目标缓存 | 关 |
| `--jog-only` | 读 `pick_target.json` 直接进键盘微调；第一次按键从**当前位姿**算，不跳回缓存点 | 关 |
| `--out-dir` | 缓存/报告目录 | `log/handeye` |

脚本内部：

- 码中心：`T_base_marker = C · T_cam_marker`；
- 工具姿态：`z_tool = −n`（笔朝下），`x_tool` 取码面 x 轴并正交化；
- 抓取位姿：`p_tool = p_marker − R_tool · t_tip`（使笔尖落在码中心）；
- 悬停位姿：`p_hover = p_grasp + approach · n`；
- IK：`/compute_ik`，`ik_link_name=link6`，`avoid_collisions=False`；
- 运动：`/arm_controller/follow_joint_trajectory`，单点轨迹；
- 到位判定：等位姿连续 0.5 s 变化 <0.5 mm 再读（`arm_controller` 没配 constraints，JTC 不校验误差）。

---

## 4. 产物

| 文件 | 内容 |
|---|---|
| `log/handeye/pick_target.json` | 码中心、`T_bm`、抓取/悬停位姿、t_tip、calib 名 |
| `log/handeye/pick_probe.json` | Δ（base/码面）、C_old、C_new、码中心、t_tip |
| `~/.ros2/easy_handeye2/calibrations/dm_armx_handeye_fixed_corrected.calib` | 修正后的标定 |
| `log/handeye/T_base_cam_fixed_corrected.yaml` | 用 `dump` 导出（可选） |

导出 4×4 / 四元数 / static TF：

```bash
pixi run python /home/ylnn/realsense_ws/scripts/handeye/handdrag_qc.py dump \
  --name dm_armx_handeye_fixed_corrected --out log/handeye/T_base_cam_fixed_corrected.yaml
```

---

## 5. 误差与限制

- **一个点只修平移**：`C_new = translate(Δ)·C`，旋转不变。若换位置后残差变大，需要
  至少 3 个不共线点做刚体/直接重解。
- **TCP 误差会被吸收**：如果后续都保持同一个顶抓姿态，笔尖 TCP 的常值误差会进入 Δ；
  换工具/换姿态后要重测。
- **相机/桌面晃动**：脚本用 10 帧平均并打印 `中心抖动`；晃动 >1 mm 时 Δ 本身就有
  1 mm 级噪声。多按几次 `r` 取平均更稳。
- **JTC 不校验到位**：`arm_controller` 没有 constraints，走完轨迹就报 success。脚本
  自己等稳定并打印误差，>5 mm 要停下来查限位/跟踪。

---

## 6. 故障排查

| 现象 | 原因 | 处理 |
|---|---|---|
| `没检出 id0` | 笔挡住码 / 码不在视野 / 光照 | 先把臂退回折叠位（`--move-only` 之前的状态），或用手动 JTC 退开 |
| 到位误差几十 mm | JTC 刚结束时读到旧 TF | 已改成等稳定再读；仍大则查关节限位/跟踪 |
| `IK 失败` | 目标不可达/姿态超出限位 | 把 `--approach` 调大、或换一个码位置 |
| `/compute_ik` 不存在 | 没起 `dm_armx`（当前是 `test_mit`） | 切到 `dm_armx.launch.xml` |
| 串口变成 `ttyACM1` | USB 重新枚举 | 用 by-id；重起栈；必要时直插 USB 口 |
| `ERR=13` 通讯丢失 | 锁存错误 | **断电再上电**，软件清不掉 |
| `usb_cam` 打不开 by-id | 0.8.1 把 `../../videoN` 拼错 | `CAM=$(readlink -f /dev/v4l/by-id/...-video-index0)` 再传 |
| 夹爪读数 `-0.0525` | 单圈绝对编码器差整圈 | 已加 2π 展开；看启动日志 `2π 展开 ...` |
| joint6 超 ±3.14 | 姿态压到限位 | 手动摆回范围内再上电/起 hover |

---

## 7. 本次会话的当前状态

- `dm_armx.launch.xml` 在跑，`arm_controller` / `gripper_controller` active；
- `usb_cam` 在跑，码中心 PnP 抖动 ~0.2 mm；
- 臂已在**码上方 1 cm** 悬停，FK/TF 确认为 `(-0.0425, -0.2889, 0.2930)`；
- `log/handeye/pick_target.json` 已生成。

接下来你只需要在自己的终端跑：

```bash
cd /home/ylnn/DMArmx_learning
pixi run python /home/ylnn/realsense_ws/scripts/handeye/marker_pick_probe.py \
  --jog-only --out-dir /home/ylnn/DMArmx_learning/log/handeye
```

然后用 `a/d/w/s/q/e` 微调、`r` 记录、`ESC` 退出。
