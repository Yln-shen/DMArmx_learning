# 笔尖 TCP + 手眼精度验证 Runbook（MIT 悬浮手拖）

> 场景：圆珠笔夹在夹爪上，胶带绑紧；机械臂在重力补偿下用手拖；笔尖碰桌面十字标定 TCP，
> 再碰桌上 id0 码中心验证手眼标定。**全程不向控制器发运动指令**，只读 TF。
>
> 前置：手眼标定已完成（`~/.ros2/easy_handeye2/calibrations/dm_armx_handeye.calib`），
> 末端 id0 板已拆下。

---

## 0. 准备（一次性，5 分钟）

1. **拆掉末端的 id0 板**。它在末端时，验证画面里会出现两个 id0，`probe` 会挑错。
2. **笔**：圆珠笔/中性笔，硬尖；用胶带把笔杆绑在夹爪手指上，**至少绑两处**，笔尖伸出 10–15 cm。
   - 绑完用手轻推笔尖，笔不能转、不能轴向滑。
   - 更稳的做法：把笔绑在夹爪**固定底座**（`end_link`）上，而不是会动的两指上；这样即使指关节微动，笔也不动。
3. **十字**：桌面画好，笔尖能从 8 个以上不同姿态碰到同一点。
4. **验证码**：id0 平贴桌面（最好垫一块硬亚克力，避免笔尖一碰就翘），位置在相机视野内，和十字**不是同一点**。
5. **卡尺量码的黑方块边长**（标称 0.096，打印常有 94–96 mm），后面所有 `--marker-size` 用它。

---

## 1. 起悬浮（两个终端）

### T1：MIT 悬浮

```bash
cd /home/ylnn/DMArmx_learning

PORT=/dev/serial/by-id/usb-HDSC_CDC_Device_00000000050C-if00
[ -e "$PORT" ] && echo "port ok -> $(readlink -f "$PORT")" || echo "PORT MISSING"

ros2 launch dm_armx_bringup test_mit.launch.xml \
  arm_kp:=0.0 arm_kd:=2.5 serial_port:="$PORT"
```

⚠️ 起之前先扶住臂：`arm_kp=0` 没有位置保持，2 秒内必须起 T2，否则臂会靠阻尼垂下。

⚠️ `test_mit` **没有夹爪控制器**。夹爪靠硬件 `on_activate` 时写的那一帧保持（位置=启动瞬间实测位置，力=45 N）。所以笔必须**在起 T1 之前**绑好；起之后再想开合夹爪没有接口。

### T2：重力前馈 + 牵绳

```bash
cd /home/ylnn/DMArmx_learning
pixi run python src/test/gravity_feedforward_test.py \
  --urdf log/arm_gravity2.urdf --scale 1.0 --ramp 3 \
  --hold-kp 6 --hold-leash 0.15
```

- 拖出 0.15 rad 后锚点会弹到当前位置，所以可以拖着走、松手即停。
- 笔只增加十几克，重力模型没含它，残差由牵绳兜住；碰点时用手扶一下更稳。

### 检查

```bash
ros2 run tf2_ros tf2_echo base_link link6          # 应有连续输出
ros2 topic echo /joint_states --once               # finger_left 应稳定，不跳变
```

---

## 2. 标定笔尖 TCP（T3）

```bash
cd /home/ylnn/DMArmx_learning
pixi run python /home/ylnn/realsense_ws/scripts/handeye/handdrag_qc.py tcp \
  --out log/handeye/tcp.json
```

操作：

1. 笔尖对准十字中心，**轻碰**（别把笔压弯），停稳 0.6 s，回车；
2. 抬起，换一个姿态（绕十字倾斜 ±20–30°，或绕笔轴转 90°），再碰同一点，回车；
3. 重复到 **8 个以上**姿态，输入 `q` 求解。

判据：

```
rank=6/6                       ← 少于 6 说明姿态不够多样
残差: 最大 < 2 mm              ← 圆珠笔尖应能到 1 mm 以内
```

输出同时给两个系：

```
t_tip (在 link6 系)    = (...)
t_tip (在 end_link 系) = (...)
```

写入 `log/handeye/tcp.json`（字段 `t_tip`、`t_tip_tool`、`P_base`、`residual_m`、`rank`）。

**残差大怎么办**：

| 现象 | 原因 | 处理 |
|---|---|---|
| 残差随倾斜角变大 | 笔在夹爪里转/滑，或笔尖是软头 | 重新绑；换硬尖 |
| 残差整体大但随机 | 碰点不准 | 十字画细一点，笔尖对准中心 |
| rank < 6 | 8 个姿态太像 | 增加倾斜/绕轴旋转 |

---

## 3. 验证手眼精度

### T4：相机

```bash
cd /home/ylnn/DMArmx_learning
CAM=$(ls /dev/v4l/by-id/*-video-index0 | head -1)
pixi run ros2 run usb_cam usb_cam_node_exe --ros-args \
  -p video_device:=$CAM \
  -p image_width:=640 -p image_height:=480 -p framerate:=30.0 \
  -p pixel_format:=mjpeg2rgb -p io_method:=mmap \
  -p frame_id:=camera_optical_frame -p camera_name:=camera \
  -p camera_info_url:=file:///home/ylnn/DMArmx_learning/ost.yaml \
  -r image_raw:=/image_raw -r camera_info:=/camera_info
```

### T5：验证

```bash
cd /home/ylnn/DMArmx_learning
pixi run python /home/ylnn/realsense_ws/scripts/handeye/handdrag_qc.py probe \
  --calib dm_armx_handeye \
  --tcp log/handeye/tcp.json \
  --val-id 0 \
  --marker-size 0.096 \
  --out log/handeye/probe_results.json
```

每轮两步（**8 个以上姿态**，都碰同一个码中心）：

1. 笔尖碰到 id0 码的**中心**，停稳，回车 → 脚本用 FK + `t_tip` 算 `P_base`；
2. 手拖机械臂移开（别挡码），回车 → 相机 PnP 算码中心，用 `T_base_cam` 投到 base，得 `P_cam`。

输出每个点的 `P_base`、`P_cam`、三轴误差；结束时给 RMS/均值/最大。

判据：

| 指标 | 合格 | 可用 | 不可用 |
|---|---|---|---|
| 单点误差模长 | < 5 mm | < 10 mm | > 15 mm |
| 多点 RMS | < 5 mm | < 10 mm | > 15 mm |
| 误差方向 | 随机 | 某轴系统性偏大 | 全部同向偏 |

**误差全部同向偏**：优先查 `marker_size`、内参、验证码是否贴平；不是 TCP 的问题。
**误差随姿态变**：TCP 没标好（笔在动），回第 2 步重标。

---

## 4. 参数表

### `test_mit.launch.xml`

| 参数 | 含义 | 你的值 | 影响 |
|---|---|---|---|
| `arm_kp` | MIT 刚度 | `0.0` | 0=无力矩位置保持，可手拖 |
| `arm_kd` | MIT 阻尼 | `2.5` | 越大越黏，抑制抖动 |
| `serial_port` | U2CAN 串口 | by-id 路径 | 用变量传，防中文标点 |

### `gravity_feedforward_test.py`

| 参数 | 含义 | 你的值 | 影响 |
|---|---|---|---|
| `--urdf` | 辨识过惯性的模型 | `log/arm_gravity2.urdf` | 用错模型→漂移快 |
| `--scale` | 前馈总增益 | `1.0` | <1 下沉，>1 上飘 |
| `--ramp` | 启动斜坡(s) | `3` | 太短会突然发力 |
| `--hold-kp` | 牵绳刚度 | `6` | 越大越顶，越小越好拖 |
| `--hold-leash` | 锚点重设阈值(rad) | `0.15` | 拖出这么多就认新位置 |
| `--rate` | 发布频率 | 100 | 默认 |
| `--topic` | 力矩话题 | `/forward_effort_controller/commands` | 必须和控制器一致 |

### `handdrag_qc.py tcp`

| 参数 | 含义 | 默认 | 建议 |
|---|---|---|---|
| `--base-frame` | 基座帧 | `base_link` | 不改 |
| `--ee-frame` | 末端帧（解算 t_tip 的系） | `link6` | 不改 |
| `--tool-frame` | 同时输出该工具系偏移 | `end_link` | 你要的 1.3 就是它；空串=不输出 |
| `--out` | 结果 JSON | `log/handeye/tcp.json` | probe 直接读这个 |

内部数学：对每个姿态 `T_base_ee_i · t_tip = P`，8 个姿态给 24 个方程解 3 个未知数；`rank` 是方程组的秩，`residual` 是每点到平均触碰点的距离。

### `handdrag_qc.py probe`

| 参数 | 含义 | 默认 | 建议 |
|---|---|---|---|
| `--calib` | easy_handeye2 标定名 | `dm_armx_handeye` | 你已保存的名字 |
| `--tcp` | 笔尖 TCP JSON | `log/handeye/tcp.json` | 第 2 步产物 |
| `--val-id` | 验证码 id | `1` | 只有 id0 就传 `0` |
| `--marker-size` | 黑方块实测边长 | `0.096` | **卡尺实测** |
| `--dict` | 字典 | `DICT_4X4_50` | 和打印件一致 |
| `--max-reproj` | PnP 重投影上限(px) | 2.0 | 严一点 1.5 |
| `--min-side` | 码最小边长(px) | 80 | 对应 ≤0.6 m |
| `--min-margin` | 离画面边缘(px) | 10 | 防贴边畸变 |
| `--min-z/--max-z` | 距离范围(m) | 0.15/1.5 | 按取景改 |
| `--out` | 结果 JSON | `log/handeye/probe_results.json` | |
| `--port` | 预览端口 | 8099 | 和 view.py 别同时开 |

### 为什么是"点"而不是"指尖"

`probe` 的 `P_base` 走的是 `base_link → link6 → t_tip`，与夹爪两指的实际位置无关。所以：

- 笔尖作为 TCP 时，`t_tip` 是笔尖相对 `link6` 的偏移；
- 笔必须刚性固定在 `link6` 的刚性下游（夹爪底座或夹紧的手指），指关节一动，TCP 就变了；
- 换回夹爪指尖使用时，必须重新做一次 `tcp`，不能复用笔尖的值。

---

## 5. 产物

| 文件 | 内容 |
|---|---|
| `log/handeye/tcp.json` | `t_tip`（link6）、`t_tip_tool`（end_link）、`P_base`、残差、rank |
| `log/handeye/probe_results.json` | 每个验证点的 `P_base`/`P_cam`/三轴误差、RMS |
| `log/handeye/*.jpg` | probe 每帧标注图（调试用） |

---

## 6. 常见问题

| 现象 | 原因 | 处理 |
|---|---|---|
| `probe` 挑错码 | 末端 id0 没拆，画面里两个 id0 | 拆掉末端板，只留桌上那张 |
| `probe` 一直“码没检出” | 相机没起 / 码不在视野 / 内参没喂 | 先 `rqt_image_view /image_raw`；确认 `/camera_info` 非零 |
| `tcp` 等不到 TF | test_mit 没起 / RSP 没出 `base_link→link6` | `tf2_echo base_link link6` |
| `tcp` 残差大 | 笔在夹爪里动 / 软头 / 碰点不准 | 重绑、换硬尖、画细十字 |
| `probe` 误差随姿态变 | TCP 不稳 | 回第 2 步重标 |
| `probe` 误差全部同向 | 手眼尺度/内参/码不平 | 查 `marker_size`、K、码是否贴平 |
| 夹爪在悬浮中张开/闭合 | `test_mit` 没夹爪控制器，指关节被外力带动 | 胶带绑死；或把笔绑在夹爪底座上 |
| 笔尖一碰桌，臂就沉 | 笔质量不在重力模型里 | 牵绳 `--hold-kp` 调大一点（6→8），或用手扶 |
