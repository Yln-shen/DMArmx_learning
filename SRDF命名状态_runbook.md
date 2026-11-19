# SRDF 命名状态 Runbook（DM_Armx）

> **状态**：2026-11-15，P0 完成 —— SRDF 里已有 `home` / `ready` / `place` + 夹爪开合，
> 静态校验 0 错，install 已更新。**真机执行与 RViz 规划还没做**（见 §3）。

一句话：命名状态是**关节空间**的固定姿态，值一律从真机快照来，不手编；
改完 SRDF 必须重新 build（install 是复制安装，不是 symlink）。

---

## 1. 现有状态

| 状态 | 用途 | 工具轴 | 笔尖位置（base_link） | 离底座竖轴 | 离基准面* |
|---|---|---|---|---|---|
| `home` | 折叠收拢 / 结束位 | 水平朝外 −60° | (0.134, −0.238, 0.190) | 0.273 m | 0.190 m |
| `ready` | 相机观察 / 待抓预备 | 朝下 −77.5° | (−0.149, −0.110, 0.042) | 0.185 m | 0.042 m |
| `place` | 放置区上方释放位 | 朝下 −80.3° | (−0.409, −0.120, 0.055) | 0.426 m | 0.055 m |
| `gripper_open` | 夹爪张开 50 mm | — | — | — | — |
| `gripper_closed` | 夹爪闭合 | — | — | — | — |

\* 按 base_link 平面 ≈ 台面估算；两者高度关系还没实测（§5）。
笔尖位置 = link6 工具轴 + 0.167 m（笔尖 TCP，见 `log/handeye/tcp.json`）；
换工具/换夹爪后这个数要重算，但状态本身（关节角）不变。

`transport` 按需求取消了，不要再加。

---

## 2. 新增 / 修改一个状态（三步）

### 2.1 把臂摆到目标姿态

粗调用 MIT 悬浮（可手拖、带重力前馈，松手即停）：

```bash
cd /home/ylnn/DMArmx_learning
PORT=/dev/serial/by-id/usb-HDSC_CDC_Device_00000000050C-if00
pixi run ros2 launch dm_armx_bringup test_mit.launch.xml \
  arm_kp:=0.0 arm_kd:=2.5 serial_port:="$PORT"

# 另一个终端：重力前馈 + 牵绳
pixi run python src/test/gravity_feedforward_test.py \
  --urdf log/arm_gravity2.urdf --scale 1.0 --ramp 3 --hold-kp 6 --hold-leash 0.15
```

> `test_mit` 没有夹爪控制器，夹爪停在启动那一帧；命名状态只记六轴，够用。
> 也可以在正常 bringup 里用 RViz 的 joint 滑块微调。

### 2.2 快照成 SRDF 片段

```bash
pixi run python src/test/snapshot_srdf_state.py --name transport
```

把输出的 `<group_state>` 原样贴进 `src/dm_armx_config/config/DM_Armx.srdf`，
放在 `home` / `ready` / `place` 旁边，**不要**放进夹爪那一段。

### 2.3 校验 → 生效

```bash
pixi run python src/test/check_srdf_states.py       # 退出码 0 才能往下走
pixi run colcon build --packages-select dm_arm_config
```

`check_srdf_states.py` 查：关节名是否存在、是否属于该 group、**有没有漏写关节**
（漏写 MoveIt 会静默当 0）、是否越限、mimic 关节有没有被误写、`disable_collisions` 重复等。
退出码：`0` 全过 / `1` 有错 / `2` 只有警告。

注意脚本读的是**源码** SRDF；若它提示"install 副本"，说明你改错地方了。

---

## 3. 验收（顺序不能跳）

1. **加载**：`pixi run ros2 launch dm_armx_bringup dm_armx.launch.xml`
   → RViz 的 MotionPlanning → Context → Goal State 里能看到三个状态；
2. **规划**：`Select Goal State` 选一个 → `Plan`（**不要 Execute**）→ 能出轨迹；
3. **真机慢速**：第一次把 RViz 的 Velocity Scaling 拉到 **0.1**
   （`joint_limits.yaml` 里 max_velocity=0.5，0.1×0.5 = 0.05 rad/s ≈ 2.9°/s），
   逐个状态 Execute，手放电源上，一次只走一个；
4. **核对**：执行后 `ros2 topic echo /joint_states --once`，与目标值偏差应 < 0.02 rad。

---

## 4. 踩坑清单

| 现象 | 原因 | 处理 |
|---|---|---|
| 改完 SRDF 不生效 | install 是复制安装；旧 move_group 残留 | `colcon build --packages-select dm_arm_config`；`pgrep -af move_group` 清残留 |
| SRDF 解析失败 | XML 注释里出现连续两个减号 | 注释里禁止写 `--` |
| 状态里写了 `finger_right` | 它是 mimic（`multiplier=-1`） | 只写 `finger_left` |
| 状态漏了某个关节 | MoveIt 静默按 0 处理，不报错 | 用快照脚本生成，或跑校验脚本 |
| 启动报 `joint6 out of bounds` | DM 电机单圈绝对编码器，掉电后差 ±2π | 已给六轴配 `unwrap_min/max`；实测 joint6 三次上电分别读到 −0.05 / +3.20 / +6.33 |
| joint1 到 −2.886 报越界 | 原 URDF 限位 ±2.8 太保守 | 已放宽到 ±2.95（`arm.urdf` + 2π 展开区间同步放大），**机械行程要真机确认** |
| RViz 里 joint1 看着转了角度 | Orbit 视角 Yaw=−0.623（−35.7°）；也可能是常值零点/底座安装偏移 | 先把 `Views → Yaw 0 / Pitch 90°` 正俯视再看；再转 joint1 判断差值是否恒定 |
| RViz 报 `No kinematics plugins` | `dm_armx.launch.xml` 直接起 rviz2，没喂 MoveIt 参数 | 交互标记不可用，但 `Select Goal State` 规划仍可用；要修就改成 include `moveit_rviz.launch.py` |

---

## 5. 未决 / 已知限制

- **base_link 与台面、房间朝向的关系没实测**：手眼数据里两处"台面高度"不一致
  （十字触碰 z≈0.033、码中心 z≈0.123），且 base_link 的 +X 是否等于"桌子前方"未验证。
  影响：以后加桌面/相机的碰撞体、或用世界系坐标时会踩；纯 base_link 内的抓取链路不受影响。
- **相机不在 URDF/SRDF 里**（eye-to-hand，靠 static TF 发布），所以 MoveIt 只做自碰撞检查，
  看不到桌子、相机、颗粒。
- **`place` 依赖放宽后的 joint1 限位**（−2.95）；机械行程若不够，回退方案：限位改回 −2.90、
  展开区间改回 `[-2.88, 3.02]`、place 重拖到 −2.89 以内。
- SRDF 里 `gripper` group 含 fixed 的 `end_joint`，是 MSA 的写法，无害，暂不动。

---

## 6. 文件地图

| 文件 | 作用 |
|---|---|
| `src/dm_armx_config/config/DM_Armx.srdf` | 命名状态本体（改这里） |
| `src/dm_armx_description/urdf/arm.urdf` | 活 URDF（限位；bringup 用的就是它） |
| `src/dm_armx_config/config/DM_Armx.ros2_control.xacro` | 电机映射 + 2π 展开区间 |
| `src/test/snapshot_srdf_state.py` | 快照当前关节角 → SRDF 片段 |
| `src/test/check_srdf_states.py` | 静态校验（不启动 ROS） |
