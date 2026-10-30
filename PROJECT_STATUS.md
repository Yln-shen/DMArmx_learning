# DMArmx 项目进度与交接（给下一个 AI 读）

> 最后更新：2026-10-30 21:20（Asia/Shanghai）
> 用途：任何新会话先读这一份，再读 `ARMWORK.md`（五阶段工作流）和 `常用命令.md`。
> 维护约定：每完成一个里程碑、或每次真机验证后，更新「已完成」「当前任务」「踩坑」三节。

---

## 0. 一句话现状

6 轴真机（DM4340P ×3 + DM4310 ×3）已经通过 `dm_armx_hardware` 插件 + ros2_control 跑通 MoveIt 位置轨迹；
**爪夹（DM4310，力位混控/力矩模式）刚装到末端，硬件驱动层还没接入**——URDF/MoveIt 侧的夹爪模型已就绪，真机控制链路还缺。

---

## 1. 环境与构建

| 项 | 值 |
|---|---|
| 工作区 | `/home/ylnn/DMArmx_learning` |
| ROS | ROS 2 **Lyrical**（pixi 环境） |
| 进入环境 | `cd /home/ylnn/DMArmx_learning && pixi shell`（activation 会自动 source `install/setup.bash`） |
| 构建 | `colcon build`。**注意：当前 install 是复制安装，不是 `--symlink-install`**，改完 C++/xacro 必须重新 build 才生效 |
| 启动 | 见 `常用命令.md`：`ros2 launch dm_armx_bringup dm_armx.launch.xml`，另一终端 `ros2 run dm_armx_commander commander_template` |
| 包名陷阱 | 目录 `src/dm_armx_config` 的 package 名是 **`dm_arm_config`**（少一个 x）。launch 里全用 `dm_arm_config`，别按目录名找 |

---

## 2. 硬件配置

### 2.1 机械臂

| 关节 | 电机 | 电机 ID | 档位 PMAX/VMAX/TMAX（实测回读） | offset（rad） | direction | URDF 限位（rad） |
|---|---|---|---|---|---|---|
| joint1 | DM4340P | 1 | 12.5 / 10.0 / 28.0 | 1.368925 | +1 | ±2.80 |
| joint2 | DM4340P | 2 | 12.5 / 10.0 / 28.0 | 1.802663 | +1 | -3.14 ~ 0.02 |
| joint3 | DM4340P | 3 | 12.5 / 10.0 / 28.0 | -0.296979 | +1 | -3.14 ~ 0.02 |
| joint4 | DM4310 | 4 | 12.5 / 30.0 / 10.0 | 1.859121 | +1 | -1.87 ~ 1.57 |
| joint5 | DM4310 | 5 | 12.5 / 30.0 / 10.0 | 1.300259 | +1 | ±1.57 |
| joint6 | DM4310 | 6 | 12.5 / 30.0 / 10.0 | 0.059319 | +1 | ±3.14 |

- 换算约定（`dm_joint.hpp`）：`motor = direction × joint + offset`，`joint = direction × (motor − offset)`。
- ⚠️ 4340P 与 4310 的 VMAX/TMAX 档位**不同**，用错解出来的速度/力矩差数倍且不报错。上面是实测回读值。
- 4310：减速比 10:1，额定 3.5 N·m，峰值 12.5 N·m，输出轴单位 rad。

### 2.2 夹爪

| 项 | 值 |
|---|---|
| 电机 | DM4310（用户口径：**力矩模式**，具体 CTRL_MODE 待确认） |
| 结构 | 平行两指 + 交叉齿条（rack & pinion），Seeed 风格网格 |
| URDF 关节 | `finger_left`：prismatic，沿 +Y，行程 **0 ~ 0.05 m**；`finger_right`：mimic `finger_left`，multiplier = −1 |
| 固定连接 | `end_joint`（fixed，link6 → end_link） |
| URDF 限位 | effort 8 N，velocity 0.08 m/s；`gripper_closed` = finger_left 0，`gripper_open` = 0.05 |
| 电机 ID / 通信 | **ESC_ID = 0x7**，与六轴共用同一个 `/dev/ttyACM0` 适配器 |
| 控制模式 | **力位混控 CTRL_MODE=4**（0x300+ID） |
| 实测角度 | 闭合（finger_left=0）：**+0.393874 rad**；全开（0.05 m）：**−5.572786 rad** |
| 换算 | Δ = 5.96666 rad / 0.05 m → **scale = 119.3332 rad/m**（节圆半径 8.38 mm）；**direction = −1**，**offset = +0.393874** |
| 机械限位 | 有（闭合/全开都是硬限位，实测角度即停在限位处） |

### 2.3 通信

- U2CAN 适配器，`/dev/ttyACM0`，`921600` bps（CDC-ACM，波特率对固件无意义但必须设成功）。
- 协议：主机 → 适配器 30 字节；适配器 → 主机 16 字节。
- CAN 1 Mbps。ID 规则：MIT = `ID`；位置速度 = `0x100+ID`；速度 = `0x200+ID`；力位混控 = `0x300+ID`；寄存器/刷新 = `0x7FF` + 数据段带目标 ID。
- 带宽粗算：7 台电机 @100 Hz ≈ 7×30 B 发 + 7×16 B 收 ≈ 322 B/周期，921600 足够。

---

## 3. 软件架构

### 3.1 包职责

| 包（目录） | 职责 |
|---|---|
| `dm_armx_description` | URDF/xacro/meshes/rviz；`urdf/arm.urdf` 是**展开后的真机模型（已含夹爪）** |
| `dm_arm_config`（`src/dm_armx_config`） | MoveIt 配置 + **真机 ros2_control.xacro** + launch；`config/DM_Armx.ros2_control.xacro` 才是真机插件声明 |
| `dm_armx_hardware` | ros2_control SystemInterface 插件 + 协议栈（唯一碰串口的包） |
| `dm_armx_bringup` | 顶层 launch + controller 配置（`dm_armx.launch.xml`、`test_motors.launch.xml`） |
| `dm_armx_commander` | MoveIt 封装节点，订阅 `/open_gripper`，用 named target 开合夹爪 |
| `dm_armx_interfaces` | 只有 `PoseCommand.msg`（当前链路未使用） |

### 3.2 驱动分层（自下而上）

```
dm_serial  串口字节流（非阻塞读、写满、TIOCEXCL 独占）
   ↓
dm_frames  协议层纯函数：造帧 / 切帧 / 解码 / 寄存器帧（第二份 Python 实现，需逐字节一致）
   ↓
dm_bus     一条总线：收发、反馈缓存、注册表、未知 ID 记账、寄存器 I/O、sync_states
   ↓
dm_joint   单关节语义：换算、软限位、NaN 拦截、使能/失能/保持帧
   ↓
dm_armx_hardware_interface  ros2_control 生命周期 + read()/write()
```

- 控制频率 100 Hz，命令接口只有 **position**，状态导出 position + velocity。
- `write()`：每个关节 `set_pos_vel(target, velocity_limit_=0.5)`，**每周期每关节只发一帧**。
- `read()`：`bus_->poll()` 后把缓存状态写进 ros2_control state interface。
- 生命周期：`on_init`（读参数/建关节）→ `on_configure`（开串口、等反馈、查 CTRL_MODE）→ `on_activate`（使能 + 保持帧）→ 循环 → `on_deactivate`（全部失能）。

---

## 4. 已完成 / 已验证

- [x] 6 轴真机点动 + ros2_control 位置控制（`test_motors.launch.xml` + `forward_position_controller`）。
- [x] MoveIt 规划 + 执行全链路（commit `90c7826`，2026-10-26）。
- [x] 6 个关节 offset 标定（见上表），direction 全为 +1。
- [x] 4340P / 4310 档位实测修正（**未提交**：`DM_Armx.ros2_control.xacro` 的 v_max/t_max 互换修正）。
- [x] 夹爪 URDF（`finger_left` + mimic `finger_right`）、SRDF `gripper` group、`gripper_open/closed` named state、`end_effector`、MoveIt `gripper_controller` 配置。
- [x] 总线未知 ID 警告（**未提交**：`on_configure` 里打印 `unknown_ids()`，用于发现夹爪的 ID 冲突）。
- [x] `dm_frames::force_pos_frame()` 已实现（力位混控造帧），但 **`MotorBus` 还没有 `send_force_pos`，也没有任何调用方**。

---

## 5. 当前任务：夹爪驱动接入（进行中）

**目标**：把 DM4310 夹爪以「力位混控（CTRL_MODE=4，0x300+ID）」接入 `dm_armx_hardware`，让 `/gripper_controller`（`finger_left`）能开合，且夹紧力受电流限幅约束。

### 5.1 需求确认（2026-10-30 访谈）

1. **控制模式**：力位混控 `CTRL_MODE=4`（0x300+ID，`p_des/v_des` + `i_des` 电流限幅）。
2. **接线与传动**：ESC_ID = **0x7**，与六轴共用 `/dev/ttyACM0`；闭合 +0.393874 rad、全开 −5.572786 rad。
   ⇒ `Δ = 5.96666 rad / 0.05 m`，**scale = 119.3332 rad/m**，**direction = −1**，**offset = +0.393874**。
   （用网格尺寸交叉验证：底座 184 mm 宽、单指 31 mm、齿条 90 mm，单指走 50 mm / 总开口 100 mm 自洽；
   节圆半径 8.38 mm ≈ 模数 1、16 齿，可信。）
3. **上层接口**：保留 JTC/MoveIt 位置开合，**再加 effort 接口运行时可调夹紧力**。
4. **安全**：夹爪有机械限位（闭合/全开都是硬限位）。故障时整臂怎么反应**尚未明确**，计划里按「夹爪故障不拖垮手臂」做，可用配置切回整机 ERROR。
5. **验收**：本轮只验收**空载**——`finger_left` 跟随 `/gripper_controller` 开合，位置/速度/温度/电流正常。
   带载、commander 全链路、抓取检测不作为本轮必过项。

### 5.2 已定位的代码缺口（改之前先看）

1. `on_configure` 的模式检查**写死 `CTRL_MODE == 2`**；夹爪是 4，会直接 `ERROR` 拒绝启动。→ 需要按关节声明期望模式（2/4）。
2. `MotorBus` 没有 `send_force_pos()`（只有造帧函数）→ 力位混控发不出去。
3. `dm_arm_config/config/DM_Armx.ros2_control.xacro` **没有 `finger_left` 关节** → 真机根本没有夹爪接口。
4. `ros2_controllers.yaml` 两份不一致：`dm_armx_bringup` 版给 gripper 声明了 velocity/effort，但插件只导出 position/velocity；`dm_arm_config` 版只声明 position。→ 控制器加载会因接口不存在而失败。
5. `Joint::enable()` 的保持帧用 **pos_vel 帧**；力位混控电机必须用 force_pos 帧保持，否则位置环没电流限幅。
6. `Joint` 的换算只有 `direction/offset`，**没有 scale**：`finger_left` 是米、电机是 rad，必须加传动比 `rad/m`。
7. `read()` 中任一关节 `ERR ∉ {0,1}` 就返回 `ERROR` → 整个 system 出错 → **全臂失能（重力下会掉）**。夹爪故障是否该拖垮整臂，是安全性决策。
8. 软限位没生效：`JointConfig::position_min/max` 是 `optional`，但 `on_init` 从没填过 → `clamp()` 实际是空操作，限位只靠 URDF/JTC。
9. `on_deactivate` 无条件失能；夹爪失能 = 松开，可能掉物。
10. **速度限幅是全局的 `velocity_limit_=0.5`**：对夹爪相当于 0.5 m/s，是 URDF 上限 0.08 m/s 的 6 倍 → 必须支持 per-joint 速度限幅。
11. 没有 **effort 命令接口**、没有力→电流换算、没有 `gripper_force_controller` → “运行时可调夹紧力”无处落地。
12. `param_bool()` 辅助函数被注释掉了，`fatal_on_error` 这类开关要把它恢复。

### 5.3 不要做的事（本轮）

- 不动 6 轴已有的 POS_VEL 路径（除非为了加 `scale`/`mode` 做最小改动）。
- 不生成测试文件（用户明确禁止，除非要求）。
- 不扩展成力控/抓取检测，除非访谈确认。

---

## 6. 关键文件地图

| 文件 | 作用 |
|---|---|
| `src/dm_armx_hardware/src/dm_armx_hardware_interface.cpp` | 生命周期、read/write、模式检查 |
| `src/dm_armx_hardware/include/dm_armx_hardware/dm_joint.hpp` | JointConfig/JointState/换算公式 |
| `src/dm_armx_hardware/src/dm_joint.cpp` | 使能/失能/保持帧/故障处理 |
| `src/dm_armx_hardware/src/dm_bus.cpp` | 收发、缓存、寄存器、`sync_states` |
| `src/dm_armx_hardware/src/dm_frames.cpp` | 所有帧编解码（含 `force_pos_frame`、`mit_frame`） |
| `src/dm_arm_config/config/DM_Armx.ros2_control.xacro` | **真机** joints/IDs/offset/档位 |
| `src/dm_arm_config/config/ros2_controllers.yaml` | controller_manager + JTC 配置（config 版） |
| `src/dm_armx_bringup/config/ros2_controllers.yaml` | 实际 launch 用的 controller 配置（bringup 版） |
| `src/dm_armx_bringup/launch/dm_armx.launch.xml` | 全链路 launch（rsp + ros2_control + move_group + rviz） |
| `src/dm_armx_bringup/launch/test_motors.launch.xml` | 只测电机的最小 launch |
| `src/dm_armx_description/urdf/arm.urdf` | 展开后的真机 URDF（含夹爪） |
| `src/dm_arm_config/config/DM_Armx.srdf` | arm/gripper group、named states |
| `src/dm_armx_commander/src/commander_template.cpp` | `/open_gripper` → MoveIt named target |
| `src/test/read_motor_angles.py` | 只读角度、辅助标定 offset/direction |

---

## 7. 真机踩坑（血泪清单，改代码前必读）

1. **CTRL_MODE 不对时位置帧被静默忽略**：使能正常、反馈正常，但电机不动。必须先读 0x0A 确认模式。
2. **没有实测位置就使能**：位置模式下电机内部指令被清零，会使能后朝零位冲。`Joint::enable()` 已强制要求先有缓存位置。
3. **保持帧必须用电机侧实测值**，且不能过软限位/换算（钳了就不是保持）。
4. **4310 与 4340P 档位不同**（12.5/30/10 vs 12.5/10/28），用错差数倍且不报错。
5. **寄存器回包和反馈帧共用 CMD=0x11**，必须先 `is_reg_response()` 分流，否则反馈缓存被污染。
6. **陈旧的过期回包会骗缓存**：要信状态之前用 `sync_states()`，别自己写排空循环。
7. **一个串口同一时刻只能一个进程**：`TIOCEXCL` 已加；同时开两个 controller_manager 会 EBUSY（以前踩过，表现为「发轨迹不动」）。
8. **陌生 ID 只记账、不进缓存**：防止夹爪 ID 与 1–6 撞车时污染手臂状态。
9. **模式切换/使能瞬间**：电机内部指令清零，必须先刷新位置再使能。
10. **夹爪 URDF 是 `prismatic`（米），电机是 `rad`**：现有 Joint 只有 direction/offset，直接复用会把 0.05 m 当成 0.05 rad 发出去。
11. 改 xacro 后必须重新 `colcon build`（install 是复制），并重启 launch；`robot_state_publisher` 只在启动时读一次 URDF。

---

## 8. 常用验证命令

```bash
# 只读电机角度（标定 offset/direction）
pixi run python src/test/read_motor_angles.py --ids 1,2,3,4,5,6 --config src/dm_arm_config/config/DM_Armx.ros2_control.xacro

# 最小真机测试（只 rsp + ros2_control + rviz）
ros2 launch dm_armx_bringup test_motors.launch.xml
ros2 topic pub --once /forward_position_controller/commands std_msgs/msg/Float64MultiArray "{data: [0,0,0,0,0,0]}"

# 看关节状态
ros2 topic echo /joint_states --once

# 夹爪（MoveIt 全链路起来后）
ros2 topic pub --once /open_gripper example_interfaces/msg/Bool "{data: true}"
```

---

## 9. 给下一个 AI 的硬性约定

- 先按 `ARMWORK.md` 的五阶段走，**Phase 1 访谈没确认前不要改代码**。
- 不改无关文件、不扩范围、不生成测试文件（除非用户明确要求）。
- 涉及真机：先确认手臂姿态/是否有急停；失能前想清楚重力负载会不会掉。
- 每次重大修改前提醒用户 `git commit`（当前有 3 个未提交改动）。
- 结论要落到文件/行号，不要凭记忆。

---

## 10. 变更记录

| 日期 | 内容 |
|---|---|
| 2026-10-26 | 6 轴 MoveIt 全链路打通（commit 90c7826） |
| 2026-10-29 | 4340P/4310 档位修正、未知 ID 警告（未提交） |
| 2026-10-30 | 新增本文档；夹爪驱动接入启动（访谈阶段） |
