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
| joint1 | DM4340P | 1 | 12.5 / 10.0 / 28.0 | 0.069238（2026-10-31 重标，原 1.368925） | +1 | ±2.80 |
| joint2 | DM4340P | 2 | 12.5 / 10.0 / 28.0 | 1.802663 | +1 | -3.14 ~ 0.02 |
| joint3 | DM4340P | 3 | 12.5 / 10.0 / 28.0 | -0.296979 | +1 | -3.14 ~ 0.02 |
| joint4 | DM4310 | 4 | 12.5 / 30.0 / 10.0 | 1.859121 | +1 | -1.87 ~ 1.57 |
| joint5 | DM4310 | 5 | 12.5 / 30.0 / 10.0 | 1.300259 | +1 | ±1.57 |
| joint6 | DM4310 | 6 | 12.5 / 30.0 / 10.0 | -0.264744 | +1 | ±3.14 |

- 换算约定（`dm_joint.hpp`）：`motor = offset + direction × scale × joint`，`joint = direction × (motor − offset) / scale`。六轴 `scale = 1`。
- ⚠️ 4340P 与 4310 的 VMAX/TMAX 档位**不同**，用错解出来的速度/力矩差数倍且不报错。上面是实测回读值。
- 4310：减速比 10:1，额定 3.5 N·m，峰值 12.5 N·m，输出轴单位 rad。

### 2.2 夹爪

| 项 | 值 |
|---|---|
| 电机 | DM4310，**力位混控 CTRL_MODE=4**（位置/速度环 + `i_des` 电流上限） |
| 结构 | 平行两指 + 交叉齿条（rack & pinion），Seeed 风格网格 |
| URDF 关节 | `finger_left`：prismatic，沿 +Y，行程 **0 ~ 0.05 m**；`finger_right`：mimic `finger_left`，multiplier = −1 |
| 固定连接 | `end_joint`（fixed，link6 → end_link） |
| URDF 限位 | **effort 50 N**（Lyrical 的 JointLimiter 用它钳 effort 命令，必须 ≥ `grip_force_max`），position −0.002 ~ 0.052，velocity 0.08 m/s；`gripper_closed` = finger_left 0，`gripper_open` = 0.05 |
| 电机 ID / 通信 | **ESC_ID = 0x7**，与六轴共用同一个 `/dev/ttyACM0` 适配器 |
| 控制模式 | **力位混控 CTRL_MODE=4**（0x300+ID） |
| 实测角度 | 闭合（finger_left=0）：**+6.560426 rad**；全开（0.05 m）：**+0.594911 rad**（2026-10-31 重测；绝对角比 10-30 漂了约 +6.17） |
| 换算 | Δ = 5.965515 rad / 0.05 m → **scale = 119.3103 rad/m**（节圆半径 8.38 mm）；**direction = −1**，**offset = +6.560426** |
| 机械限位 | 有（闭合/全开都是硬限位，实测角度即停在限位处） |
| 夹紧力 | `grip_force_default = 45 N`、`grip_force_max = 48 N`、`current_per_force ≈ 0.0009`（i_des / N）→ 默认 i_des ≈ 0.0405 |
| 摩擦 | 阻力**随行程增大**：闭合端静摩擦 ≈15 N，开启过程 11mm→20.7 N、24mm→30.0 N、37mm→41.7 N。30 N 上限在顶死闭合端后会拉不开（力恒定 +32 N 堵转），45 N 才能全行程（2026-10-31 实测） |

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

- 控制频率 100 Hz。命令接口：六轴只有 **position**；夹爪 **position + effort**（effort 单位 N = 夹紧力上限）。状态：六轴 position + velocity；夹爪多一个 effort。
- `write()`：每个关节 `set_pos_vel(target, velocity_limit_=0.5)`，**每周期每关节只发一帧**。
- `read()`：`bus_->poll()` 后把缓存状态写进 ros2_control state interface。
- 生命周期：`on_init`（读参数/建关节）→ `on_configure`（开串口、等反馈、查 CTRL_MODE）→ `on_activate`（使能 + 保持帧）→ 循环 → `on_deactivate`（全部失能）。

---

## 4. 已完成 / 已验证

- [x] 6 轴真机点动 + ros2_control 位置控制（`test_motors.launch.xml` + `forward_position_controller`）。
- [x] MoveIt 规划 + 执行全链路（commit `90c7826`，2026-10-26）。
- [x] 6 个关节 offset 标定（见上表），direction 全为 +1。
- [x] 4340P / 4310 档位实测修正（已提交 `9445306`）。
- [x] 夹爪 URDF（`finger_left` + mimic `finger_right`）、SRDF `gripper` group、`gripper_open/closed` named state、`end_effector`、MoveIt `gripper_controller` 配置。
- [x] 总线未知 ID 警告（已提交 `9445306`）。
- [x] **夹爪驱动接入（2026-10-30，未提交）**：`MotorBus::send_force_pos`、`Joint` 的 mode/scale/力参数、`finger_left` xacro、effort 接口 + `gripper_force_controller` 配置。编译 + xacro 展开已通过，**待真机验证**。
- [x] **重力补偿模型辨识 + 拖动示教（2026-11-01~05）**：模型 RMS **2.795 → 0.572 N·m**（留出集验证过），
  真机 kp 法复验 5 点；示教配置定为 `kp=0 / kd=2.5 / 纯重力前馈`。详见 §11。
- [x] **夹爪 JTC 开合全链路（2026-10-31 已真机验证）**：`/gripper_controller/follow_joint_trajectory` 发 `0.0` / `0.05`，两次都 `SUCCEEDED`，实测行程 42.9 / 43.8 mm；此前「发了不动」的两个原因见 5.5。

---

## 5. 当前任务：夹爪驱动接入（进行中）

**目标**：把 DM4310 夹爪以「力位混控（CTRL_MODE=4，0x300+ID）」接入 `dm_armx_hardware`，让 `/gripper_controller`（`finger_left`）能开合，且夹紧力受电流限幅约束。

### 5.1 需求确认（2026-10-30 访谈）

1. **控制模式**：力位混控 `CTRL_MODE=4`（0x300+ID，`p_des/v_des` + `i_des` 电流限幅）。
2. **接线与传动**：ESC_ID = **0x7**，与六轴共用 `/dev/ttyACM0`；2026-10-31 重测闭合 +6.560426 rad、全开 +0.594911 rad。
   ⇒ `Δ = 5.965515 rad / 0.05 m`，**scale = 119.3103 rad/m**，**direction = −1**，**offset = +6.560426**。
   ⚠️ 绝对角会随重新上电/重新清零/拆装整体漂移（10-30 的闭合角是 +0.393874）；换一次就重测一次。
   （用网格尺寸交叉验证：底座 184 mm 宽、单指 31 mm、齿条 90 mm，单指走 50 mm / 总开口 100 mm 自洽；
   节圆半径 8.38 mm ≈ 模数 1、16 齿，可信。）
3. **上层接口**：保留 JTC/MoveIt 位置开合；再加 effort 接口运行时可调夹紧力。力控制器 **随 launch 自动起**（0 视为默认力）。
4. **安全**：夹爪有机械限位；**夹爪故障不拖垮整臂**（`fatal_on_error=false`）；失能 = 松开、夹着的东西会掉，用户接受。
5. **验收**：本轮只验收**空载**——`finger_left` 跟随 `/gripper_controller` 开合，位置/速度/温度/电流正常。
   带载、commander 全链路、抓取检测不作为本轮必过项。

### 5.2 本轮实现（2026-10-30，代码已完成，待真机验证）

- `dm_bus`：新增 `send_force_pos()`（0x300+ID）。
- `dm_joint`：`JointConfig` 加 `mode/scale/velocity_limit/grip_force_default/grip_force_max/current_per_force/fatal_on_error`；换算带 `scale`；新增 `set_force_pos()`；`enable()` 按模式发 0x300 保持帧；`effort ≤ 0/NaN → 默认力`；故障隔离。
- 插件：`on_init` 解析新参数；`on_configure` 按关节期望模式检查 0x0A；`on_activate` 设置 effort 初值；`read()` 导出 effort、夹爪故障不返回 ERROR；`write()` 按模式分支。
- xacro：`finger_left`（ID 7 / mode 4 / direction −1 / offset 6.560426 / scale 119.3103 / vlim 0.08 / 0..0.05 m / **20N 默认 / 40N 上限** / `fatal_on_error=false`）。
- `ros2_controllers.yaml`：gripper JTC 只声明 position 命令 + position/velocity/effort 状态 + 终点 2mm / 路径 10mm 容差；新增 `gripper_force_controller`，并在 launch 里自动 spawn。
- 已编译通过（`colcon build --packages-select dm_armx_hardware dm_arm_config dm_armx_bringup`），xacro 展开检查通过。

### 5.3 调力流程（自动起）

```bash
# 终端 1：起 bringup（gripper_force_controller 会自动 active）
ros2 launch dm_armx_bringup dm_armx.launch.xml
ros2 control list_controllers | grep gripper   # 应有 gripper_controller + gripper_force_controller
# 终端 2：直接发 topic
ros2 topic pub --once /gripper_force_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [12.0]}"
```

- `0/负数/NaN` → 默认 30N（`grip_force_default`），不会松。
- 力上限 45N（`grip_force_max`；URDF `effort=50` 兜底）；`current_per_force=0.0009`（i_des / N，2026-10-31 按实测重标）。
- 力控制器随 launch 起，激活后每周期写 0 → 默认 30N；发过 35N 后保持 35N。
- **如果空载开合不动/很慢**：先看日志 `夹爪 write` 里的 `力=` 是否恒定在某个值（= 电流到顶、电机堵转）。齿条静摩擦 **≈15N**，`i_des` 必须 ≥ 0.02（≈22N）才走得动，0.025~0.03 稳定；低于这个值 0x300 帧完全正常、位置就是不动，**没有任何报错**。

### 5.4 本轮没做 / 已知限制

- 抓取检测、力闭环、`GripperCommand` action 没做。
- 启动阶段夹爪没反馈仍会让 `on_configure` 失败（fail fast）；`fatal_on_error` 只管运行中。
- `on_deactivate` 无条件失能；夹爪失能 = 松开、掉物。

### 5.5 2026-10-31 真机验证：JTC 开合链路

`/gripper_controller/follow_joint_trajectory` 依次发 `0.0`、`0.05`（`time_from_start` 3 s）：

| goal | 结果 | 实测行程 |
|---|---|---|
| 0.0（闭合） | `SUCCEEDED` | 0.0463 → 0.0034 m（42.9 mm） |
| 0.05（打开） | `SUCCEEDED` | 0.0034 → 0.0466 m（43.8 mm） |

之前「发了不动」是**两个叠加原因**，都不是驱动 bug：

1. **单位**：命令写成了 `positions: [5]`（= 5 m，全行程只有 0.05 m）。JTC 20 ms 内就因跟踪误差超差 abort
   （`Position Error 0.059951 > Position Tolerance 0.05`），JointLimiter 把命令钳到 0.0506，物理上只挪了 0.8 mm。
2. **力太小**：旧配置 `20 N × 0.00062` → `i_des = 0.0124`，正好等于齿条静摩擦，电机堵转（日志里 `力=` 恒定 ±14.28N）。
   直连实测：0.0124 完全不动，0.02 能动但迟滞大，0.025~0.03 全行程稳定。

修复：`grip_force_default 20→30`、`grip_force_max 40→45`、`current_per_force 0.00062→0.0009`（`DM_Armx.ros2_control.xacro`），
重建 `dm_arm_config` 后复验通过。另：`pvt_motor_test.py` 新增 `--disable`，用于节点被强杀后把还使能着的电机放掉。

---

## 6. 关键文件地图

| 文件 | 作用 |
|---|---|
| `src/dm_armx_hardware/src/dm_armx_hardware_interface.cpp` | 生命周期、read/write、模式检查 |
| `src/dm_armx_hardware/include/dm_armx_hardware/dm_joint.hpp` | JointConfig/JointState/换算公式 |
| `src/dm_armx_hardware/src/dm_joint.cpp` | 使能/失能/保持帧/故障处理 |
| `src/dm_armx_hardware/src/dm_bus.cpp` | 收发、缓存、寄存器、`sync_states` |
| `src/dm_armx_hardware/src/dm_frames.cpp` | 所有帧编解码（含 `force_pos_frame`、`mit_frame`） |
| `src/dm_armx_config/config/DM_Armx.ros2_control.xacro` | **真机** joints/IDs/offset/档位/mode |
| `src/dm_armx_config/config/ros2_controllers.yaml` | MoveIt 配置里的 controller 副本（**未同步**；实际以 bringup 版为准） |
| `src/dm_armx_bringup/config/ros2_controllers.yaml` | 实际 launch 用的 controller 配置（含 `gripper_force_controller`） |
| `src/dm_armx_bringup/launch/dm_armx.launch.xml` | 全链路 launch（rsp + ros2_control + move_group + rviz） |
| `src/dm_armx_bringup/launch/test_motors.launch.xml` | 只测电机的最小 launch |
| `src/dm_armx_description/urdf/arm.urdf` | 展开后的真机 URDF（含夹爪） |
| `src/dm_armx_config/config/DM_Armx.srdf` | arm/gripper group、named states |
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
10. **夹爪是 `prismatic`（米），电机是 `rad`**：`Joint` 现在有 `scale`（119.31 rad/m）和 `mode`；scale 配错会让手指行程成比例错（2 倍/一半）。
11. **Lyrical 的 JTC 不接受 `effort` 作为 state interface**：只允许 position/velocity/acceleration。夹爪的 effort 状态由 `joint_state_broadcaster` 发布，**不要**写进 `gripper_controller.state_interfaces`，否则 `gripper_controller` 初始化失败、action server 不出现（2026-10-30 真机踩过）。
12. **Lyrical 的 JointLimiter 会用 URDF `<limit effort>` 钳 effort 命令**：夹爪 URDF 原来 `effort="8"`，发 20N 被钳回 8N，日志里能看到 `command: effort 20 / limited: effort 8`；超限命令还会让位置命令那一拍被卡住。已把夹爪 URDF 的 effort 改成 **30N**，`grip_force_max` 仍由软件钳在 20N。
13. 改 xacro 后必须重新 `colcon build`（install 是复制），并重启 launch；`robot_state_publisher` 只在启动时读一次 URDF。
14. **4310-2EC 的绝对角会整体漂移（约 2π）**：重新上电/重新清零/拆装后，闭合角可能从 +0.394 变成 +6.560。用 `read_motor_angles.py` 重测闭合/全开并更新 xacro 的 `offset/scale`；否则驱动会把“全开”算成“已闭合”，发闭合命令只走 0.2 rad，看起来像**完全不动**（2026-10-31 踩过）。
15. **夹爪 action 的单位是米**：`positions: [5]` = 5 m（全行程 0.05 m）。JTC 20 ms 内就因跟踪误差 abort（`Position Error 0.06 > tolerance 0.05`），命令被钳到 0.0506，最多动 0.8 mm；另外全开位直接发 0.05 也几乎不动，**要先发 0.0 再发 0.05** 才看得出动作（2026-10-31）。
16. **力小于齿条摩擦时，一切「正常」但电机堵转**：`夹爪 write` 日志里 `力=` 恒定在某个值就是电流到顶。实测静摩擦 **≈15N**，`i_des ≥ 0.02` 才走；旧配置 `20N × 0.00062 = 0.0124` 正好卡在阈值上（2026-10-31）。
17. **强杀 ros2_control_node 不会失能**：SIGKILL（或 launch 被 SIGKILL）后电机保持最后一条指令继续顶住（本次夹爪顶在开口限位 32N，还带发热）。正常用前台 Ctrl-C 让生命周期走 `on_deactivate`；已经卡住就用 `pixi run python src/test/pvt_motor_test.py --disable` 补一帧失能。
18. **`0xFE` 是「保存位置零点」，不是「清除错误」**：说明书里这两节都印成了 `0xFE`（复制错误），误发会让当前角立刻变成 0、标定 offset 全废。2026-10-31 就对 joint1 干过一次（1.368925 → 0.069238，靠平移 offset 救回来）。不确定的命令字不要试。
19. **ERR=0xD(13) 通讯丢失是锁存故障，只能断电清**：4340P 的 `TIMEOUT(0x09)=10000 ms`、4310 夹爪为 0。使能状态下只要断命令 10 秒就锁存，之后必能失败、所有 controller 起不来。`0xFE` 清不掉（实测）。配套第 17 条：别强杀、别在使能时拔 USB。
20. **残留的 `robot_state_publisher` 会让「改了 xacro 却不生效」**：它持有旧的 `/robot_description` 继续发布，controller_manager 收到哪份先到就用哪份。2026-10-31 就是这个假象（改了 joint1 offset 却一直读到 1.369）。改完 xacro 重建后先 `ps -ef | grep [r]obot_state_publisher` 清干净再 launch。
21. **Lyrical 的 `JointLimiter` 对「实测位置越界」是 `terminate`，不是钳位**：`joint3` 实测 +0.0336 越了 URDF 上限 0.02 ⇒ 整条 `ros2_control_node` 启动 4 s 就 Abort；`joint2` 越 0.019 ⇒ `forward_position_controller` 被停用。URDF 上限必须比真机实际可达范围**宽**（2026-11-01 踩过，连锁导致 6 台电机看门狗锁存 ERR=13）。
22. **MIT 的 `p_des` 若取「每周期实测位置」⇒ `kp` 完全失效**：`kp·(p_des−p) ≡ 0`，调多少都是纯零刚度。必须用**位置命令**（`on_activate` 锁定的值）。2026-11-02 修。
23. **`xacro:arg` 必须声明在顶层文件**：写在被 `include` 的文件里会报 `Undefined substitution argument`。
24. **杀进程只杀 bash 外壳会留下孤儿 python**：表现为「两个 100 Hz 力矩发布者互相覆盖」⇒ 电机收到抖动指令，手感变成**又沉又涩又停不住**。判据用 `ros2 node list` / `ros2 topic info <topic> --verbose` 看 **Publisher count 必须是 1**，别只看 `ps`。
25. **摩擦前馈（沿运动方向补力）会吃掉唯一的自然阻尼**：本机摩擦 0.23~0.43 N·m，抵消后臂松手会继续滑（"停不住"）；`tanh(qd/0.02)` 在 |qd|>0.05 就饱和，力是跳变的 ⇒ "涩"。示教不要开。
27. **`gravity_feedforward_test.py --record` 原本是「纯被动模式」**：只记录、**不建 publisher、一个力矩都不发**（设计给位置模式下的辨识采集用）。想一边发力一边记录必须用**改后**的 `--record`；老语义改名 `--record-only`。2026-11-09 就是因为这条，用户以为在补重力，实际全程零补偿 ⇒ j4 自己往下掉。
28. **同一个串口只能有一个 `ros2_control_node`**：`dm_armx.launch.xml` 还开着时再起 `test_mit.launch.xml`，后者报 `打开串口失败 /dev/ttyACM0：Device or resource busy` ⇒ 硬件 `configure` 失败、控制器加载失败，但**进程看着像起来了**。此时任何「发力矩却没反应」的结论都不成立。判据：`fuser -v /dev/ttyACM0` 看谁占着；`ros2 control list_controllers` 看你的栈里有没有那个控制器。
：`p_des` 滞后 1.5 s + 钳 ±0.25 rad ⇒ 回拉 0.5 N·m，松手被拽回。要"黏滞感"应该加 **`kd`**，不是 `kp`。


---

## 8. 常用验证命令

```bash
# 只读电机角度（标定 offset/direction；注意 finger_left 的 joint 显示未除 scale，看电机角即可）
pixi run python src/test/read_motor_angles.py --ids 1,2,3,4,5,6 --config src/dm_armx_config/config/DM_Armx.ros2_control.xacro

# 最小真机测试（只 rsp + ros2_control + rviz）
ros2 launch dm_armx_bringup test_motors.launch.xml
ros2 topic pub --once /forward_position_controller/commands std_msgs/msg/Float64MultiArray "{data: [0,0,0,0,0,0]}"

# 看关节状态（finger_left 的 position 单位 m，effort 是力估计 N）
ros2 topic echo /joint_states --once

# 夹爪开合（JTC；0=闭，0.05=开）
ros2 action send_goal /gripper_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [finger_left], points: [{positions: [0.05], time_from_start: {sec: 2}}]}}"

# 夹爪调力（力控制器随 launch 自动起；0/负数 = 默认 8N）
ros2 control list_controllers | grep gripper
ros2 topic pub --once /gripper_force_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [12.0]}"

# commander 全链路（MoveIt named target）
ros2 topic pub --once /open_gripper example_interfaces/msg/Bool "{data: true}"
```

---

## 9. 给下一个 AI 的硬性约定

- 先按 `ARMWORK.md` 的五阶段走，**Phase 1 访谈没确认前不要改代码**。
- 不改无关文件、不扩范围、不生成测试文件（除非用户明确要求）。
- 涉及真机：先确认手臂姿态/是否有急停；失能前想清楚重力负载会不会掉。
- 每次重大修改前提醒用户 `git commit`（当前有 12 个未提交改动，均为夹爪接入）。
- 结论要落到文件/行号，不要凭记忆。

---

---

---

## 10. 变更记录

| 日期 | 内容 |
|---|---|
| 2026-10-26 | 6 轴 MoveIt 全链路打通（commit 90c7826） |
| 2026-10-29 | 4340P/4310 档位修正、未知 ID 警告 |
| 2026-10-30 | 新增本文档；夹爪驱动接入（访谈 → 计划 → 编码）；J6 offset 重标为 -0.264744（commit 9445306） |
| 2026-10-30 | 夹爪驱动最小实现：`send_force_pos`、mode/scale、effort 接口、随 launch 自动起的力控制器；编译通过，待真机验证 |
| 2026-10-31 | 夹爪 JTC 开合全链路真机验证通过；力参数重标为 30/45 N、`current_per_force=0.0009`；新增 `pvt_motor_test.py`（含 `--disable`）；joint1 offset 因误发 `0xFE` 重标为 0.069238 |
| 2026-11-01 | 修 joint3 上限越界（Lyrical JointLimiter terminate）⇒ 整链可启动；采 60 姿态辨识重力模型，RMS 2.795→0.602 |
| 2026-11-02 | 修 MIT 的 `p_des`（`kp` 原本完全失效）；`kp`/`kd` 改成 xacro 参数；真机 kp 法复验 5 点，j3 残余 8.87→0.07 N·m |
| 2026-11-05 | 正反两遍重采（120 姿态）⇒ RMS 0.572；摩擦实测；示教三轮对比（摩擦前馈/`kp=2` 都失败）⇒ 定稿 `kp=0 / kd=2.5` |

---

## 11. 重力补偿辨识与拖动示教（2026-11-01 ~ 11-05）

### 11.1 一句话结论

重力模型**已辨识并真机验证**（RMS 2.795 → **0.572** N·m），示教配置定为
**`arm_kp=0 / arm_kd=2.5` + 纯重力前馈**。剩余「略沉」来自 j2 的模型误差 ~0.4 N·m + 静摩擦 0.34 N·m，
调参解决不了，需要**运动学参数辨识**（见 11.7）。

### 11.2 这轮抓到的两个真 bug（都是原本就在，不是新引入）

| # | 现象 | 根因 | 修复 |
|---|---|---|---|
| 1 | 整条 `ros2_control_node` 启动 4 s 就 Abort，随后 6 台电机锁存 ERR=13 | `joint3` 实测 +0.0336 超过 URDF 上限 0.02，Lyrical 的 `JointLimiter` **对实测越界是 terminate** | `arm.urdf`：joint3 上限 0.02→**0.05**、joint2 0.02→**0.06** |
| 2 | `kp` 调多少都不起作用，MIT 是纯零刚度 | `write()` 里 MIT 的 `p_des` 取**每周期实测位置** ⇒ `kp·(p_des−p) ≡ 0` | 改用**位置命令**（`on_activate` 锁定值）+ NaN 兜底 |

### 11.3 改了哪些文件

| 文件 | 改动 |
|---|---|
| `src/dm_armx_hardware/src/dm_armx_hardware_interface.cpp` | MIT 的 `p_des` 改用位置命令（bug #2 的核心修复） |
| `src/dm_armx_description/urdf/arm.urdf` | joint2 上限 → 0.06、joint3 上限 → 0.05（bug #1） |
| `src/dm_armx_config/config/DM_Armx.ros2_control.xacro` | `kp`/`kd` 改为引用 xacro 参数 `$(arg arm_kp)` / `$(arg arm_kd)` |
| `src/dm_armx_config/config/DM_Armx.urdf.xacro` | 声明 `arm_kp`（默认 0.0）、`arm_kd`（默认 0.1） |
| `src/dm_armx_bringup/launch/test_mit.launch.xml` | 新增 launch 参数 `arm_kp` / `arm_kd`，透传给 xacro |
| `src/dm_armx_bringup/config/mit_test_controllers.yaml` | 加 `forward_position_controller`（MIT 下当位置伺服用） |
| `src/test/collect_gravity_poses.py` | 加 `--poses <json>` 姿态表；`--hold` 默认 2→**4 s**；新增 `travel_time()` 按最大转角给时长（原来固定 3 s 会让 JTC 跟踪误差超差 abort） |

### 11.4 数据与产物（都在 `log/`，已 gitignore）

> 2026-11-05 清理过一轮：中间副本、旧数据集、示教原始 CSV 与运行日志已删除（48 M → 9.2 M）。
> **只保留下面 4 个**——它们是 §11.5 全部数字的来源，删了就不可复核。

| 文件 | 内容 |
|---|---|
| `log/gravity_poses_plan2.json` | 60 个 FK 校验过的姿态（离桌余量 ≥16 cm），正序 + 反序 = 120 |
| `log/gravity_poses4.csv` | 120 姿态**正反两遍**的原始记录，18450 行（9.0 M） |
| `log/gravity_fit4.json` | 拟合结果：24 个质量/质心参数（RMS 0.572 的来源） |
| **`log/arm_gravity2.urdf`** | 把 fit4 写回惯性块的**可执行模型**，前馈就用它 |

已删除（需要时可重跑 §11.8 重新生成）：`gravity_poses{,2,3,_gear}.csv`、`gravity_fit{,2,2b,3,3r,_gear,_offsets}.json`、
`gravity_practical.json`、`synth_*.{csv,json}`、`tau_calib.csv`、`test_record.csv`、`gravity_home.json`、
`arm_gravity.urdf`、`gravity_poses_plan.json`、`teach_trace*.csv`、`log/*.log`、`log/build_*`、`log/ros/`。

`log/mit_check.urdf`（11-01 17:29）**不是本轮的**，未动。

`src/test/` 也清了 3 个一次性脚本（2026-11-05）：`fit_practical_model.py`（二次模型实验，已证明过拟合，结论在 §11.5）、
`tau_calib.py`（`effort` 尺度标定，已完成）、`park_to_home.py`（`collect_gravity_poses` 自带 park）。
**保留**：`gravity_feedforward_test.py`（含 `build_chain`，被另外两个 import）、`fit_gravity_params.py`、
`fit_gravity_offsets.py`、`collect_gravity_poses.py` —— 这 4 个就是 §11.8 的复现链路。
⚠️ `fix_easy_handeye2_scripts.py` / `patch_camera_calibration.py` 属于相机标定那条线（11-04），与本轮无关，未动。

### 11.5 辨识方法与关键数字

- 方法：POS_VEL 托住 → 停 4 s → 读 `/joint_states` 的 `effort`（DM_Armx 同款）；重力矩对每连杆
  `(m, m·cx, m·cy, m·cz)` 线性 ⇒ 最小二乘，24 参数、rank 满。
- 姿态集用 FK 离线校验：**活动连杆质心离桌面 ≥ 0.30 m**（桌面约在基座系 z≈0.14，由折叠位反推），
  姿态间插值路径最低 0.301 m。

| 模型 | 总 RMS (N·m) | j2 | j3 | j4 | j5 | j6 |
|---|---|---|---|---|---|---|
| CAD 名义 | **2.795** | 3.39 | 4.55 | 3.79 | 0.49 | 0.25 |
| 单遍 60 姿态 | **0.602** | 0.66 | 1.20 | 0.18 | 0.24 | 0.07 |
| 正反两遍 120 姿态 | **0.572** | 0.69 | 1.13 | 0.21 | 0.16 | 0.08 |

- **过拟合检验**：32 点拟合 / 31 点留出 ⇒ 0.769 / **0.633**，泛化成立。
- **DM_Armx 的辨识参数不能用**：运动学逐字节相同，但把它的参数直接搬过来会拟合出 **−0.914 kg 的末端质量**；
  穷举 9 种关节符号翻转，最好也只有 RMS 2.33 ⇒ 两台臂**动力学不一致**，必须各自辨识。
- **摩擦实测**（正反两遍同一姿态的 `effort` 差的一半）：j1 0.23 / **j2 0.34** / **j3 0.43** / j4 0.06 / j5 0.05 / j6 0.02 N·m。
- **j2 的误差不是可补的偏置**：残差按 q2 分段有 ±0.2~0.33 N·m 的形状偏置，但加"模型+残差多项式"
  后留出集**几乎不变**（0.620 → 0.617），而 j3 能降（1.43 → 1.00）⇒ j2 那部分是噪声/摩擦，j3 才是模型形状误差。

### 11.6 示教三轮对比（都是真机手感）

| 配置 | 你的感受 | 机理 |
|---|---|---|
| `kp=0 / kd=1.0`，纯重力前馈 | 「j2 补偿不足」 | 模型 j2 差 ~0.4 + 摩擦 0.34 |
| **A**：+ 摩擦前馈 `fc·tanh(qd/0.02)` | **涩、沉重、停不住** | 摩擦本是唯一的自然阻尼，抵消掉就失去制动；`tanh` 在 0.05 rad/s 就饱和 ⇒ 力跳变 |
| **B**：`kp=2` + `p_des` 慢跟随（τ=1.5 s，钳 ±0.25 rad） | **像弹簧，松手被拉回原位** | 滞后目标产生 0.5 N·m 回拉；该用 `kd` 实现"黏滞"，不是 `kp` |
| **最终**：`kp=0 / kd=2.5`，无摩擦前馈 | 待你确认 | 纯重力前馈 + 阻尼 |

### 11.7 已知限制 / 下一步

1. **j2 仍会略沉**（~0.4 N·m 模型误差 + 0.34 N·m 静摩擦）。要真正消除需要把**连杆长度/关节偏置**
   也纳入辨识（运动学参数辨识），属于更大的工程。
2. 模型参数**不是物理值**（`link4` 质心 1.08 m、`link6` 质量 0.18 kg）。它作为 `τ_g(q)` 的**预测器**是准的，
   **不能**当真实质量分布用，也别拿它算动力学。
3. 示教目前是"整臂同时使能"：折叠位附近我用 `gate`（q3 > −0.20 时淡出）避免在桌面上自顶起来。
   这个 gate 是工程折中，不是物理量。
4. 温度/限位/看门狗这些安全项没有新增改动。

### 11.8 复现步骤

```bash
# 1) 采姿态（自己起 dm_armx.launch.xml，结束会 park 再 SIGINT）
pixi run python src/test/collect_gravity_poses.py \
  --poses log/gravity_poses_plan2.json --csv log/gravity_poses4.csv --hold 4

# 2) 拟合
pixi run python src/test/fit_gravity_params.py --csv log/gravity_poses4.csv --out log/gravity_fit4.json

# 3) 示教（kp=0 纯重力补偿；kd=2.5 阻尼）
ros2 launch dm_armx_bringup test_mit.launch.xml arm_kp:=0.0 arm_kd:=2.5
#    另起一个只发 τ_ff 的节点：τ = clamp(ramp·gate·KDL_JntToGravity(arm_gravity2.urdf))
```
⚠️ 示教节点**只能有一个发布者**，起之前用 `ros2 topic info /forward_effort_controller/commands --verbose`
确认 `Publisher count` 是 1（见 §7 第 24 条）。

---
