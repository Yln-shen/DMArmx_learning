# 手眼标定 Runbook（eye-on-base，单目 USB，MIT 悬浮手拖采样）

> 目标：求 `T_base_link → camera`，用于后续抓取时把像素/PnP 结果换到基座系。
> 路线：**easy_handeye2 求解 + 自研质检采样**；验证用**夹爪指尖四点法 TCP + 桌面验证码**。
> 所有命令都在 `/home/ylnn/DMArmx_learning` 下、`pixi` 环境里执行。
> 已 `pixi shell` 进去的话可省略 `pixi run` 前缀；两种写法等价。

---

## 0. 先决条件核对（5 分钟）

```bash
cd /home/ylnn/DMArmx_learning

# 1) 包都在
pixi run ros2 pkg list | grep -E "easy_handeye2|aruco_ros|usb_cam"

# 2) 相机设备（插上后看 by-id，别用会漂的 /dev/video4）
ls -l /dev/v4l/by-id/
v4l2-ctl --list-devices

# 3) 相机支持的分辨率/格式（要 640x480 MJPG 或 YUYV）
v4l2-ctl -d /dev/video4 --list-formats-ext

# 4) 串口
ls -l /dev/serial/by-id/
```

把相机设备记成 `CAM`：

```bash
CAM=$(ls /dev/v4l/by-id/*-video-index0 | head -1)
echo $CAM
```

**内参核对**（`ost.yaml` 必须是这台相机 640×480 下标定的）：

```bash
# 起相机后用下面命令读一次 /camera_info，和 ost.yaml 的 K 对比
pixi run ros2 topic echo /camera_info --once
```

`ost.yaml` 里应是 `fx≈490.68, fy≈493.10, cx≈319.21, cy≈221.94`，畸变 `[0.0858, -0.1067, -0.0000, 0.0033, 0]`。对不上就先重标内参。

---

## 1. 标定板与安装（一次性）

| 板 | 规格 | 用途 | 安装 |
|---|---|---|---|
| 主标定板 | 96 mm 黑方块、`DICT_4X4_50`、**id=0** | 手眼采样 | 螺丝/夹具固定在法兰或夹爪侧面，**刚性**、不遮相机视线 |
| 验证板 | 96 mm、**id=1** | 采后验证 | 平贴桌面/亚克力板，双面胶固定 |

**必须用卡尺量黑色方块的实际边长**（打印缩放会让 96.0 变成 95.2 之类），后面 `--marker-size` 填实测值。填错 2 倍，z 就错 2 倍。

安装要点：

- 板面朝向要能覆盖采样姿态——末端伸到相机视野里时，板面大致对着相机。
- 板尽量轻（薄亚克力 + 3D 打印支架），每增加 100 g 在 0.1 m 处约多 0.1 N·m，会加大悬浮漂移。
- 板不能挡住夹爪指尖（后面还要用指尖做探针）。

---

## 2. 启动相机 + ArUco（采样/验证都要）

### 终端 A：相机

```bash
cd /home/ylnn/DMArmx_learning
CAM=$(ls /dev/v4l/by-id/*-video-index0 | head -1)
pixi run ros2 run usb_cam usb_cam_node_exe --ros-args \
  -p video_device:=$CAM \
  -p image_width:=640 -p image_height:=480 -p framerate:=30.0 \
  -p pixel_format:=mjpeg2rgb -p io_method:=mmap \
  -p frame_id:=camera_optical_frame \
  -p camera_name:=camera \
  -p camera_info_url:=file:///home/ylnn/DMArmx_learning/ost.yaml \
  -r image_raw:=/image_raw -r camera_info:=/camera_info
```

检查：

```bash
pixi run ros2 topic hz /image_raw          # 应 ~30 Hz
pixi run ros2 topic echo /camera_info --once
pixi run ros2 run rqt_image_view rqt_image_view /image_raw
```

帧率不对或花屏，多半是 `pixel_format` 与相机实际输出不匹配（MJPG 相机用 `mjpeg2rgb`，YUYV 用 `yuyv`）。

### 终端 B：marker TF（关键，先读这段）

⚠️ **aruco_ros 读不了 OpenCV 的 `DICT_4X4_50`**。它内置的是 ArUco 3.x 的字典集（ARUCO / ARUCO_MIP / ARTAG / AprilTag / ChiliTags），实测对 4x4_50 的码一张都检不出，`tf2_echo camera_optical_frame aruco_marker` 会一直等不到 frame。

所以主流程**不跑 aruco_ros**：由 `handdrag_qc.py sample` 自己检测，并实时发布 `camera_optical_frame -> aruco_marker`（OpenCV 4x4_50 + 5 参数畸变，正好匹配你的板）。easy_handeye2 只是这个 TF 的消费者。

顺序：先让 `handdrag_qc.py sample` 跑起来（它在发 TF），再起 `calibrate.launch.py`；反过来也行，server 每 2 s 重试，qc 起来后会自动连上。

```bash
# 验证 TF 已在发（qc 跑起来之后）
pixi run ros2 run tf2_ros tf2_echo camera_optical_frame aruco_marker
```

**尺度自检**：把板放在离相机卷尺量好的 `D` 处，看 TF 的 `translation.z`，应满足 `|z-D|/D < 1%`。差太多 = `marker_size` 或内参不对，别急着往下走。

（可选，二选一）如果你换印 **ARUCO_ORIGINAL** 的板，可以改回 aruco_ros，命令：

```bash
pixi run ros2 run aruco_ros single --ros-args \
  -p marker_size:=0.096 -p marker_id:=0 \
  -p reference_frame:=camera_optical_frame \
  -p camera_frame:=camera_optical_frame \
  -p marker_frame:=aruco_marker \
  -p image_is_rectified:=false -p min_marker_size:=0.005 \
  -r image:=/image_raw -r camera_info:=/camera_info
```

此时 qc 要加 `--no-tf`，否则两个节点抢同一个 TF。另注意 aruco_ros 只支持 **4 个**畸变系数，5 参数的 `plumb_bob` 会被当成零畸变（你的 k1=0.0858，边缘约 1–2 px 偏差）。

## 3. TCP 四点法（夹爪指尖，dm_armx 下做）

> 为什么不在悬浮下做：`test_mit` 只起 `forward_effort_controller`，**没起夹爪控制器**，指尖可能不刚性。TCP 必须让夹爪闭合且使能。

### 终端 1：正常 bringup（带夹爪控制器 + MoveIt）

```bash
cd /home/ylnn/DMArmx_learning
pixi run ros2 launch dm_armx_bringup dm_armx.launch.xml
```

### 终端 2：夹爪闭合（0 = 闭，0.05 = 全开）

```bash
pixi run ros2 action send_goal /gripper_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [finger_left], points: [{positions: [0.0], time_from_start: {sec: 3}}]}}"
```

### 终端 3：TCP 采集

```bash
cd /home/ylnn/DMArmx_learning
pixi run python /home/ylnn/realsense_ws/scripts/handeye/handdrag_qc.py tcp \
  --out log/handeye/tcp.json
```

操作：在桌面上画一个十字点，用 RViz 的 MotionPlanning 交互标记（或 commander）让**指尖**以 4–6 个差异大的姿态碰同一点：

- 从正上方下压
- 从左前方斜下
- 从右前方斜下
- 从后上方斜下
- （可选）再绕指尖轴转 90°

每个姿态停稳后回车。结束时输入 `q`。输出：

```
rank=6/6
t_tip (在 link6 系) = (...)
残差: 最大 x.xx mm  均值 y.yy mm
```

判据：`rank=6`、残差最大 **< 3 mm**。超了说明有的姿态没碰准或指尖在动，重来。

> 指尖是平面不是尖点，"碰准"本身有 1–2 mm 不确定性。想要更准就换一根尖针；换针后重跑这一步即可。

---

## 4. 手眼采样（悬浮 + 手拖，30+ 帧）

### 终端 1：MIT 悬浮 bringup

```bash
cd /home/ylnn/DMArmx_learning

# 用变量 + 预检：路径里混进中文标点（尤其「、」）会直接 ENOENT
PORT=/dev/serial/by-id/usb-HDSC_CDC_Device_00000000050C-if00
[ -e "$PORT" ] && echo "port ok -> $(readlink -f "$PORT")" || echo "PORT MISSING"

ros2 launch dm_armx_bringup test_mit.launch.xml \
  arm_kp:=0.0 arm_kd:=2.5 serial_port:="$PORT"
```

⚠️ 这个 launch 把驱动器切到 MIT（`CTRL_MODE=1`）并**失能再使能**，kp=0 时没有位置保持。**起它之前先用手扶住臂**，2 秒内启动终端 2，否则臂会靠阻尼慢慢垂下。

### 终端 2：重力前馈 + 牵绳保持

```bash
cd /home/ylnn/DMArmx_learning
pixi run python src/test/gravity_feedforward_test.py \
  --urdf log/arm_gravity2.urdf --scale 1.0 --ramp 3 \
  --hold-kp 6 --hold-leash 0.15
```

行为：

- `--ramp 3`：3 秒内前馈增益从 0 爬到 1，避免突然发力。
- `--hold-kp 6 --hold-leash 0.15`：始终有一个弱弹簧把臂拴在锚点；**拖出 0.15 rad 就把锚点弹到当前位置**，所以可以拖着走、松手即停。
- 板装上去后模型里没有这块质量，会有几十 mN·m 的残差；牵绳会兜住，但会有 1–2° 的静态偏差——不影响标定，因为我们读的是实际关节角。

### 终端 3/4/5：相机、easy_handeye2、qc

相机见第 2 节终端 A；marker TF 由终端 6 的 qc 发布，不需要 aruco_ros。再开：

```bash
cd /home/ylnn/DMArmx_learning
pixi run ros2 launch easy_handeye2 calibrate.launch.py \
  name:=dm_armx_handeye \
  calibration_type:=eye_on_base \
  tracking_base_frame:=camera_optical_frame \
  tracking_marker_frame:=aruco_marker \
  robot_base_frame:=base_link \
  robot_effector_frame:=link6
```

它会：

- 起 `handeye_server`（采样/求解/存盘服务）；
- 起 rqt 标定面板（**可以不用它采样**，只用它看状态）；
- 起一个 dummy `base_link → camera_optical_frame`（x=1 m 的占位），把 TF 树连起来——**标定完必须停掉这个 launch**，否则会和真实 TF 打架。

### 终端 6：质检采样

```bash
cd /home/ylnn/DMArmx_learning
pixi run python /home/ylnn/realsense_ws/scripts/handeye/handdrag_qc.py sample \
  --marker-size 0.096 --out log/handeye/samples_qc.json
```

> 这个进程同时发布 `camera_optical_frame -> aruco_marker` 的 TF（aruco_ros 读不了 4x4_50）。easy_handeye2 的 server 会等这个 TF；如果它先起，会每 2 s 重试，qc 跑起来就自动接上。

浏览器打开 `http://127.0.0.1:8099/` 看实时画面（带检测框、uv、side、z、关节角）。

操作节奏：

1. 手拖机械臂，让末端板进入画面、**朝向相机**、离画面边缘 ≥10 px；
2. 松手，等 0.5 s 不漂；
3. 回车 → 脚本做静止检查 + 质检（reproj/side/margin/z），合格才调 `take_sample` 入库；
4. 换个姿态（**重点动 joint4/5/6 让板面法向变化，joint1 左右摆扩大平移**），重复；
5. 收满 30+ 帧，输入 `q`。

姿态覆盖要求（不合格的样本再多也没用）：

| 维度 | 目标 |
|---|---|
| 板面法向变化 | 至少覆盖 ±30° 的俯仰/偏航/滚转组合 |
| 末端平移 | x/y/z 各方向跨度 ≥ 15 cm |
| 板在画面中的位置 | 中心、四角、上/下/左/右都要有 |
| 板像素尺寸 | ≥ 80 px（96 mm 码约对应 ≤ 0.6 m） |
| 重投影误差 | ≤ 2 px（大多数应 < 1 px） |

脚本每 5 帧会用自研解算报一次 `C_t` 和离散度；**离散度不再下降**、且样本数 ≥ 30 就可以停。

---

## 5. 求解、保存、导出

### 5.1 求解（两种方式，任选）

rqt 面板点 **Compute**，或命令行：

```bash
pixi run ros2 service call /easy_handeye2/calibration/compute_calibration \
  easy_handeye2_msgs/srv/ComputeCalibration "{}"
```

返回里看 `valid: true` 和 `calibration.transform`。

### 5.2 保存

```bash
pixi run ros2 service call /easy_handeye2/calibration/save_calibration \
  easy_handeye2_msgs/srv/SaveCalibration "{}"
```

写入 `~/.ros2/easy_handeye2/calibrations/dm_armx_handeye.calib`（YAML，含 `transform.translation/rotation`）。

### 5.3 导出 4×4 / 四元数 / TF 命令

```bash
cd /home/ylnn/DMArmx_learning
pixi run python /home/ylnn/realsense_ws/scripts/handeye/handdrag_qc.py dump \
  --name dm_armx_handeye --out log/handeye/T_base_cam.yaml
```

输出三样东西：

- `T_base_camera_optical`：easy_handeye 直接给的光学系结果（z 朝前）；
- `T_base_camera_link`：转到 REP-103 机身系（x 朝前）的结果；
- 两条可直接复制的 `static_transform_publisher` 命令。

### 5.4 停掉标定 launch，发 TF

先 `Ctrl-C` 掉第 4 节的 easy_handeye2 launch（**必须**，否则 dummy TF 还在）。然后：

```bash
# 机身系（x 朝前）
pixi run ros2 run tf2_ros static_transform_publisher \
  --x ... --y ... --z ... --qx ... --qy ... --qz ... --qw ... \
  --frame-id base_link --child-frame-id camera_link

# 光学系（z 朝前），机身系到光学系的固定旋转
pixi run ros2 run tf2_ros static_transform_publisher \
  --x 0 --y 0 --z 0 --qx 0.5 --qy -0.5 --qz 0.5 --qw -0.5 \
  --frame-id camera_link --child-frame-id camera_optical_frame
```

（具体数字用 `T_base_cam.yaml` 里的，别抄这里的省略号；第二条的四元数对应 `rpy=(-90°,0,-90°)`。）

验证：

```bash
pixi run ros2 run tf2_ros tf2_echo base_link camera_link
pixi run ros2 run tf2_ros tf2_echo base_link camera_optical_frame
```

> **二选一**：要么发 static TF，要么把相机写进 URDF（见 5.5）。两边都做会出现两个 `base_link→camera_link`，TF 会跳。

### 5.5 写进 URDF（可选）

在 `src/dm_armx_description/urdf/arm.xacro` 末尾加（数值来自 YAML）：

```xml
<link name="camera_link"/>

<joint name="camera_joint" type="fixed">
  <parent link="base_link"/>
  <child link="camera_link"/>
  <origin xyz="0.xx 0.yy 0.zz" rpy="r p y"/>
</joint>

<link name="camera_optical_frame"/>
<joint name="camera_optical_joint" type="fixed">
  <parent link="camera_link"/>
  <child link="camera_optical_frame"/>
  <origin xyz="0 0 0" rpy="-1.5708 0 -1.5708"/>
</joint>
```

`rpy` 由 `T_base_camera_link` 的旋转矩阵转欧拉角得到（`scipy.spatial.transform.Rotation.from_matrix(R).as_euler('xyz')`）。加完要重新 `colcon build` 并重启 bringup 才生效。

---

## 6. 采后验证（V1：指尖碰验证码中心）

### 6.1 准备

1. 打印 **id=1** 的 96 mm 码，平贴在一块刚性亚克力板上，双面胶固定到桌面（板不能动）。
2. 确认它在相机视野内、离相机 0.3–0.6 m。
3. 机械臂回到 `dm_armx.launch`（夹爪闭合、指尖可用），相机继续跑。

### 6.2 采集

```bash
cd /home/ylnn/DMArmx_learning
pixi run python /home/ylnn/realsense_ws/scripts/handeye/handdrag_qc.py probe \
  --calib dm_armx_handeye \
  --tcp log/handeye/tcp.json \
  --val-id 1 \
  --out log/handeye/probe_results.json
```

每轮两步：

1. 用 RViz/commander 把**指尖**移到验证码中心，停稳，回车 → 脚本用 FK + `t_tip` 算 `P_base`；
2. 移开机械臂（别挡码），回车 → 脚本用 `T_base_cam` 把码中心投到 base，算 `P_cam`。

输出每个点的 `P_base`、`P_cam`、三轴误差和模长。至少做 **3 个不同位置**（不是同一位置换姿态），最好 5 个。

### 6.3 判据

| 指标 | 合格 | 可用 | 不可用 |
|---|---|---|---|
| 单点误差模长 | < 5 mm | < 10 mm | > 15 mm |
| 多点 RMS | < 5 mm | < 10 mm | > 15 mm |
| 误差方向 | 随机分布 | 某一轴系统性偏大 → 查该轴 | 全部同向偏 → 标定/尺度错 |

误差是**系统性同向**的（比如 z 一直 +12 mm），优先怀疑：`marker_size` 填错、内参不对、`image_is_rectified` 没关、板没固定牢。

---

## 7. 参数总表（每个文件/参数干什么）

### 7.1 `ost.yaml`（相机内参）

| 字段 | 含义 | 你的值 | 作用 |
|---|---|---|---|
| `image_width/height` | 标定时的分辨率 | 640×480 | 必须和实际流一致 |
| `camera_matrix` | fx, fy, cx, cy | 490.68, 493.10, 319.21, 221.94 | PnP 的投影模型；fx/fy 错 → 距离错 |
| `distortion_model` | 畸变模型 | plumb_bob | OpenCV 标准 5 参数 |
| `distortion_coefficients` | k1,k2,p1,p2,k3 | 0.0858, -0.1067, ~0, 0.0033, 0 | 去畸变；`image_is_rectified:=false` 时生效 |
| `rectification_matrix` | 立体校正 | 单位阵 | 单目不用 |
| `projection_matrix` | 投影矩阵 | fx'≈499, cx'≈321 | 只对已校正图像有意义；我们的 PnP 用 K+D，不用它 |

### 7.2 `usb_cam` 参数

| 参数 | 含义 | 推荐 |
|---|---|---|
| `video_device` | 设备节点 | 用 `/dev/v4l/by-id/...`，别用 `/dev/video4` |
| `image_width/height` | 输出分辨率 | 640×480（与内参一致） |
| `framerate` | 帧率 | 30.0 |
| `pixel_format` | V4L2 转换格式 | MJPG 相机用 `mjpeg2rgb`，YUYV 用 `yuyv` |
| `io_method` | 取流方式 | `mmap`（默认，稳） |
| `frame_id` | 图像/TF 的相机帧名 | `camera_optical_frame`（必须是光学系） |
| `camera_name` | CameraInfo 名字 | `camera`（与 ost.yaml 一致） |
| `camera_info_url` | 内参文件 | `file:///home/ylnn/DMArmx_learning/ost.yaml` |
| `brightness/contrast/...` | 图像调节 | `-1` = 自动/不改 |

### 7.3 `aruco_ros single` 参数（可选方案，4x4_50 用不了）

| 参数 | 含义 | 推荐 | 坑 |
|---|---|---|---|
| `marker_size` | 黑方块边长（米） | 实测值，约 0.096 | 错 2 倍 → z 错 2 倍 |
| `marker_id` | 要跟踪的码 id | 0（末端板） | 验证板 id=1 别混 |
| `reference_frame` | TF 父帧 | `camera_optical_frame` | eye_on_base 必须是相机帧 |
| `camera_frame` | 解释 CameraInfo 的帧 | 同上 | 留空会回退到 reference_frame |
| `marker_frame` | TF 子帧名 | `aruco_marker` | 要和 easy_handeye 参数一致 |
| `image_is_rectified` | 图像是否已去畸变 | **false** | 默认 true；usb_cam 出的是原始图，true 会忽略畸变 |
| `min_marker_size` | 最小码面积占比 | 0.005 | 默认 0.02 会把 0.6 m 外的码滤掉 |
| `detection_mode` | 检测模式 | `DM_FAST` | 慢就换 `DM_VIDEO_FAST` |

### 7.4 `easy_handeye2 calibrate.launch.py` 参数

| 参数 | 含义 | 你的值 |
|---|---|---|
| `name` | 标定名，决定 `.calib` 文件名 | `dm_armx_handeye` |
| `calibration_type` | 标定类型 | `eye_on_base`（相机固定、码在末端） |
| `tracking_base_frame` | 跟踪系统基准帧 | `camera_optical_frame` |
| `tracking_marker_frame` | 被跟踪的码帧 | `aruco_marker` |
| `robot_base_frame` | 机器人基座帧 | `base_link` |
| `robot_effector_frame` | 末端帧（码刚接在它下游） | `link6` |
| dummy TF | 占位 `base_link→camera_optical_frame`（x=1 m） | 只为连通 TF 树，标定完停掉 |

服务（都在 `/easy_handeye2/calibration/` 下）：

| 服务 | 作用 |
|---|---|
| `take_sample` | 采一个样本（读 TF，存内存） |
| `get_sample_list` | 看已采样本数 |
| `remove_sample` | 删指定样本（`-1` = 最后一个） |
| `compute_calibration` | 求解（默认 OpenCV/Tsai-Lenz） |
| `save_calibration` | 写 `.calib` |

`.calib` 结构：

```yaml
parameters: {name, calibration_type, robot_base_frame, ..., tracking_marker_frame}
transform:
  translation: {x, y, z}      # T_base_link→camera_optical_frame 的平移（米）
  rotation: {x, y, z, w}      # 四元数
```

### 7.5 `test_mit.launch.xml` 参数

| 参数 | 含义 | 你的值 | 影响 |
|---|---|---|---|
| `arm_kp` | MIT 刚度 | `0.0` | 0 = 无力矩位置保持，可手拖；>0 会顶住 |
| `arm_kd` | MIT 阻尼 | `2.5` | 越大越"黏"，抑制抖动 |
| `serial_port` | U2CAN 串口 | by-id 路径 | 别用会漂的 `/dev/ttyACM0` |

它只起 `joint_state_broadcaster` + `forward_effort_controller`，**没有**位置控制器、MoveIt、夹爪控制器。所以：

- `/forward_position_controller/commands` 不存在，旧 `sample_onbase.py` 不能用；
- 夹爪不使能，指尖可能不刚性 → TCP 在 `dm_armx` 下做；
- `/joint_states` 和 `base_link→link6` TF 正常，采样脚本只读没问题。

### 7.6 `gravity_feedforward_test.py` 参数

| 参数 | 含义 | 你的值 | 影响 |
|---|---|---|---|
| `--urdf` | 带辨识惯性的模型 | `log/arm_gravity2.urdf` | 用错模型 → 补偿偏差大、漂移快 |
| `--scale` | 前馈总增益 | `1.0` | <1 欠补偿（往下沉），>1 过补偿（往上飘） |
| `--ramp` | 启动斜坡（秒） | `3` | 太短会突然发力 |
| `--hold-kp` | 牵绳弹簧刚度 | `6` | 越大越"顶"，越小越自由 |
| `--hold-leash` | 锚点重设阈值（rad） | `0.15` | 拖出这么多就认新位置；太小会一直弹回 |
| `--rate` | 发布频率 | 100 | 默认即可 |
| `--topic` | 力矩话题 | `/forward_effort_controller/commands` | 必须和 launch 的控制器一致 |
| `--record` | 记录 CSV | 采样时不需要 | 只在辨识/测漂移时用 |

### 7.7 `handdrag_qc.py` 参数

| 参数 | 含义 | 默认 | 建议 |
|---|---|---|---|
| `--marker-size` | 黑方块实测边长 | 0.096 | 卡尺实测值 |
| `--dict` | 字典 | `DICT_4X4_50` | 和打印件一致 |
| `--marker-id` | 采样码 id | 0 | |
| `--val-id` | 验证码 id | 1 | |
| `--max-reproj` | 重投影上限（px） | 2.0 | 严一点用 1.5 |
| `--min-side` | 码最小边长（px） | 80 | 对应 ≤0.6 m |
| `--min-margin` | 离画面边缘最小距离（px） | 10 | 防止贴边畸变 |
| `--min-z/--max-z` | 距离范围（m） | 0.15/1.5 | 按实际取景改 |
| `--still` | 静止阈值（rad/0.35 s） | 0.004 | 漂得厉害就放宽到 0.006 |
| `--camera-frame` | 相机帧 | `camera_optical_frame` | |
| `--no-server` | 只存本地不调 easy_handeye | 关 | 先空跑时用 |
| `--no-tf` | 不发布 `camera->marker` TF | 关（默认发） | 只有改用 aruco_ros 时才加 |
| `--auto N` / `--auto-interval S` | 自动采 N 帧后退出 | 0 / 4.0 | 测试/无人值守；正常手拖别加 |
| `--port` | MJPEG 预览端口 | 8099 | 和 view.py 别同时开 |

---

## 8. 验收清单

- [ ] `ost.yaml` 的 K 与 `/camera_info` 一致
- [ ] ArUco TF 的 z 与卷尺距离误差 < 1%
- [ ] TCP：rank=6，残差 < 3 mm
- [ ] 样本 ≥ 30，reproj 中位数 < 1 px
- [ ] 自研解与 easy_handeye 解的平移差 < 2 mm、旋转差 < 0.5°
- [ ] 自研解的 `C_spread` < 5 mm / 0.5°
- [ ] 验证：3 点以上，RMS < 10 mm（目标 < 5 mm）
- [ ] `tf2_echo base_link camera_link` 稳定、不跳
- [ ] 结果 YAML 已存档（`log/handeye/T_base_cam.yaml`）

---

## 9. 常见坑

| 现象 | 原因 | 处理 |
|---|---|---|
| z 差约 2 倍 | `marker_size` 用了 0.05 而实际 0.096 | 卡尺量，改参数 |
| 系统性 1–2 px 偏差 | `image_is_rectified:=true` 但图是原始畸变图 | 改 false，或先跑 image_proc |
| TF 跳变/两个父节点 | 标定 launch 的 dummy TF 没停 | 停 calibrate.launch，再发真实 TF |
| 样本数不涨 | `take_sample` 服务名/命名空间不对 | `ros2 service list | grep handeye` |
| 采样时码总在画面顶部 | 三脚架取景偏低 | 相机不能挪就调整采样姿态，让末端抬高/靠近 |
| 悬浮时慢慢下沉 | 板质量不在重力模型里 / `--scale` 偏小 | 提高 `--scale` 到 1.05，或减小板质量 |
| 拖拽时臂突然弹回 | 拖出 `hold-leash` 后锚点重设的瞬态 | 正常；慢一点拖 |
| `test_mit` 启动瞬间臂下坠 | kp=0，重力前馈还没接管 | 起 launch 前扶住，2 s 内起重力脚本 |
| `打开串口失败 … No such file or directory` | `serial_port` 末尾混入中文标点（「、」「，」）或设备号漂移 | 用变量传路径并 `[ -e "$PORT" ]` 预检；`cat -A` 看命令 |
| 指尖碰点重复性差 | 夹爪未使能 / 指尖是平面 | TCP 在 dm_armx 下做；换尖针 |
| 验证误差全部同向 | 尺度/内参/板没固定 | 查 `marker_size`、K、板刚性 |
| `/dev/video4` 消失 | 设备号漂移 | 用 `/dev/v4l/by-id/...` |
| `rqt_calibrator.py` 找不到 | shebang 被 colcon build 覆盖 | 重跑 `src/test/fix_easy_handeye2_scripts.py` |
| `tf2_echo camera_optical_frame aruco_marker` 一直等 | aruco_ros 不支持 `DICT_4X4_50` | 用 `handdrag_qc.py sample` 发 TF，或换 ARUCO_ORIGINAL 板 |
| `hand_world_samples` AttributeError | 该 fork 的 `SampleList` 字段是 `samples` | 已在新脚本里修正；旧脚本请改 `resp.samples.samples` |
| dump 出来的结果明显不对 | `dm_armx_handeye.calib` 是 11-09 的旧文件（可能是 eye_in_hand） | 先 `save_calibration` 覆盖，再 dump；或 dump 前看 `calibration_type` |

---

## 10. 产物清单

| 文件 | 内容 |
|---|---|
| `log/handeye/samples_qc.json` | 每帧 T_ee、T_cam_marker、角点、K/D、质检值 |
| `log/handeye/*.jpg` | 每帧标注图 + 原图（可复查） |
| `~/.ros2/easy_handeye2/calibrations/dm_armx_handeye.calib` | easy_handeye2 结果 |
| `log/handeye/T_base_cam.yaml` | 4×4 + 四元数（optical/link 两套） |
| `log/handeye/tcp.json` | 指尖 TCP 偏移 + 残差 |
| `log/handeye/probe_results.json` | 验证点误差表 |
