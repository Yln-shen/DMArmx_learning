#!/usr/bin/env bash
# 在 Ubuntu 24.04 (ROS 2 Jazzy apt) 容器里运行 MoveIt Setup Assistant。
# 需要你本机执行一次（会提示 sudo 密码），Agent 无法代跑 sudo。
set -euo pipefail

WORKSPACE="/home/ylnn/DMArmx_learning"
URDF="$WORKSPACE/src/dm_armx_description/urdf/arm.urdf"

if ! command -v docker >/dev/null 2>&1; then
  echo ">>> 安装 docker.io（需要 sudo）"
  sudo apt-get update
  sudo apt-get install -y docker.io
  sudo systemctl enable --now docker
fi

if [ -z "${DISPLAY:-}" ]; then
  echo "DISPLAY 为空，请在图形会话的终端里运行本脚本" >&2
  exit 1
fi

# 允许容器里的 root 连接本机 XWayland
xhost +local:root >/dev/null 2>&1 || true

echo ">>> 启动 ROS 2 Jazzy 容器（首次会拉取约 2GB 镜像并安装 MoveIt，请耐心等待）"
sudo docker run --rm -it \
  --name dmarm_msa \
  -e DISPLAY="$DISPLAY" \
  -e QT_X11_NO_MITSHM=1 \
  -e QT_QPA_PLATFORM=xcb \
  -e LIBGL_ALWAYS_SOFTWARE=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "$WORKSPACE:/work" \
  -e HOST_UID="$(id -u)" \
  -e HOST_GID="$(id -g)" \
  osrf/ros:jazzy-desktop \
  bash -lc '
    set -e
    apt-get update
    apt-get install -y ros-jazzy-moveit python3-colcon-common-extensions
    source /opt/ros/jazzy/setup.bash
    cd /work
    colcon build --packages-select dm_armx_description \
      --build-base /tmp/build --install-base /tmp/install
    source /tmp/install/setup.bash
    ros2 run moveit_setup_assistant moveit_setup_assistant \
      -u /work/src/dm_armx_description/urdf/arm.urdf
    chown -R "$HOST_UID:$HOST_GID" /work/src
  '
