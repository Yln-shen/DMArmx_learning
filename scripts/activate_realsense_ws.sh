# 让 pixi 环境也能找到 ~/realsense_ws 里构建的 easy_handeye2
# （该工作区不存在时静默跳过，不影响主环境）
if [ -f /home/ylnn/realsense_ws/install/setup.bash ]; then
    . /home/ylnn/realsense_ws/install/setup.bash
fi
