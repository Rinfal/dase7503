#!/usr/bin/env bash
set -eo pipefail
project_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
    echo '尚未安装 ROS 2 Jazzy。'
    exit 1
fi
source /opt/ros/jazzy/setup.bash
export CMAKE_BUILD_PARALLEL_LEVEL=${ROBOT_BUILD_JOBS:-2}
export MAKEFLAGS=-j${ROBOT_BUILD_JOBS:-2}
python3 "$project_root/tools/prepare_sources.py"
cd "$project_root/agent_ws"
colcon build --executor sequential --packages-up-to micro_ros_agent \
    --event-handlers console_direct+ \
    --cmake-args -DUAGENT_BUILD_EXECUTABLE=OFF -DUAGENT_P2P_PROFILE=OFF -DCMAKE_BUILD_TYPE=Release
echo '代理已构建；没有启动代理、打开串口或改写控制板固件。'
