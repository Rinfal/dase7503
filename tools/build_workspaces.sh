#!/usr/bin/env bash
set -eo pipefail
project_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
    echo '尚未安装 ROS 2 Jazzy，请先运行 tools/install_system.sh。'
    exit 1
fi
source /opt/ros/jazzy/setup.bash
export CMAKE_BUILD_PARALLEL_LEVEL=2
export MAKEFLAGS=-j2
python3 "$project_root/tools/prepare_sources.py"
cd "$project_root/camera_ws"
colcon build --executor sequential --packages-select libcamera \
    --event-handlers console_direct+ --meson-args \
    --buildtype=release -Dpipelines=rpi/pisp -Dipas=rpi/pisp \
    -Dtest=false -Dcam=enabled -Dqcam=disabled -Ddocumentation=disabled \
    -Dgstreamer=disabled -Dpycamera=disabled -Dwerror=false
source install/setup.bash
colcon build --executor sequential --packages-select camera_ros \
    --event-handlers console_direct+ --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
cd "$project_root/robot_ws"
colcon build --symlink-install --packages-select logistics_robot
source install/setup.bash
python3 "$project_root/tools/doctor.py" --output "$project_root/docs/evidence/after_build.json"
echo '构建完成。此脚本没有启动任何电机、舵机、雷达或导航节点。'
