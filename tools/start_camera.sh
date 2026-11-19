#!/usr/bin/env bash
set -eo pipefail
project_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
source "$project_root/tools/environment.bash"
if [[ ! -f "$project_root/camera_ws/install/camera_ros/share/camera_ros/package.xml" ]]; then
    echo '摄像头驱动尚未构建完成。'
    exit 1
fi
exec ros2 run camera_ros camera_node --ros-args \
    -r __node:=camera \
    -p width:=1280 -p height:=720 -p format:=BGR888 \
    -p AfMode:=2 -p 'FrameDurationLimits:=[50000,50000]' \
    -p frame_id:=camera_optical_frame "$@"
