#!/usr/bin/env bash
set -eo pipefail
project_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
source "$project_root/tools/environment.bash"
exec ros2 run camera_ros camera_node --ros-args -r __node:=camera \
    -p width:=960 -p height:=540 -p format:=BGR888 -p jpeg_quality:=80 \
    -p AfMode:=2 -p 'FrameDurationLimits:=[50000,50000]' \
    -p frame_id:=camera_optical_frame
