#!/usr/bin/env bash
set -eo pipefail
project_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
source "$project_root/tools/environment.bash"
exec ros2 run logistics_robot qr_scanner --ros-args \
    -p max_rate:=5.0 -p preview_rate:=8.0 -p preview_width:=640 \
    -p jpeg_quality:=55 -p confirmation_frames:=3 -p compressed_input:=true
