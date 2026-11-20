#!/usr/bin/env bash
set -eo pipefail
project_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
source "$project_root/tools/environment.bash"
exec ros2 run logistics_robot qr_scanner --ros-args \
    -p max_rate:=5.0 -p preview_rate:=12.0 -p preview_width:=960 \
    -p jpeg_quality:=70 -p confirmation_frames:=3 -p compressed_input:=true
