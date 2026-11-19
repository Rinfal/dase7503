#!/usr/bin/env bash
set -euo pipefail
project_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
export IDF_TOOLS_PATH="$project_root/runtime/espressif"
source "$project_root/runtime/esp-idf/export.sh"
# Keep the Pi responsive during both colcon's CMake phase and the IDF build.
export CMAKE_BUILD_PARALLEL_LEVEL=1
export MAKEFLAGS=-j1
export COLCON_DEFAULTS_FILE="$project_root/firmware/drive_controller/colcon-defaults.yaml"
cd "$project_root/firmware/drive_controller"
idf.py -DIDF_TARGET=esp32s3 reconfigure
ninja -C build -j1
