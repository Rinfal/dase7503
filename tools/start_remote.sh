#!/usr/bin/env bash
set -eo pipefail
project_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
source "$project_root/tools/environment.bash"
exec ros2 run logistics_robot phone_remote "$@"
