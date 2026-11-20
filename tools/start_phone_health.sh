#!/usr/bin/env bash
set -eo pipefail
project_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
source "$project_root/tools/environment.bash"
exec python3 "$project_root/tools/watch_phone_services.py"
