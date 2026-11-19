#!/usr/bin/env bash
set -euo pipefail
project_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
if [[ ${1:-} != --wheels-raised || ${2:-} != --unloaded ]]; then
    echo '恢复原厂固件前必须确认四轮悬空、机构空载。'
    exit 2
fi
port=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
if fuser "$port" >/dev/null 2>&1; then
    echo '串口仍被占用，拒绝回滚。'
    exit 1
fi
backup="$project_root/runtime/firmware-backups/20261119T064817Z"
python3 - "$backup" <<'PY'
import hashlib
import json
import sys
from pathlib import Path
p = Path(sys.argv[1])
m = json.loads((p/'manifest.json').read_text())
b = (p/'original_full_flash.bin').read_bytes()
assert m['double_read_verified'] and len(b) == 4194304
assert b == (p/'verification_full_flash.bin').read_bytes()
assert hashlib.sha256(b).hexdigest() == m['sha256']
PY
exec "$project_root/runtime/esp-tools/bin/esptool" --chip esp32s3 --port "$port" \
    --baud 460800 --before default-reset --after hard-reset \
    write-flash 0 "$backup/original_full_flash.bin"
