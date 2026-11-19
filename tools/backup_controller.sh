#!/usr/bin/env bash
set -euo pipefail
umask 077
project_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
controller_port=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
esptool="$project_root/runtime/esp-tools/bin/esptool"
if [[ ${1:-} != --wheels-raised || ${2:-} != --unloaded ]]; then
    echo '必须先确认四轮悬空、机构空载且人员避让，再指定 --wheels-raised --unloaded。'
    exit 2
fi
if [[ ! -x "$esptool" || ! -r "$controller_port" || ! -w "$controller_port" ]]; then
    echo '备份工具未安装，或已确认串口不可访问。'
    exit 1
fi
if pgrep -f "^$project_root/agent_ws/install/micro_ros_agent/lib/micro_ros_agent/micro_ros_agent serial" >/dev/null; then
    echo '请先发零停车并停止代理，禁止同时占用串口。'
    exit 1
fi
if fuser "$controller_port" >/dev/null 2>&1; then
    echo '串口仍被其他进程占用，停止备份。'
    exit 1
fi
backup_root="$project_root/runtime/firmware-backups"
mkdir -p "$backup_root"
chmod 700 "$backup_root"
backup_dir="$backup_root/$(date -u +%Y%m%dT%H%M%SZ)"
mkdir -m 700 "$backup_dir"
export NO_COLOR=1
echo "只读 Flash 备份目录：$backup_dir"
"$esptool" --chip esp32s3 --port "$controller_port" --baud 460800 \
    --after no-reset get-security-info 2>&1 | tee "$backup_dir/security.log"
"$esptool" --chip esp32s3 --port "$controller_port" --baud 460800 \
    --before default-reset --after no-reset flash-id 2>&1 | tee "$backup_dir/flash_id.log"
"$esptool" --chip esp32s3 --port "$controller_port" --baud 460800 \
    --before default-reset --after no-reset read-flash --no-progress \
    0 ALL "$backup_dir/original_full_flash.bin" 2>&1 | tee "$backup_dir/read_first.log"
"$esptool" --chip esp32s3 --port "$controller_port" --baud 460800 \
    --before default-reset --after no-reset read-flash --no-progress \
    0 ALL "$backup_dir/verification_full_flash.bin" 2>&1 | tee "$backup_dir/read_second.log"
python3 - "$backup_dir" <<'PY'
import hashlib
import json
import sys
from datetime import datetime, timezone
from pathlib import Path

directory = Path(sys.argv[1])
original = directory / "original_full_flash.bin"
verification = directory / "verification_full_flash.bin"
first = original.read_bytes()
second = verification.read_bytes()
first_hash = hashlib.sha256(first).hexdigest()
second_hash = hashlib.sha256(second).hexdigest()
valid_size = len(first) >= 1024 * 1024 and len(first) & (len(first) - 1) == 0
verified = valid_size and first == second
report = {
    "created_utc": datetime.now(timezone.utc).isoformat(),
    "chip": "esp32s3",
    "bytes": len(first),
    "sha256": first_hash,
    "verification_sha256": second_hash,
    "double_read_verified": verified,
    "security_review_required_before_write": True,
    "flash_written": False,
    "device_left_in_download_mode": True,
    "contains_private_configuration": True,
}
(directory / "manifest.json").write_text(json.dumps(report, indent=2) + "\n")
print(json.dumps(report, indent=2))
if not verified:
    raise SystemExit("两次完整读取不一致或容量异常，禁止后续刷写。")
PY
sync
echo '两次完整 Flash 读取和比对完成；没有擦除或刷写。设备保持下载模式。'
