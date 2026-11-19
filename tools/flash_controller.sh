#!/usr/bin/env bash
set -euo pipefail
umask 077
project_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
if [[ ${1:-} != --wheels-raised || ${2:-} != --unloaded ]]; then
    echo '必须确认四轮悬空、机构空载，再指定 --wheels-raised --unloaded。'
    exit 2
fi
port=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
if fuser "$port" >/dev/null 2>&1; then
    echo '控制板串口仍被占用，拒绝刷写。'
    exit 1
fi
python3 - "$project_root" <<'PY'
import hashlib
import json
import struct
import sys
from pathlib import Path
root = Path(sys.argv[1])
backup = root / 'runtime/firmware-backups/20261119T064817Z'
manifest = json.loads((backup / 'manifest.json').read_text())
original = (backup / 'original_full_flash.bin').read_bytes()
verification = (backup / 'verification_full_flash.bin').read_bytes()
assert manifest['double_read_verified'] and original == verification
assert len(original) == 4194304
assert hashlib.sha256(original).hexdigest() == manifest['sha256']
security = (backup / 'security.log').read_text()
assert 'Secure Boot: Disabled' in security and 'Flash Encryption: Disabled' in security
build = root / 'firmware/drive_controller/build'
partition = (build / 'partition_table/partition-table.bin').read_bytes()
def records(table):
    result = []
    for offset in range(0, len(table)-31, 32):
        record = struct.unpack('<HBBII16sI', table[offset:offset+32])
        if record[0] != 0x50aa:
            break
        result.append(record)
    return result
assert records(original[0x8000:0x8c00]) == records(partition), '必须保持原厂分区布局'
files = json.loads((build / 'flasher_args.json').read_text())['flash_files']
for offset, name in files.items():
    start = int(offset, 0)
    data = (build / name).read_bytes()
    assert data
    for low, high in ((0x9000, 0x10000), (0x330000, 0x336000)):
        assert start+len(data) <= low or start >= high, '禁止覆盖原厂配置分区'
print('完整备份、分区布局及配置保留范围核验通过。')
PY
export IDF_TOOLS_PATH="$project_root/runtime/espressif"
source "$project_root/runtime/esp-idf/export.sh" >/dev/null
cd "$project_root/firmware/drive_controller"
cd build
python -m esptool --chip esp32s3 --port "$port" --baud 460800 \
    --before default_reset --after hard_reset write_flash @flash_args
