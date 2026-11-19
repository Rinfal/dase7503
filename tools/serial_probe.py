#!/usr/bin/env python3
import argparse
import json
import re
import time
from pathlib import Path

import serial


def main():
    parser = argparse.ArgumentParser(description="被动读取 ESP32 日志；不发送字节、不主动复位。")
    parser.add_argument("--port", required=True)
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--seconds", type=float, default=3.0)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    device = serial.Serial(port=None, baudrate=args.baud, timeout=0.05, exclusive=True)
    device.dtr = False
    device.rts = False
    device.port = args.port
    collected = bytearray()
    try:
        device.open()
        deadline = time.monotonic() + args.seconds
        while time.monotonic() < deadline:
            collected.extend(device.read(min(max(device.in_waiting, 1), 4096)))
            if len(collected) >= 65536:
                break
    finally:
        device.close()
    text = collected.decode("utf-8", errors="replace")
    lines = []
    for line in text.splitlines():
        if re.search(r"pass|psk|secret|token|ssid", line, re.IGNORECASE):
            lines.append("[已隐藏网络或凭据字段]")
        else:
            lines.append("".join(character if character.isprintable() else "." for character in line))
    report = {"port": args.port, "baud": args.baud, "bytes_received": len(collected),
              "printable_log": "\n".join(lines)[-2500:],
              "note": "没有主动发送串口数据；USB 串口打开时的线路瞬态由适配器决定。"}
    result = json.dumps(report, ensure_ascii=False, indent=2)
    print(result)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(result + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
