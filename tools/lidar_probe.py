#!/usr/bin/env python3
import argparse
import json
import math
import sys
import time
from pathlib import Path

import serial

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "robot_ws/src/logistics_robot"))

from logistics_robot.lidar import PacketParser, ScanAssembler


def main():
    parser = argparse.ArgumentParser(description="只读采集指定雷达串口，不发送配置命令。")
    parser.add_argument("--port", required=True)
    parser.add_argument("--seconds", default=5.0, type=float)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    packets = PacketParser()
    assembler = ScanAssembler()
    scans = []
    port = serial.Serial(port=None, baudrate=230400, timeout=0.05, exclusive=True)
    port.dtr = False
    port.rts = False
    port.port = args.port
    try:
        port.open()
        deadline = time.monotonic() + args.seconds
        while time.monotonic() < deadline:
            for packet in packets.feed(port.read(4096)):
                scans.extend(assembler.add(packet, time.monotonic()))
    finally:
        port.close()
    report = {"port": args.port, "valid_packets": packets.valid_packets,
              "bad_packets": packets.bad_packets, "full_scans": len(scans)}
    if scans:
        distances = [value for value in scans[-1].ranges if math.isfinite(value)]
        report.update({"valid_fraction": len(distances) / len(scans[-1].ranges),
                       "nearest_m": min(distances) if distances else None,
                       "scan_period": scans[-1].duration})
    result = json.dumps(report, indent=2)
    print(result)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(result + "\n")
    return 0 if scans else 1


if __name__ == "__main__":
    raise SystemExit(main())
