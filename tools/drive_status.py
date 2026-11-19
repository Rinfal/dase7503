#!/usr/bin/env python3
"""Record the new controller's telemetry without publishing commands."""
import argparse
import json
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--seconds", type=float, default=3)
    args = parser.parse_args()
    if not 1 <= args.seconds <= 10:
        parser.error("观察时间1..10秒")
    rclpy.init()
    node = Node("drive_passive_observer")
    samples = []

    def receive(msg):
        try:
            value = json.loads(msg.data)
            if str(value.get("firmware", "")).startswith("dase-rc-"):
                samples.append(dict(value, received=time.monotonic()))
        except (ValueError, TypeError):
            pass

    node.create_subscription(String, "/drive/status", receive, qos_profile_sensor_data)
    deadline = time.monotonic()+10
    try:
        while time.monotonic() < deadline and not samples:
            rclpy.spin_once(node, timeout_sec=0.1)
        deadline = time.monotonic()+args.seconds
        while samples and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        report = {"sample_count": len(samples), "samples": samples}
        report["idle_verified"] = bool(len(samples) >= 10
            and time.monotonic()-samples[-1]["received"] < 0.5
            and samples[-1]["board_us"] > samples[0]["board_us"]
            and all(s["pwm"] == [0]*4 and not s["armed"] for s in samples)
            and all(s["encoders"] == samples[0]["encoders"] for s in samples))
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2)+"\n")
        print(json.dumps({k: v for k, v in report.items() if k != "samples"}))
        if not report["idle_verified"]:
            raise RuntimeError("未确认新固件静止状态")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
