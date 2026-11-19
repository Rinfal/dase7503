#!/usr/bin/env python3
import argparse
import json
import math
import multiprocessing
import os
import time
from pathlib import Path

from bench_pulse import stop_guard


def main():
    parser = argparse.ArgumentParser(description="仅悬空验证控制板命令丢失停车，独立进程兜底发零。")
    parser.add_argument("--wheels-raised", action="store_true")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if not args.wheels_raised or os.environ.get("ROS_DOMAIN_ID") != "20":
        parser.error("仅允许已确认悬空且 ROS_DOMAIN_ID=20 的实际底盘")

    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data

    pulse_duration = 0.4
    quiet_duration = 1.2
    context = multiprocessing.get_context("spawn")
    start_event = context.Event()
    ready_event = context.Event()
    guard = context.Process(target=stop_guard,
                            args=("/cmd_vel", start_event, ready_event,
                                  pulse_duration + quiet_duration))
    guard.start()
    rclpy.init()
    node = Node("supervised_watchdog_probe")
    publisher = node.create_publisher(Twist, "/cmd_vel", 1)
    samples = []

    def receive(message):
        velocity = message.twist.twist
        speed = math.hypot(velocity.linear.x, velocity.linear.y)
        samples.append((time.monotonic(), speed, abs(velocity.angular.z)))

    node.create_subscription(Odometry, "/odom_raw", receive, qos_profile_sensor_data)
    report = {"verified": False}
    try:
        if not ready_event.wait(timeout=5):
            raise RuntimeError("独立停车进程未就绪")
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            subscriptions = node.get_subscriptions_info_by_topic("/cmd_vel")
            publishers = node.get_publishers_info_by_topic("/cmd_vel")
            other_publishers = [info.node_name for info in publishers
                                if info.node_name not in ("supervised_watchdog_probe", "bench_independent_stop")]
            if other_publishers:
                raise RuntimeError("存在其他速度发布者，无法验证 MCU 看门狗：" + str(other_publishers))
            if (samples and any(info.node_name == "YB_Car_Node"
                                and info.topic_type == "geometry_msgs/msg/Twist" for info in subscriptions)):
                break
        else:
            raise RuntimeError("控制板订阅或真实里程计未就绪")
        start_event.set()
        started = time.monotonic()
        command = Twist()
        command.linear.x = 0.06
        print("悬空看门狗测试：前进脉冲 0.4 秒，随后静默观察 1.2 秒，再强制发零。", flush=True)
        while time.monotonic() - started < pulse_duration:
            publisher.publish(command)
            rclpy.spin_once(node, timeout_sec=0.02)
        last_command = time.monotonic()
        while time.monotonic() - last_command < quiet_duration:
            rclpy.spin_once(node, timeout_sec=0.02)
        observed = [sample for sample in samples if started <= sample[0] <= last_command + quiet_duration]
        moving = [sample for sample in observed if sample[1] >= 0.02]
        quiet = [sample for sample in observed if sample[0] >= last_command]
        stable_start = None
        stop_delay = None
        for stamp, speed, rotation in quiet:
            if speed < 0.01 and rotation < 0.08:
                if stable_start is None:
                    stable_start = stamp
                if stamp - stable_start >= 0.2:
                    stop_delay = max(0.0, stable_start - last_command)
                    break
            else:
                stable_start = None
        report = {
            "verified": bool(moving) and stop_delay is not None and len(quiet) >= 5,
            "test": "command_silence_with_agent_running",
            "does_not_verify": "USB_disconnection_or_agent_crash",
            "pulse_duration": pulse_duration,
            "quiet_duration": quiet_duration,
            "stop_delay_seconds": stop_delay,
            "peak_speed": max((sample[1] for sample in observed), default=0),
            "samples": [{"seconds_after_last_command": stamp - last_command,
                         "linear_speed": speed, "angular_speed": rotation}
                        for stamp, speed, rotation in observed],
        }
    finally:
        start_event.set()
        deadline = time.monotonic() + 1
        while rclpy.ok() and time.monotonic() < deadline:
            publisher.publish(Twist())
            rclpy.spin_once(node, timeout_sec=0.02)
        guard.join(timeout=3)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2) + "\n")
        print(json.dumps({key: value for key, value in report.items() if key != "samples"}, indent=2))
        if guard.is_alive():
            raise RuntimeError("停车进程未退出，请人工切断电机电源")


if __name__ == "__main__":
    main()
