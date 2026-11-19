#!/usr/bin/env python3
import argparse
import json
import multiprocessing
import os
import time
from pathlib import Path


def stop_guard(topic, start_event, ready_event, duration):
    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.node import Node

    rclpy.init()
    node = Node("bench_independent_stop")
    publisher = node.create_publisher(Twist, topic, 10)
    ready_event.set()
    start_event.wait(timeout=15.0)
    deadline = time.monotonic() + duration + 0.1
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.01)
    stop_deadline = time.monotonic() + 1.5
    while rclpy.ok() and time.monotonic() < stop_deadline:
        publisher.publish(Twist())
        rclpy.spin_once(node, timeout_sec=0.03)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description="仅供四轮悬空、有人看护的短脉冲测试，不是地面遥控。")
    parser.add_argument("--topic", required=True)
    parser.add_argument("--controller-node", required=True)
    parser.add_argument("--axis", choices=["forward", "back", "left", "right", "ccw", "cw"], required=True)
    parser.add_argument("--duration", type=float, default=0.25)
    parser.add_argument("--linear-speed", type=float, default=0.03)
    parser.add_argument("--angular-speed", type=float, default=0.15)
    parser.add_argument("--report", type=Path)
    parser.add_argument("--countdown", type=float, default=0.0)
    parser.add_argument("--wheels-raised", action="store_true")
    args = parser.parse_args()
    if not args.wheels_raised or not 0.05 <= args.duration <= 15.0:
        parser.error("必须已确认四轮悬空，持续时间限制为 0.05..15.0 秒")
    if not 0 <= args.countdown <= 5:
        parser.error("准备时间限制为 0..5 秒")
    if not 0.01 <= args.linear_speed <= 0.08:
        parser.error("悬空线速度限制为 0.01..0.08 米/秒")
    if not 0.05 <= args.angular_speed <= 0.4:
        parser.error("悬空角速度限制为 0.05..0.4 弧度/秒")
    if not args.topic.startswith("/"):
        parser.error("必须指定从真实 ROS 图确认的绝对控制话题")
    if os.environ.get("ROS_DOMAIN_ID") != "20":
        parser.error("当前已确认控制板使用 ROS_DOMAIN_ID=20")

    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data

    context = multiprocessing.get_context("spawn")
    start_event = context.Event()
    ready_event = context.Event()
    guard = context.Process(target=stop_guard, args=(args.topic, start_event, ready_event, args.duration))
    guard.start()
    rclpy.init()
    node = Node("bench_short_pulse")
    publisher = node.create_publisher(Twist, args.topic, 10)
    samples = []

    def receive(message):
        velocity = message.twist.twist
        samples.append({"monotonic": time.monotonic(), "forward": velocity.linear.x,
                        "lateral": velocity.linear.y, "angular": velocity.angular.z})

    node.create_subscription(Odometry, "/odom_raw", receive, qos_profile_sensor_data)
    try:
        if not ready_event.wait(timeout=3.0):
            raise RuntimeError("独立停车进程未就绪，禁止运动")
        discovery_deadline = time.monotonic() + 3.0
        verified = False
        while time.monotonic() < discovery_deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            subscriptions = node.get_subscriptions_info_by_topic(args.topic)
            verified = any(info.node_name == args.controller_node
                           and info.topic_type == "geometry_msgs/msg/Twist" for info in subscriptions)
            if verified:
                break
        if not verified:
            raise RuntimeError("未找到指定控制板的 Twist 订阅，禁止运动")
        if args.countdown:
            print(f"已就绪，{args.countdown:.0f} 秒后开始；请观察轮子，不要触碰机构。", flush=True)
            ready_deadline = time.monotonic() + args.countdown
            while rclpy.ok() and time.monotonic() < ready_deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
        moves = {
            "forward": (args.linear_speed, 0.0, 0.0), "back": (-args.linear_speed, 0.0, 0.0),
            "left": (0.0, args.linear_speed, 0.0), "right": (0.0, -args.linear_speed, 0.0),
            "ccw": (0.0, 0.0, args.angular_speed), "cw": (0.0, 0.0, -args.angular_speed),
        }
        command = Twist()
        command.linear.x, command.linear.y, command.angular.z = moves[args.axis]
        print(f"悬空测试：{args.axis}，最长 {args.duration:.2f} 秒，随后持续发零速度。", flush=True)
        start_event.set()
        deadline = time.monotonic() + args.duration
        while rclpy.ok() and time.monotonic() < deadline:
            publisher.publish(command)
            rclpy.spin_once(node, timeout_sec=0.02)
    finally:
        start_event.set()
        stop_deadline = time.monotonic() + 1.0
        while rclpy.ok() and time.monotonic() < stop_deadline:
            publisher.publish(Twist())
            rclpy.spin_once(node, timeout_sec=0.02)
        guard.join(timeout=3.0)
        if guard.is_alive():
            raise RuntimeError("独立停车进程尚未退出，请人工切断电机电源并检查")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if args.report:
            args.report.parent.mkdir(parents=True, exist_ok=True)
            args.report.write_text(json.dumps({"axis": args.axis, "duration": args.duration,
                                              "linear_speed": args.linear_speed,
                                              "angular_speed": args.angular_speed,
                                              "samples": samples}, indent=2) + "\n")


if __name__ == "__main__":
    main()
