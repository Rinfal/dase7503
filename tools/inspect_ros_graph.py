#!/usr/bin/env python3
import argparse
import json
import os
import time
from datetime import datetime, timezone
from pathlib import Path

import rclpy
from rclpy.node import Node


def endpoint(info):
    return {
        "node_name": info.node_name,
        "node_namespace": info.node_namespace,
        "topic_type": info.topic_type,
        "reliability": info.qos_profile.reliability.name,
        "durability": info.qos_profile.durability.name,
    }


def main():
    parser = argparse.ArgumentParser(description="只读取 ROS 图并保存接口清单，不创建运动发布者。")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--discovery-seconds", type=float, default=5.0)
    args, ros_args = parser.parse_known_args()
    if not 0.5 <= args.discovery_seconds <= 30:
        parser.error("发现等待时间必须在 0.5..30 秒之间")
    rclpy.init(args=ros_args)
    node = Node("readonly_interface_inspector", enable_rosout=False,
                start_parameter_services=False)
    try:
        deadline = time.monotonic() + args.discovery_seconds
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        nodes = []
        for name, namespace in sorted(node.get_node_names_and_namespaces()):
            if name == node.get_name() and namespace == node.get_namespace():
                continue
            nodes.append({
                "name": name,
                "namespace": namespace,
                "services": node.get_service_names_and_types_by_node(name, namespace),
            })
        topics = {}
        for name, types in sorted(node.get_topic_names_and_types()):
            topics[name] = {
                "types": types,
                "publishers": [endpoint(info) for info in node.get_publishers_info_by_topic(name)],
                "subscribers": [endpoint(info) for info in node.get_subscriptions_info_by_topic(name)],
            }
        report = {
            "observed_utc": datetime.now(timezone.utc).isoformat(),
            "ros_domain_id": os.environ.get("ROS_DOMAIN_ID", "0"),
            "nodes": nodes,
            "topics": topics,
        }
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
        print(json.dumps(report, ensure_ascii=False, indent=2))
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
