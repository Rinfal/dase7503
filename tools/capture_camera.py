#!/usr/bin/env python3
import argparse
import time
from pathlib import Path

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


def main():
    parser = argparse.ArgumentParser(description="等待实际 ROS 摄像头图像并保存一帧，不驱动机器人。")
    parser.add_argument("--topic", default="/camera/image_raw")
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--timeout", default=15.0, type=float)
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = Node("camera_snapshot")
    bridge = CvBridge()
    received = []

    def image(message):
        received.append(message)
        if len(received) > 2:
            del received[:-2]

    node.create_subscription(Image, args.topic, image, qos_profile_sensor_data)
    deadline = time.monotonic() + args.timeout
    warmup_deadline = None
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if received and warmup_deadline is None:
                warmup_deadline = time.monotonic() + 1.5
            if warmup_deadline is not None and time.monotonic() >= warmup_deadline:
                break
        if not received:
            raise RuntimeError("未收到实际摄像头图像：" + args.topic)
        message = received[-1]
        frame = bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        args.output.parent.mkdir(parents=True, exist_ok=True)
        if not cv2.imwrite(str(args.output), frame):
            raise OSError("保存摄像头图像失败")
        print(f"已保存实际图像 {message.width}x{message.height}: {args.output.resolve()}")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
