import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


def main(args=None):
    rclpy.init(args=args)
    node = Node("keyboard_teleop")
    publisher = node.create_publisher(Twist, "cmd_vel_requested", 1)
    if not sys.stdin.isatty():
        node.destroy_node()
        rclpy.shutdown()
        raise RuntimeError("键盘遥控需要交互式终端")
    settings = termios.tcgetattr(sys.stdin)
    moves = {
        "w": (0.06, 0.0, 0.0), "s": (-0.06, 0.0, 0.0),
        "a": (0.0, 0.06, 0.0), "d": (0.0, -0.06, 0.0),
        "q": (0.0, 0.0, 0.25), "e": (0.0, 0.0, -0.25),
    }
    print("按住 W/S 前后、A/D 横移、Q/E 转向；空格停，X 退出。松键后最多 0.15 秒停车。")
    message = Twist()
    deadline = 0.0
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            if select.select([sys.stdin], [], [], 0.04)[0]:
                key = sys.stdin.read(1).lower()
                if key in ("x", "\x03"):
                    break
                message = Twist()
                if key in moves:
                    message.linear.x, message.linear.y, message.angular.z = moves[key]
                deadline = time.monotonic() + 0.15
            if time.monotonic() > deadline:
                message = Twist()
            publisher.publish(message)
            rclpy.spin_once(node, timeout_sec=0)
    finally:
        for _ in range(3):
            if not rclpy.ok():
                break
            publisher.publish(Twist())
            rclpy.spin_once(node, timeout_sec=0.02)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
