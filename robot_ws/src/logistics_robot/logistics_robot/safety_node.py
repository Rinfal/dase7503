import json
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger

from .safety import SafetyGate, SafetyLimits, Velocity


class SafetyNode(Node):
    def __init__(self):
        super().__init__("safety_gate")
        defaults = SafetyLimits()
        parameters = {
            key: self.declare_parameter(key, value).value
            for key, value in defaults.__dict__.items()
        }
        self.hardware_ready = self.declare_parameter("hardware_ready", False).value
        self.gate = SafetyGate(SafetyLimits(**parameters))
        self.publisher = self.create_publisher(Twist, "cmd_vel", 1)
        self.status = self.create_publisher(String, "safety/status", 10)
        self.create_subscription(Twist, "cmd_vel_requested", self.command, 1)
        self.create_subscription(LaserScan, "scan", self.scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, "odom", self.odom, qos_profile_sensor_data)
        self.create_subscription(Bool, "safety/estop", self.estop, 1)
        self.create_service(SetBool, "safety/set_estop", self.set_estop)
        self.create_service(Trigger, "safety/arm", self.arm)
        self.create_service(Trigger, "safety/disarm", self.disarm)
        self.create_timer(0.05, self.tick)
        self.previous_status = None
        self.previous_status_time = -float("inf")

    def command(self, message):
        self.gate.update_command(Velocity(message.linear.x, message.linear.y, message.angular.z), time.monotonic())

    def scan(self, message):
        age = (self.get_clock().now().nanoseconds - message.header.stamp.sec * 1000000000
               - message.header.stamp.nanosec) / 1e9
        if message.header.frame_id != "laser" or not -0.1 <= age <= self.gate.limits.scan_timeout:
            self.gate.scan_time = None
            return
        self.gate.update_scan(message.ranges, message.angle_increment, message.range_min,
                              message.range_max, time.monotonic())

    def odom(self, message):
        orientation = message.pose.pose.orientation
        values = (
            message.pose.pose.position.x, message.pose.pose.position.y,
            orientation.x, orientation.y, orientation.z, orientation.w,
            message.twist.twist.linear.x, message.twist.twist.linear.y,
            message.twist.twist.angular.z,
        )
        age = (self.get_clock().now().nanoseconds - message.header.stamp.sec * 1000000000
               - message.header.stamp.nanosec) / 1e9
        valid = (all(math.isfinite(value) for value in values)
                 and abs(sum(value * value for value in
                             (orientation.x, orientation.y, orientation.z, orientation.w)) - 1) < 0.05
                 and message.header.frame_id == "odom" and message.child_frame_id == "base_link"
                 and -0.1 <= age <= self.gate.limits.odom_timeout)
        self.gate.update_odom(valid, time.monotonic())

    def estop(self, message):
        if message.data:
            self.gate.emergency_stop(True)
            self.publisher.publish(Twist())

    def set_estop(self, request, response):
        self.gate.emergency_stop(request.data)
        self.publisher.publish(Twist())
        response.success = True
        response.message = "急停已锁定" if request.data else "急停解除；仍需显式解锁运动"
        return response

    def arm(self, request, response):
        if not self.hardware_ready:
            response.success = False
            response.message = "hardware_ready=false：尚未完成硬件标定"
        else:
            response.success, response.message = self.gate.arm(time.monotonic())
        return response

    def disarm(self, request, response):
        self.gate.disarm()
        self.publisher.publish(Twist())
        response.success = True
        response.message = "运动已锁定"
        return response

    def tick(self):
        velocity = self.gate.output(time.monotonic())
        message = Twist()
        message.linear.x = velocity.forward
        message.linear.y = velocity.lateral
        message.angular.z = velocity.angular
        self.publisher.publish(message)
        status = json.dumps({"armed": self.gate.armed, "estop": self.gate.estop,
                             "reason": self.gate.reason})
        now = time.monotonic()
        if status != self.previous_status or now - self.previous_status_time >= 0.25:
            self.status.publish(String(data=status))
            if status != self.previous_status:
                self.get_logger().info(status)
            self.previous_status = status
            self.previous_status_time = now


def main(args=None):
    rclpy.init(args=args)
    node = SafetyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.publisher.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
