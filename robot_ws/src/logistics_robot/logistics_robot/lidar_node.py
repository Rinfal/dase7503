import json
import math
import time

import rclpy
import serial
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from .lidar import PacketParser, ScanAssembler


class LidarNode(Node):
    def __init__(self):
        super().__init__("lidar_ms200p")
        port = self.declare_parameter("port", "").value
        if not port:
            raise ValueError("必须指定已确认属于雷达的串口；禁止自动选择 ESP32 串口")
        self.frame_id = self.declare_parameter("frame_id", "laser").value
        self.serial = serial.Serial(port=None, baudrate=230400, timeout=0, exclusive=True)
        self.serial.dtr = False
        self.serial.rts = False
        self.serial.port = port
        self.serial.open()
        self.parser = PacketParser()
        self.assembler = ScanAssembler()
        self.publisher = self.create_publisher(LaserScan, "scan", qos_profile_sensor_data)
        self.status = self.create_publisher(String, "lidar/status", 10)
        self.create_timer(0.005, self.read)
        self.create_timer(1.0, self.report)
        self.last_scan = None
        self.failed = False

    def read(self):
        if self.failed:
            return
        try:
            data = self.serial.read(min(self.serial.in_waiting, 8192))
        except serial.SerialException as error:
            self.get_logger().error(str(error))
            self.failed = True
            return
        now = time.monotonic()
        for packet in self.parser.feed(data):
            for scan in self.assembler.add(packet, now):
                message = LaserScan()
                message.header.frame_id = self.frame_id
                message.header.stamp = (self.get_clock().now() - Duration(
                    seconds=max(0.0, time.monotonic() - scan.started))).to_msg()
                message.angle_min = 0.0
                message.angle_increment = 2 * math.pi / len(scan.ranges)
                message.angle_max = message.angle_increment * (len(scan.ranges) - 1)
                message.time_increment = 0.0
                message.scan_time = scan.duration
                message.range_min = 0.03
                message.range_max = 12.0
                message.ranges = list(scan.ranges)
                message.intensities = list(scan.intensities)
                self.publisher.publish(message)
                self.last_scan = now

    def report(self):
        status = {"valid_packets": self.parser.valid_packets,
                  "bad_packets": self.parser.bad_packets, "failed": self.failed,
                  "scan_age": None if self.last_scan is None else time.monotonic() - self.last_scan}
        self.status.publish(String(data=json.dumps(status)))

    def destroy_node(self):
        self.serial.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = LidarNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
