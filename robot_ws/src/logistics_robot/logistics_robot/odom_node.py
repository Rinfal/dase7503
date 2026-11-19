import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import TransformBroadcaster


class OdomNode(Node):
    def __init__(self):
        super().__init__("odom_tf")
        self.input_frame = self.declare_parameter("input_frame", "odom").value
        self.input_child = self.declare_parameter("input_child", "base_link").value
        self.broadcaster = TransformBroadcaster(self)
        self.publisher = self.create_publisher(Odometry, "odom", 10)
        self.create_subscription(Odometry, "odom_raw", self.receive, qos_profile_sensor_data)
        self.get_logger().info("Odometry bridge ready")

    def receive(self, message):
        if message.header.frame_id != self.input_frame or message.child_frame_id != self.input_child:
            return
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        values = (position.x, position.y, position.z, orientation.x,
                  orientation.y, orientation.z, orientation.w)
        if not all(math.isfinite(value) for value in values):
            return
        if abs(sum(value * value for value in values[3:]) - 1) > 0.05:
            return
        age = (self.get_clock().now().nanoseconds
               - message.header.stamp.sec * 1000000000 - message.header.stamp.nanosec) / 1e9
        if not -0.1 <= age <= 0.3:
            return
        message.header.frame_id = "odom"
        message.child_frame_id = "base_link"
        transform = TransformStamped()
        transform.header = message.header
        transform.child_frame_id = "base_link"
        transform.transform.translation.x = position.x
        transform.transform.translation.y = position.y
        transform.transform.translation.z = position.z
        transform.transform.rotation = orientation
        self.publisher.publish(message)
        self.broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = OdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
