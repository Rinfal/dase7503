import importlib.util
import math
import sys
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "robot_ws/src/logistics_robot"))


@unittest.skipUnless(importlib.util.find_spec("rclpy"), "需要先 source ROS 2 Jazzy 环境")
class ROSSafetyTests(unittest.TestCase):
    def test_namespaced_sensor_command_and_watchdog_chain(self):
        import rclpy
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import LaserScan
        from std_srvs.srv import SetBool, Trigger

        from logistics_robot.safety_node import SafetyNode

        rclpy.init(args=["--ros-args", "-r", "__ns:=/isolated_safety_verification",
                        "-p", "hardware_ready:=true"], domain_id=231)
        safety = SafetyNode()
        tester = Node("tester")
        executor = SingleThreadedExecutor()
        executor.add_node(safety)
        executor.add_node(tester)
        outputs = []
        tester.create_subscription(Twist, "cmd_vel", outputs.append, 10)
        scan_publisher = tester.create_publisher(LaserScan, "scan", qos_profile_sensor_data)
        odom_publisher = tester.create_publisher(Odometry, "odom", qos_profile_sensor_data)
        command_publisher = tester.create_publisher(Twist, "cmd_vel_requested", 1)
        release = tester.create_client(SetBool, "safety/set_estop")
        arm = tester.create_client(Trigger, "safety/arm")

        def spin_until(condition, timeout=2):
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.01)
                if condition():
                    return
            self.fail("ROS 集成测试等待超时")

        def fresh_sensors():
            scan = LaserScan()
            scan.header.frame_id = "laser"
            scan.header.stamp = tester.get_clock().now().to_msg()
            scan.angle_increment = math.pi / 180
            scan.range_min = 0.05
            scan.range_max = 20.0
            scan.ranges = [2.0] * 360
            scan_publisher.publish(scan)
            odom = Odometry()
            odom.header.frame_id = "odom"
            odom.header.stamp = tester.get_clock().now().to_msg()
            odom.child_frame_id = "base_link"
            odom.pose.pose.orientation.w = 1.0
            odom_publisher.publish(odom)

        try:
            self.assertEqual(safety.publisher.topic_name, "/isolated_safety_verification/cmd_vel")
            self.assertEqual(command_publisher.topic_name, "/isolated_safety_verification/cmd_vel_requested")
            spin_until(lambda: scan_publisher.get_subscription_count() > 0
                       and odom_publisher.get_subscription_count() > 0
                       and command_publisher.get_subscription_count() > 0
                       and safety.publisher.get_subscription_count() > 0
                       and release.service_is_ready() and arm.service_is_ready())
            self.assertTrue(all(message.linear.x == 0 for message in outputs))
            response = release.call_async(SetBool.Request(data=False))
            spin_until(response.done)
            self.assertTrue(response.result().success)
            tester.create_timer(0.05, fresh_sensors)
            fresh_sensors()
            spin_until(lambda: safety.gate.scan_time is not None and safety.gate.odom_time is not None)
            response = arm.call_async(Trigger.Request())
            spin_until(response.done)
            self.assertTrue(response.result().success)
            command = Twist()
            command.linear.x = 0.05
            command_publisher.publish(command)
            spin_until(lambda: any(message.linear.x > 0 for message in outputs))
            spin_until(lambda: not safety.gate.armed)
            self.assertEqual(safety.gate.reason, "command_stale")
            spin_until(lambda: outputs[-1].linear.x == 0)
        finally:
            executor.remove_node(safety)
            executor.remove_node(tester)
            executor.shutdown()
            safety.destroy_node()
            tester.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    unittest.main()
