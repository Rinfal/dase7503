import math
import json
import os
import secrets
import threading
import time
from pathlib import Path

import cv2
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Bool, Int32, Int32MultiArray, String, UInt16

from .web_server import RobotHTTPServer, WebState
from .drive_pwm import DrivePWM
from .pwm_bridge import PWMBridge, decode_status, wheel_pwm_limit


class WebNode(Node):
    def __init__(self):
        super().__init__("phone_remote")
        share = Path(get_package_share_directory("logistics_robot"))
        profile_path = self.declare_parameter("hardware_profile", str(share / "config/hardware_profile.yaml")).value
        password_path = Path(self.declare_parameter("password_file", "~/.config/dase7503/remote-password").value).expanduser()
        evidence_dir = self.declare_parameter("evidence_dir", "~/dase7503_evidence").value
        host = self.declare_parameter("host", "0.0.0.0").value
        port = self.declare_parameter("port", 8080).value
        password_path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
        try:
            fd = os.open(password_path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
        except FileExistsError:
            pass
        else:
            with os.fdopen(fd, "w") as stream:
                stream.write(secrets.token_urlsafe(12) + "\n")
        password = password_path.read_text().strip()
        if not password:
            raise ValueError("访问口令不能为空")
        profile = yaml.safe_load(Path(profile_path).read_text())
        self.state = WebState(password, profile, evidence_dir)
        self.firmware = profile.get('controller', {}).get('firmware', '')
        self.pwm_mode = self.firmware.startswith('dase-rc-')
        self.pwm_bridge = None
        self.wheels = self.arm_pub = None
        self.bridge = CvBridge()
        self.last_frame = -float("inf")
        if self.pwm_mode:
            controller = profile['controller']
            if self.state.control.drive_enabled:
                lateral_duty = controller.get('lateral_pwm', controller['verified_bench_running_pwm'])
                forward_duty = controller.get('forward_pwm', controller['verified_bench_running_pwm'])
                start_duty = controller.get('lateral_start_pwm')
                for value in (forward_duty, lateral_duty, *(() if start_duty is None else (start_duty,))):
                    if type(value) is not int or not 1 <= value <= wheel_pwm_limit(self.firmware):
                        raise ValueError('Drive PWM exceeds the configured firmware limit')
                self.pwm_bridge = PWMBridge(DrivePWM(controller['motor_forward_pwm_signs'],
                    duty=controller['verified_bench_running_pwm'],
                    lateral_enabled=self.state.control.lateral_enabled,
                    lateral_duty=lateral_duty, forward_duty=forward_duty),
                    lateral_start_duty=start_duty,
                    lateral_start_seconds=controller.get('lateral_start_seconds', .18))
                self.wheels = self.create_publisher(Int32MultiArray, '/wheel_pwm', qos_profile_sensor_data)
                self.arm_pub = self.create_publisher(Bool, '/drive_arm', qos_profile_sensor_data)
            self.create_subscription(String, '/drive/status', self.board_feedback, qos_profile_sensor_data)
        else:
            self.create_subscription(Odometry, '/odom_raw', self.feedback, qos_profile_sensor_data)
            self.create_subscription(UInt16, '/battery', self.battery, qos_profile_sensor_data)
        self.create_subscription(String, "/qr/detections", self.qr, 10)
        compressed_topic = self.declare_parameter('compressed_preview_topic', '').value
        if compressed_topic:
            self.create_subscription(CompressedImage, compressed_topic, self.compressed_image,
                                     qos_profile_sensor_data)
        else:
            self.create_subscription(Image, "/camera/image_raw", self.image, qos_profile_sensor_data)
        # A read-only dashboard must not compete with bench tests by publishing zeros.
        self.velocity = (self.create_publisher(Twist, "/cmd_vel", 1)
                         if self.state.control.drive_enabled and not self.pwm_mode else None)
        self.servo = (self.create_publisher(Int32, '/servo_s2_us' if self.pwm_mode else '/servo_s2', 1)
                      if self.state.control.lift_angles else None)
        self.create_timer(0.02, self.tick)
        self.server = RobotHTTPServer((host, port), self.state)
        self.http_thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.http_thread.start()
        self.get_logger().info(f"手机界面已启动，端口 {self.server.server_port}；口令保存在 {password_path}")
        self.get_logger().info("运动许可：" + str(self.state.control.drive_enabled))

    def feedback(self, msg):
        v = msg.twist.twist
        valid = all(math.isfinite(x) for x in (v.linear.x, v.linear.y, v.angular.z))
        with self.state.lock:
            self.state.control.gate.update_feedback(valid, time.monotonic())

    def board_feedback(self, msg):
        now = time.monotonic()
        with self.state.lock:
            try:
                status = decode_status(msg.data, self.firmware)
                valid, reboot = self.pwm_bridge.feedback(status, now) if self.pwm_bridge else (True, False)
                if reboot:
                    self.state.control.stop()
                self.state.control.gate.update_feedback(valid, now)
                self.state.battery = status.get('battery_adc_raw')
                self.state.board_status = status
            except (ValueError, TypeError, KeyError):
                self.state.control.gate.update_feedback(False, now)
                self.state.control.stop()

    def battery(self, msg):
        with self.state.lock:
            self.state.battery = msg.data

    def qr(self, msg):
        import json
        try:
            event = json.loads(msg.data)
            if isinstance(event, dict):
                with self.state.lock:
                    self.state.qr_events.append(event)
        except ValueError:
            pass

    def image(self, msg):
        now = time.monotonic()
        if now - self.last_frame < 0.15:
            return
        self.last_frame = now
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            frame = cv2.resize(frame, (960, 540))
            success, encoded = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 75])
            if success:
                self.state.set_frame(encoded.tobytes(), now)
        except (CvBridgeError, cv2.error) as error:
            self.get_logger().warning(str(error), throttle_duration_sec=10)

    def compressed_image(self, msg):
        if 'jpeg' not in msg.format.lower() or not msg.data:
            return
        source_ns = msg.header.stamp.sec*1_000_000_000 + msg.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds-source_ns)/1e9 if source_ns else 0
        if not 0 <= age < 1:
            return
        self.state.set_frame(msg.data, time.monotonic()-age)

    def tick(self):
        with self.state.lock:
            command = self.state.control.output(time.monotonic())
            if self.pwm_bridge:
                if self.count_publishers('/wheel_pwm') > 1 or self.count_publishers('/drive_arm') > 1:
                    self.state.control.stop()
                pwm, arm, pulse = self.pwm_bridge.step(self.state.control, command, time.monotonic())
                self.wheels.publish(Int32MultiArray(data=pwm))
                if arm is not None:
                    self.arm_pub.publish(Bool(data=arm))
                if pulse is not None and self.servo:
                    self.servo.publish(Int32(data=pulse))
            if self.velocity:
                msg = Twist()
                msg.linear.x, msg.linear.y, msg.angular.z = command.forward, command.lateral, command.angular
                self.velocity.publish(msg)
            if self.servo and not self.pwm_mode and self.state.control.pending_lift is not None:
                self.servo.publish(Int32(data=self.state.control.pending_lift))
                self.state.control.pending_lift = None
            self.state.tick_time = time.monotonic()

    def close(self):
        with self.state.lock:
            self.state.control.stop()
        if rclpy.ok():
            self.tick()
        self.server.shutdown()
        self.server.server_close()
        self.http_thread.join(timeout=2)


def main(args=None):
    # Let Python deliver KeyboardInterrupt before invalidating the ROS context,
    # so close() can still publish its final neutral/locked output.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = WebNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
