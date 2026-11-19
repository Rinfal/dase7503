import json
import time

import cv2
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from pyzbar.pyzbar import ZBarSymbol, decode
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

from .qr import DetectionGate, EvidenceStore


class QRNode(Node):
    def __init__(self):
        super().__init__("qr_scanner")
        directory = self.declare_parameter("evidence_dir", "~/dase7503_evidence").value
        confirmations = self.declare_parameter("confirmation_frames", 3).value
        self.max_rate = self.declare_parameter("max_rate", 10.0).value
        if not 0 < self.max_rate <= 60:
            raise ValueError("max_rate 必须在 0 到 60 Hz 之间")
        self.gate = DetectionGate(count=confirmations)
        self.store = EvidenceStore(directory)
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(String, "qr/detections", 10)
        self.preview = self.create_publisher(Image, "qr/annotated", 1)
        self.create_subscription(Image, "camera/image_raw", self.image, qos_profile_sensor_data)
        self.previous_frame = -float("inf")
        self.get_logger().info("QR scanner ready")

    def image(self, message):
        now = time.monotonic()
        if now - self.previous_frame < 1.0 / self.max_rate:
            return
        self.previous_frame = now
        try:
            frame = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        except CvBridgeError as error:
            self.get_logger().error(str(error))
            return
        decoded = decode(frame, symbols=[ZBarSymbol.QRCODE])
        payloads = []
        for result in decoded:
            try:
                payload = result.data.decode("utf-8", errors="strict")
            except UnicodeDecodeError:
                continue
            payloads.append(payload)
            bounds = result.rect
            cv2.rectangle(frame, (bounds.left, bounds.top),
                          (bounds.left + bounds.width, bounds.top + bounds.height), (0, 255, 0), 2)
            cv2.putText(frame, payload[:32], (bounds.left, max(24, bounds.top - 10)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        for code in self.gate.observe(payloads, now):
            saved = self.store.record(code, lambda filename: cv2.imwrite(filename, frame))
            event = {"payload": code.payload, "kind": code.kind,
                     "rack": code.rack, "source_stamp_sec": message.header.stamp.sec,
                     "source_stamp_nanosec": message.header.stamp.nanosec}
            self.publisher.publish(String(data=json.dumps(event)))
            if saved:
                self.get_logger().info("已保存二维码与截图：" + code.payload)
        if self.preview.get_subscription_count():
            annotated = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            annotated.header = message.header
            self.preview.publish(annotated)


def main(args=None):
    rclpy.init(args=args)
    node = QRNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
