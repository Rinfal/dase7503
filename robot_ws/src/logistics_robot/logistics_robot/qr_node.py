import json
import time

import cv2
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String

from .vision import VisionProcessor


class QRNode(Node):
    def __init__(self):
        super().__init__('qr_scanner')
        directory = self.declare_parameter('evidence_dir', '~/dase7503_evidence').value
        confirmations = self.declare_parameter('confirmation_frames', 3).value
        rate = self.declare_parameter('max_rate', 5.0).value
        preview_rate = self.declare_parameter('preview_rate', 12.0).value
        width = self.declare_parameter('preview_width', 960).value
        quality = self.declare_parameter('jpeg_quality', 70).value
        compressed_input = self.declare_parameter('compressed_input', False).value
        if not 1 <= rate <= 15 or not rate <= preview_rate <= 30:
            raise ValueError('扫码频率需在1..15Hz，视频频率需在扫码频率..30Hz')
        if not 320 <= width <= 1920 or not 30 <= quality <= 90:
            raise ValueError('无效预览尺寸或JPEG质量')
        self.processor = VisionProcessor(directory, confirmations, rate, width, quality)
        cv2.setNumThreads(1)
        self.bridge = CvBridge()
        self.period = 1/preview_rate
        self.previous_frame = -float('inf')
        self.publisher = self.create_publisher(String, '/qr/detections', 10)
        latest = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.compressed = self.create_publisher(CompressedImage, '/qr/preview/compressed', latest)
        self.preview = self.create_publisher(Image, '/qr/annotated', latest)
        self.compressed_input = compressed_input
        self.create_subscription(CompressedImage if compressed_input else Image,
            '/camera/image_raw/compressed' if compressed_input else '/camera/image_raw', self.image, latest)
        self.get_logger().info(f'自动扫码 {rate}Hz，视频上限 {preview_rate}fps；连续{confirmations}帧确认')

    def image(self, message):
        now = time.monotonic()
        if now-self.previous_frame < self.period:
            return
        source_ns = message.header.stamp.sec*1_000_000_000 + message.header.stamp.nanosec
        if source_ns and not 0 <= (self.get_clock().now().nanoseconds-source_ns)/1e9 < .6:
            return
        self.previous_frame = now
        try:
            frame = (self.bridge.compressed_imgmsg_to_cv2(message, desired_encoding='bgr8')
                if self.compressed_input else self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8'))
            jpeg, events, annotated = self.processor.process(frame, now)
            msg = CompressedImage()
            msg.header = message.header
            msg.format = 'jpeg'
            msg.data = jpeg
            self.compressed.publish(msg)
            for event in events:
                event.update(source_stamp_sec=message.header.stamp.sec,
                             source_stamp_nanosec=message.header.stamp.nanosec)
                self.publisher.publish(String(data=json.dumps(event, ensure_ascii=False)))
                if event.get('new_evidence'):
                    self.get_logger().info('二维码已识别并留证')
            if self.preview.get_subscription_count():
                msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
                msg.header = message.header
                self.preview.publish(msg)
        except (CvBridgeError, cv2.error, ValueError, OSError) as error:
            self.get_logger().warning(str(error), throttle_duration_sec=10)


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = QRNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
