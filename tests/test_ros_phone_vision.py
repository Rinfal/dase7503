"""Compressed camera -> QR process -> web/video, isolated from real hardware."""
import http.client
import importlib.util
import json
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'robot_ws/src/logistics_robot'))


@unittest.skipUnless(importlib.util.find_spec('rclpy'), '需要ROS2环境')
class PhoneVisionIntegration(unittest.TestCase):
    def test_compressed_camera_qr_preview_and_evidence(self):
        import cv2
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import CompressedImage
        from logistics_robot.qr_node import QRNode
        from logistics_robot.web_node import WebNode
        from test_vision import qr_frame

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root/'profile.yaml').write_text('{}')
            (root/'password').write_text('isolated-camera-test-password')
            rclpy.init(args=['--ros-args', '-p', 'hardware_profile:='+str(root/'profile.yaml'),
                '-p', 'password_file:='+str(root/'password'), '-p', 'evidence_dir:='+str(root/'evidence'),
                '-p', 'host:=127.0.0.1', '-p', 'port:=0',
                '-p', 'compressed_preview_topic:=/qr/preview/compressed', '-p', 'compressed_input:=true'], domain_id=230)
            web, scanner, camera = WebNode(), QRNode(), Node('isolated_camera')
            publisher = camera.create_publisher(CompressedImage, '/camera/image_raw/compressed',
                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
            encoded = cv2.imencode('.jpg', qr_frame('RACKB_AM9L'))[1].tobytes()

            def publish():
                msg = CompressedImage()
                msg.header.stamp = camera.get_clock().now().to_msg()
                msg.format = 'jpeg'; msg.data = encoded
                publisher.publish(msg)

            camera.create_timer(.1, publish)
            executor = SingleThreadedExecutor()
            for node in (web, scanner, camera):
                executor.add_node(node)
            thread = threading.Thread(target=executor.spin)
            thread.start()
            connection = http.client.HTTPConnection('127.0.0.1', web.server.server_port, timeout=3)
            cookie = ''

            def request(path, body=None):
                nonlocal cookie
                connection.request('GET' if body is None else 'POST', path,
                    None if body is None else json.dumps(body),
                    {'Content-Type': 'application/json', 'Cookie': cookie})
                response = connection.getresponse()
                data = response.read()
                if response.getheader('Set-Cookie'):
                    cookie = response.getheader('Set-Cookie').split(';')[0]
                return response.status, data

            try:
                self.assertEqual(request('/api/login', {'password':'isolated-camera-test-password'})[0], 200)
                deadline = time.monotonic()+8
                records = []
                while time.monotonic()<deadline:
                    records = json.loads(request('/api/evidence')[1])
                    if records:
                        break
                    time.sleep(.1)
                self.assertEqual(len(records), 1)
                self.assertEqual(records[0]['payload'], 'RACKB_AM9L')
                code, image = request(records[0]['url'])
                self.assertEqual(code, 200)
                self.assertTrue(image.startswith(b'\x89PNG'))
                code, image = request('/api/frame')
                self.assertEqual(code, 200)
                self.assertTrue(image.startswith(b'\xff\xd8'))
                status = json.loads(request('/api/status')[1])
                self.assertFalse(status['armed'])
                self.assertLess(status['camera_age'], 1)
                self.assertEqual(web.count_publishers('/wheel_pwm'), 0)
            finally:
                connection.close()
                executor.shutdown(timeout_sec=2)
                thread.join(timeout=2)
                web.close()
                for node in (web, scanner, camera):
                    executor.remove_node(node)
                    node.destroy_node()
                rclpy.shutdown()
