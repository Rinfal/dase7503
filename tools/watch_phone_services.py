#!/usr/bin/env python3
"""Read-only ROS health checks; restart only the failing local service."""
import json
import os
import socket
import subprocess
import time
import urllib.request
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

from logistics_robot.recovery import RecoveryPolicy
from logistics_robot.durability import atomic_write


class Watcher(Node):
    def __init__(self):
        super().__init__('phone_recovery')
        self.policy = RecoveryPolicy()
        self.seen, self.clocks = {}, {}
        self.started = time.monotonic()
        self.report = Path(os.environ.get('XDG_RUNTIME_DIR', f'/run/user/{os.getuid()}'))/'dase7503-health.json'
        self.port = Path('/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0')
        self.http = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        self.create_subscription(CompressedImage, '/camera/image_raw/compressed',
            lambda msg: self.frame('camera', msg), qos_profile_sensor_data)
        self.create_subscription(CompressedImage, '/qr/preview/compressed',
            lambda msg: self.frame('vision', msg), qos_profile_sensor_data)
        self.create_subscription(String, '/drive/status', self.board, qos_profile_sensor_data)
        self.create_timer(5.0, self.check)

    def observe(self, kind, clock):
        if clock and clock != self.clocks.get(kind):
            self.clocks[kind] = clock
            self.seen[kind] = time.monotonic()

    def frame(self, kind, msg):
        # camera_ros labels JPEG data as "jpg", while QR preview uses "jpeg".
        # Check the bytes so both producers are monitored correctly.
        if len(msg.data) > 2 and msg.data[0] == 255 and msg.data[1] == 216:
            self.observe(kind, (msg.header.stamp.sec, msg.header.stamp.nanosec))

    def board(self, msg):
        try:
            data = json.loads(msg.data)
            if str(data.get('firmware', '')).startswith('dase-rc-'):
                self.observe('agent', data.get('board_us'))
        except (ValueError, AttributeError):
            pass

    def check(self):
        now = time.monotonic()
        notify = os.environ.get('NOTIFY_SOCKET')
        if notify:
            with socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM) as sock:
                sock.connect('\0'+notify[1:] if notify.startswith('@') else notify)
                sock.sendall(b'WATCHDOG=1')
        atomic_write(self.report, json.dumps({'monotonic':now,
            'age':{kind:now-stamp for kind,stamp in self.seen.items()},
            'last_restart':self.policy.last_restart}).encode())
        if now-self.started < self.policy.grace:
            return
        try:
            result = subprocess.run(['systemctl', '--user', 'show',
                *[f'dase7503-{kind}.service' for kind in ('agent', 'camera', 'vision', 'remote')],
                '-p', 'Id', '-p', 'ActiveState', '-p', 'ActiveEnterTimestampMonotonic'],
                capture_output=True, text=True, timeout=3, check=True)
            units = {}
            for block in result.stdout.strip().split('\n\n'):
                props = dict(line.split('=', 1) for line in block.splitlines() if '=' in line)
                kind = props['Id'].removeprefix('dase7503-').removesuffix('.service')
                units[kind] = (props['ActiveState'], int(props['ActiveEnterTimestampMonotonic'])/1e6)
            try:
                with self.http.open('http://10.42.50.1:8080/healthz', timeout=2) as response:
                    data = json.load(response)
                web_ok = data.get('tick_age') is not None and 0 <= data['tick_age'] < 2
            except (OSError, ValueError):
                web_ok = False
            for kind in self.policy.choose(time.monotonic(), units, self.seen, web_ok, self.port.exists()):
                self.get_logger().warning(f'{kind}数据停滞，自动重启该服务')
                subprocess.run(['systemctl', '--user', '--no-block', 'restart',
                    f'dase7503-{kind}.service'], timeout=3, check=True)
        except (subprocess.SubprocessError, ValueError, KeyError) as error:
            self.get_logger().warning(f'健康检查失败: {type(error).__name__}', throttle_duration_sec=60)


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Watcher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
