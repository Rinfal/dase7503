"""Real ROS/HTTP integration in an isolated domain, with a simulated board."""
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
class WebPWMIntegration(unittest.TestCase):
    def test_http_handshake_pwm_lift_and_client_timeout(self):
        import rclpy
        import yaml
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from std_msgs.msg import Bool, Int32, Int32MultiArray, String
        from logistics_robot.web_node import WebNode

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            flags = {k: True for k in ('wheel_directions_verified', 'forward_direction_verified',
                'backward_direction_verified', 'positive_yaw_direction_verified',
                'negative_yaw_direction_verified', 'command_watchdog_verified', 'web_drive_validated')}
            flags.update(firmware='dase-rc-0.5', motor_forward_pwm_signs=[-1,1,-1,1], verified_bench_running_pwm=600)
            profile = {'controller':flags, 'servo':{'safe_range_verified':True,
                       'calibrated_pulses_us':{'up':1540,'down':1500}}}
            (root/'profile.yaml').write_text(yaml.safe_dump(profile))
            (root/'password').write_text('isolated-test-password')
            rclpy.init(args=['--ros-args', '-p', 'hardware_profile:='+str(root/'profile.yaml'),
                '-p', 'password_file:='+str(root/'password'), '-p', 'evidence_dir:='+str(root/'evidence'),
                '-p', 'host:=127.0.0.1', '-p', 'port:=0'], domain_id=232)
            web = WebNode()
            board = Node('isolated_fake_board')
            status = {'firmware':'dase-rc-0.5','armed':False,'watchdog_timeout':False,
                'pwm':[0]*4,'encoders':[0]*4,'board_us':1000000,'servo_us':0,'servo_active':False}
            outputs, pulses = [], []

            def wheel(msg):
                outputs.append(list(msg.data))
                if status['armed']:
                    status['pwm'] = list(msg.data)

            def arm(msg):
                status['armed'] = msg.data
                if not msg.data:
                    status['pwm'] = [0]*4

            board.create_subscription(Int32MultiArray,'/wheel_pwm',wheel,qos_profile_sensor_data)
            board.create_subscription(Bool,'/drive_arm',arm,qos_profile_sensor_data)
            board.create_subscription(Int32,'/servo_s2_us',lambda msg:pulses.append(msg.data),1)
            pub = board.create_publisher(String,'/drive/status',qos_profile_sensor_data)

            def publish():
                status['board_us'] += 50000
                pub.publish(String(data=json.dumps(status)))

            board.create_timer(.05,publish)
            executor = SingleThreadedExecutor()
            executor.add_node(web)
            executor.add_node(board)
            thread = threading.Thread(target=executor.spin)
            thread.start()
            cookie = ''

            def request(path, body=None):
                nonlocal cookie
                conn = http.client.HTTPConnection('127.0.0.1',web.server.server_port,timeout=2)
                conn.request('GET' if body is None else 'POST',path,
                    body=None if body is None else json.dumps(body),
                    headers={'Content-Type':'application/json','Cookie':cookie})
                response = conn.getresponse()
                result = json.load(response)
                if response.getheader('Set-Cookie'):
                    cookie = response.getheader('Set-Cookie').split(';')[0]
                conn.close()
                self.assertEqual(response.status,200,result)
                return result

            def until(predicate):
                end = time.monotonic()+3
                while time.monotonic()<end:
                    if predicate():
                        return
                    time.sleep(.02)
                self.fail('integration timeout')

            try:
                request('/api/login',{'password':'isolated-test-password'})
                until(lambda:request('/api/status')['feedback_age'] is not None)
                lease = request('/api/arm',{})['lease']
                seq = 0

                def drive(forward):
                    nonlocal seq
                    seq += 1
                    request('/api/drive',{'lease':lease,'sequence':seq,'server_clock':time.monotonic(),'forward':forward})

                for _ in range(8):
                    drive(1)
                    time.sleep(.06)
                self.assertIn([-600,600,-600,600],outputs)
                for _ in range(6):
                    drive(0)
                    time.sleep(.06)
                seq += 1
                request('/api/lift',{'lease':lease,'sequence':seq,'server_clock':time.monotonic(),'direction':'up'})
                until(lambda:1540 in pulses)
                time.sleep(.6)
                self.assertFalse(request('/api/status')['armed'])
                until(lambda:not status['armed'] and status['pwm']==[0]*4)
            finally:
                executor.shutdown(timeout_sec=2)
                thread.join(timeout=2)
                web.close()
                executor.remove_node(web)
                executor.remove_node(board)
                web.destroy_node()
                board.destroy_node()
                rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
