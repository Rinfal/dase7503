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
    def test_web_main_sigint_exits_cleanly_with_live_ros_context(self):
        import os
        import select
        import signal
        import subprocess
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            (root/'profile').write_text('{}')
            (root/'password').write_text('shutdown-test-password')
            process=subprocess.Popen([sys.executable,'-c',
                'from logistics_robot.web_node import main; main()', '--ros-args',
                '-p','hardware_profile:='+str(root/'profile'),
                '-p','password_file:='+str(root/'password'),
                '-p','evidence_dir:='+str(root/'evidence'),'-p','host:=127.0.0.1','-p','port:=0'],
                env=dict(os.environ,ROS_DOMAIN_ID='229',
                    PYTHONPATH=str(Path(__file__).resolve().parents[1]/'robot_ws/src/logistics_robot')+
                        os.pathsep+os.environ.get('PYTHONPATH','')),
                stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
            output=''
            try:
                deadline=time.monotonic()+20
                while time.monotonic()<deadline:
                    if select.select([process.stdout],[],[],.2)[0]:
                        output+=process.stdout.readline()
                        if '手机界面已启动' in output:break
                    if process.poll() is not None:break
                self.assertIn('手机界面已启动',output)
                process.send_signal(signal.SIGINT)
                output+=process.communicate(timeout=6)[0]
                self.assertEqual(process.returncode,0,output)
                self.assertNotIn('Traceback',output)
            finally:
                if process.poll() is None:
                    process.kill();process.communicate()

    def test_http_handshake_pwm_lift_and_client_timeout(self):
        self.exercise(False)

    def test_direct_http_without_unlock_and_automatic_recovery(self):
        self.exercise(True)

    def test_manual_unlock_survives_idle_lift_and_stop(self):
        self.exercise(False, manual=True)

    def exercise(self, direct, manual=False):
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
            flags['direct_control'] = direct
            flags['manual_lock_control'] = manual
            flags['manual_lateral_authorized'] = manual
            flags['wheel_type'] = 'mecanum'
            profile = {'controller':flags, 'servo':{'safe_range_verified':True,
                       'calibrated_pulses_us':{'up':1540,'down':1500}}}
            if manual:
                profile['servo'] = {'manual_lift_authorized':True, 'manual_pulses_us':{'up':750,'down':300}}
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
            version = 0

            def request(path, body=None):
                nonlocal cookie, version
                if manual and body is not None:
                    version += 1
                    body = dict(body, control_version=version)
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
                lease = '' if direct else request('/api/arm',{})['lease']
                seq = 0

                def drive(forward):
                    nonlocal seq
                    seq += 1
                    request('/api/drive', {'forward':forward} if direct or manual else
                        {'lease':lease,'sequence':seq,'server_clock':time.monotonic(),'forward':forward})

                for _ in range(8):
                    drive(1)
                    time.sleep(.06)
                self.assertIn([-600,600,-600,600],outputs)
                if direct:
                    time.sleep(.6)
                    self.assertFalse(request('/api/status')['armed'])
                    until(lambda:not status['armed'] and status['pwm']==[0]*4)
                    count = len(outputs)
                    for _ in range(8):
                        drive(1)
                        time.sleep(.06)
                    self.assertIn([-600,600,-600,600],outputs[count:])
                for _ in range(6):
                    drive(0)
                    time.sleep(.06)
                seq += 1
                request('/api/lift', {'direction':'down' if manual else 'up'} if direct or manual else
                    {'lease':lease,'sequence':seq,'server_clock':time.monotonic(),'direction':'up'})
                until(lambda:(300 if manual else 1540) in pulses)
                time.sleep(.6)
                self.assertEqual(request('/api/status')['armed'], manual)
                until(lambda:not status['armed'] and status['pwm']==[0]*4)
                if manual:
                    # Idle and stop retain the operator latch; a new gesture re-handshakes.
                    request('/api/stop',{})
                    self.assertTrue(request('/api/status')['armed'])
                    time.sleep(1.8)
                    for _ in range(8):
                        request('/api/drive',{'lateral':1})
                        time.sleep(.06)
                    self.assertIn([-600,-600,600,600],outputs)
                    request('/api/lock',{})
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
