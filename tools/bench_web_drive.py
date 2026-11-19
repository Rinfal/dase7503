#!/usr/bin/env python3
"""Raised-wheel HTTP-to-board verification, with encoder evidence and final lock."""
import argparse
import http.cookiejar
import json
import threading
import time
import urllib.request
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--wheels-raised', action='store_true', required=True)
    parser.add_argument('--unloaded', action='store_true', required=True)
    parser.add_argument('--port', type=int, default=8080)
    parser.add_argument('--report', type=Path, required=True)
    parser.add_argument('--mode', choices=('drive','lift'), default='drive')
    parser.add_argument('--up-us', type=int, default=540)
    parser.add_argument('--down-us', type=int, default=400)
    args = parser.parse_args()
    rclpy.init()
    node = Node('web_drive_observer')
    samples = []
    node.create_subscription(String, '/drive/status',
        lambda msg: samples.append(dict(json.loads(msg.data), received=time.monotonic())),
        qos_profile_sensor_data)
    running = threading.Event()
    running.set()

    def spin():
        while running.is_set():
            rclpy.spin_once(node, timeout_sec=.02)

    thread = threading.Thread(target=spin)
    thread.start()
    client = urllib.request.build_opener(urllib.request.HTTPCookieProcessor(http.cookiejar.CookieJar()))

    def api(path, body=None):
        request = urllib.request.Request('http://127.0.0.1:'+str(args.port)+path,
            data=json.dumps(body).encode() if body is not None else None,
            headers={'Content-Type': 'application/json'})
        with client.open(request, timeout=1) as response:
            return json.load(response)

    results = []
    try:
        api('/api/login', {'password': Path('~/.config/dase7503/remote-password').expanduser().read_text().strip()})
        api('/api/stop', {})
        deadline=time.monotonic()+10
        while time.monotonic()<deadline:
            feedback=api('/api/status')['feedback_age']
            if len(samples)>=10 and time.monotonic()-samples[-1]['received']<.3 and feedback is not None and feedback<.3:
                break
            time.sleep(.1)
        assert samples and time.monotonic()-samples[-1]['received'] < .3
        assert samples[-1]['pwm'] == [0]*4 and not samples[-1]['armed']
        stages = (
                ('forward', 1, 0, [-600,600,-600,600]),
                ('backward', -1, 0, [600,-600,600,-600]),
                ('ccw', 0, 1, [-600]*4), ('cw', 0, -1, [600]*4))
        for name, forward, angular, expected in stages if args.mode=='drive' else ():
            before = samples[-1]
            armed = api('/api/arm', {})
            lease = armed['lease']
            start = time.monotonic()
            sequence = 0
            while time.monotonic()-start < 2:
                sequence += 1
                api('/api/drive', {'lease': lease, 'sequence': sequence,
                    'server_clock': time.monotonic(), 'forward': forward, 'angular': angular})
                time.sleep(.06)
            last_sent = time.monotonic()
            # Deliberately drop the HTTP heartbeat. Host and MCU must lock.
            time.sleep(.8)
            after = samples[-1]
            window = [s for s in samples if start <= s['received'] <= last_sent+.8]
            delta = [b-a for a,b in zip(before['encoders'],after['encoders'])]
            zero = [s for s in window if s['received'] >= last_sent and s['pwm'] == [0]*4 and not s['armed']]
            valid = (any(s['pwm'] == expected for s in window)
                     and all(d*p > 0 for d,p in zip(delta,expected))
                     and zero and zero[0]['received']-last_sent < .6
                     and len(window) >= 20 and after['pwm'] == [0]*4 and not after['armed']
                     and not api('/api/status')['armed'])
            results.append({'direction':name,'verified':bool(valid),'expected_pwm':expected,
                'encoder_delta':delta,'client_silence_stop_seconds':zero[0]['received']-last_sent if zero else None,
                'samples':window})
            assert valid, 'HTTP遥控实车反馈未通过：'+name
            api('/api/stop', {})
            time.sleep(.3)
        if args.mode=='lift':
            assert api('/api/status')['lift_enabled'], '网页升降未开放'
            armed = api('/api/arm', {})
            lease, sequence = armed['lease'], 0

            def heartbeat(seconds):
                nonlocal sequence
                end = time.monotonic()+seconds
                while time.monotonic()<end:
                    sequence += 1
                    api('/api/drive',{'lease':lease,'sequence':sequence,
                        'server_clock':time.monotonic(),'forward':0,'angular':0})
                    time.sleep(.06)

            heartbeat(.6)
            for direction, pulse in (('up',args.up_us),('down',args.down_us)):
                before = samples[-1]
                start = time.monotonic()
                sequence += 1
                api('/api/lift',{'lease':lease,'sequence':sequence,
                    'server_clock':start,'direction':direction})
                heartbeat(5)
                window = [s for s in samples if s['received']>=start]
                after = samples[-1]
                valid = (after['servo_us']==pulse and after['servo_active']
                    and after['servo_requests']==before['servo_requests']+1
                    and abs(after['servo_high_us']-pulse)<=60
                    and all(s['pwm']==[0]*4 and s['encoders']==before['encoders'] for s in window))
                results.append({'direction':'lift_'+direction,'verified':bool(valid),
                    'physical_motion_verified':False,'expected_us':pulse,'samples':window})
                assert valid, '网页升降信号验证失败'
    finally:
        try:
            api('/api/stop', {})
            time.sleep(.3)
        finally:
            running.clear()
            thread.join(timeout=2)
            report = {'verified':len(results)==(4 if args.mode=='drive' else 2) and all(r['verified'] for r in results),
                      'results':results,'final':samples[-1] if samples else None}
            args.report.parent.mkdir(parents=True,exist_ok=True)
            args.report.write_text(json.dumps(report,indent=2)+'\n')
            print(json.dumps({'verified':report['verified'],'directions':len(results)}))
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
