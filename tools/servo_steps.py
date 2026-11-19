#!/usr/bin/env python3
"""Observed empty lift calibration in steps of at most 20us, never a full sweep."""
import argparse
import json
import os
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Int32, String


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--target-us', type=int, required=True)
    parser.add_argument('--wheels-raised', action='store_true', required=True)
    parser.add_argument('--unloaded', action='store_true', required=True)
    parser.add_argument('--report', type=Path, required=True)
    args = parser.parse_args()
    if not 150 <= args.target_us <= 1500:
        parser.error('当前观察批次只允许150..1500us')
    rclpy.init()
    node = Node('s2_observed_steps')
    samples, commands = [], []
    pub = node.create_publisher(Int32, '/servo_s2_us', 1)
    node.create_subscription(String, '/drive/status',
        lambda msg:samples.append(dict(json.loads(msg.data), received=time.monotonic())),qos_profile_sensor_data)
    pidfile = Path('runtime/lift_steps.pid')
    pidfile.write_text(str(os.getpid()))
    verified = False

    def wait(seconds, check=True):
        deadline = time.monotonic()+seconds
        while time.monotonic()<deadline:
            rclpy.spin_once(node, timeout_sec=.02)
            if check:
                assert samples and time.monotonic()-samples[-1]['received'] < .3, '反馈中断'
                assert samples[-1]['pwm'] == [0]*4 and not samples[-1]['armed'], '底盘未锁定'

    try:
        deadline = time.monotonic()+10
        while time.monotonic()<deadline:
            wait(.1,False)
            if len(samples)>=30 and time.monotonic()-samples[-1]['received']<.3:
                break
        assert len(samples)>=20 and time.monotonic()-samples[-1]['received']<.3
        before = samples[-1]
        assert before['firmware'] in ('dase-rc-0.5','dase-rc-0.6','dase-rc-0.7','dase-rc-0.8','dase-rc-0.9') and not before['servo_active']
        assert args.target_us >= {'dase-rc-0.5':700,'dase-rc-0.6':500,'dase-rc-0.7':400,'dase-rc-0.8':250,'dase-rc-0.9':150}[before['firmware']]
        assert before['servo_us'] and abs(args.target_us-before['servo_us'])<=200, '每批最多200us'
        assert all(s['encoders']==before['encoders'] and s['pwm']==[0]*4 and not s['armed'] for s in samples)
        assert node.count_publishers('/servo_s2_us')==1 and pub.get_subscription_count()==1
        pulse = before['servo_us']
        while pulse != args.target_us:
            previous = samples[-1]
            pulse += max(-20,min(20,args.target_us-pulse))
            commands.append({'pulse_us':pulse,'sent':time.monotonic()})
            pub.publish(Int32(data=-pulse))
            wait(2)
            latest = samples[-1]
            assert latest['servo_requests']==previous['servo_requests']+1
            assert latest['servo_edges']-previous['servo_edges']>=(1 if pulse<400 else 4)
            assert abs(latest['servo_high_us']-pulse)<=60 and not latest['servo_active']
            assert latest['encoders']==before['encoders']
        verified = True
    finally:
        pidfile.unlink(missing_ok=True)
        args.report.parent.mkdir(parents=True,exist_ok=True)
        args.report.write_text(json.dumps({'signal_sequence_verified':verified,
            'physical_position_verified':False,'commands':commands,'samples':samples},indent=2)+'\n')
        print(json.dumps({'signal_sequence_verified':verified,'commands':len(commands),
                          'last_pulse_us':samples[-1]['servo_us'] if samples else None}))
        node.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':
    main()
