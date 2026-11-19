#!/usr/bin/env python3
"""One unloaded S2 signal probe; record acknowledgement and pin shutoff."""
import argparse
import json
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Int32, String


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pulse-us', type=int, required=True)
    parser.add_argument('--wheels-raised', action='store_true', required=True)
    parser.add_argument('--unloaded', action='store_true', required=True)
    parser.add_argument('--report', type=Path, required=True)
    parser.add_argument('--resume-from', type=Path, help='after reflash, restore the last observed pulse in this evidence')
    parser.add_argument('--direct-step', action='store_true', help='explicit observed calibration: step down at most100us after reflash')
    args = parser.parse_args()
    if not 150 <= args.pulse_us <= 2100:
        parser.error('探测脉宽限于150..2100us，禁止全行程扫动')
    rclpy.init()
    node = Node('s2_single_probe')
    samples = []
    node.create_subscription(String, '/drive/status',
        lambda msg: samples.append(dict(json.loads(msg.data), received=time.monotonic())),
        qos_profile_sensor_data)
    pub = node.create_publisher(Int32, '/servo_s2_us', 1)

    def wait(seconds):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.02)

    try:
        deadline = time.monotonic()+10
        while time.monotonic()<deadline:
            wait(.1)
            if len(samples)>=30 and time.monotonic()-samples[-1]['received']<.3:
                break
        assert len(samples) >= 10 and time.monotonic()-samples[-1]['received'] < .3, '反馈不新鲜'
        before = samples[-1]
        assert before['firmware'] in ('dase-rc-0.5','dase-rc-0.6','dase-rc-0.7','dase-rc-0.8','dase-rc-0.9'), '固件不支持信号测量'
        assert args.pulse_us >= {'dase-rc-0.5':700,'dase-rc-0.6':500,'dase-rc-0.7':400,'dase-rc-0.8':250,'dase-rc-0.9':150}[before['firmware']]
        assert all(s['pwm'] == [0]*4 and not s['armed'] for s in samples), '底盘未锁定'
        assert all(s['encoders'] == before['encoders'] for s in samples), '车轮仍在动'
        assert not before['servo_active'], '上次信号尚未撤去'
        previous = before['servo_us']
        if previous:
            assert abs(args.pulse_us-previous)<=40, '后续步长最多40us'
        elif args.resume_from:
            saved = json.loads(args.resume_from.read_text())
            assert saved.get('signal_sequence_verified') is True or saved.get('signal_off_verified') is True
            saved_pulse=saved['samples'][-1]['servo_us']
            if args.direct_step:
                assert 0<=saved_pulse-args.pulse_us<=100, '显式低位探测最多减小100us'
            else:
                assert saved_pulse==args.pulse_us, '只恢复刷写前已观察过的目标'
        else:
            assert args.pulse_us==1500, '首次仅1500，或提供刷写前观察记录'
        assert pub.get_subscription_count() == 1, '舵机订阅数量异常'
        assert node.count_publishers('/servo_s2_us') == 1, '存在其他舵机控制进程'
        sent = time.monotonic()
        pub.publish(Int32(data=-args.pulse_us))
        wait(2)
        after = [s for s in samples if s['received'] > sent]
        acknowledged = [s for s in after if s['servo_us'] == args.pulse_us
                        and s['servo_requests'] == before['servo_requests']+1]
        verified = bool(acknowledged and not acknowledged[-1]['servo_active']
            and acknowledged[-1]['servo_edges']-before['servo_edges'] >= (1 if args.pulse_us<400 else 4)
            and abs(acknowledged[-1]['servo_high_us']-args.pulse_us) <= 60
            and time.monotonic()-after[-1]['received'] < .3
            and all(s['pwm'] == [0]*4 and not s['armed'] for s in after)
            and all(s['encoders'] == before['encoders'] for s in after))
        report = {'pulse_us': args.pulse_us, 'signal_limit_ms': 40 if args.pulse_us<400 else 120,
                  'physical_motion_verified': False, 'signal_off_verified': verified,
                  'before': before, 'samples': after}
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(report, indent=2)+'\n')
        print(json.dumps({k:v for k,v in report.items() if k not in ('samples','before')}))
        assert verified, '未确认板端撤去信号'
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
