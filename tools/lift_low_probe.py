#!/usr/bin/env python3
"""One supported, empty lift cycle: known 750us up, then bounded 150us probes.

This is a repeated short-probe experiment, not continuous 150us holding.
Every negative request expires in the MCU even if this process disappears.
It records signal evidence only; an operator must report physical movement.
"""
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
    parser.add_argument('--unloaded', action='store_true', required=True)
    parser.add_argument('--platform-supported', action='store_true', required=True)
    parser.add_argument('--report', type=Path, required=True)
    args = parser.parse_args()
    rclpy.init()
    node = Node('supported_lift_low_probe')
    samples, commands = [], []
    node.create_subscription(String, '/drive/status',
        lambda msg: samples.append(dict(json.loads(msg.data), received=time.monotonic())),
        qos_profile_sensor_data)
    pub = node.create_publisher(Int32, '/servo_s2_us', 1)
    before = None
    verified = False
    error = None

    def check():
        assert samples and time.monotonic()-samples[-1]['received'] < .3, '控制板反馈中断'
        assert all(s['pwm'] == [0]*4 and not s['armed'] for s in samples), '底盘未保持锁定'
        assert all(s['encoders'] == samples[0]['encoders'] for s in samples), '车轮发生运动'
        assert samples[-1]['board_us'] >= samples[0]['board_us'], '控制板发生复位'

    def wait(seconds, checking=True):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.01)
            if checking:
                check()

    def send(pulse):
        check()
        assert node.count_publishers('/servo_s2_us') == 1, '存在其他升降控制进程'
        commands.append({'data': pulse, 'sent': time.monotonic()})
        pub.publish(Int32(data=pulse))

    try:
        deadline = time.monotonic()+10
        while time.monotonic() < deadline:
            wait(.1, False)
            if len(samples) >= 30 and time.monotonic()-samples[-1]['received'] < .3:
                break
        check()
        before = dict(samples[-1])
        assert before['firmware'] == 'dase-rc-0.9', '需要已验证的0.9限时探测协议'
        assert not before['servo_active'], '开始前必须已经关闭保持信号'
        assert pub.get_subscription_count() == 1, '舵机订阅数量异常'
        assert node.count_publishers('/servo_s2_us') == 1, '存在其他升降控制进程'
        # First exercise the known upper target, without enabling the wheels.
        print('5秒后升起750us，随后150us重复短探测；请扶稳且不要下压。', flush=True)
        wait(5)
        send(750)
        wait(2.5)
        assert samples[-1]['servo_us'] == 750 and samples[-1]['servo_active'], '未确认750目标'
        assert samples[-1]['servo_requests'] == before['servo_requests']+1, '升起请求未唯一确认'
        print('开始150us重复短探测，共1秒，每次由板端40ms自动截止。', flush=True)
        started = time.monotonic()
        while time.monotonic()-started < 1:
            send(-150)
            wait(.06)
        wait(.5)
        last = samples[-1]
        off = [s for s in samples if s['received'] > commands[-1]['sent']+.1]
        lower = [s for s in samples if s['servo_us'] == 150]
        verified = bool(lower and lower[-1]['servo_requests'] > before['servo_requests']+5
            and lower[-1]['servo_edges']-lower[0]['servo_edges'] >= 10
            and abs(last['servo_high_us']-150) <= 10
            and len(off) >= 3 and all(not s['servo_active'] for s in off)
            and off[-1]['servo_edges'] == off[0]['servo_edges'])
        assert verified, '未确认150信号及板端自动截止'
    except BaseException as exc:
        error = str(exc)
        raise
    finally:
        report = {'operation': '750us hold 2.5s then repeated -150 probes for 1s',
            'per_probe_mcu_limit_ms': 40, 'continuous_150us_test': False,
            'signal_verified': verified, 'physical_motion_verified': False,
            'physical_result': 'awaiting_operator', 'error': error,
            'before': before, 'commands': commands, 'samples': samples}
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(report, indent=2)+'\n')
        print(json.dumps({k:v for k,v in report.items() if k not in ('before','commands','samples')}), flush=True)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
