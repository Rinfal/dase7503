#!/usr/bin/env python3
"""Supervised, unloaded bench test for our four-channel controller only."""
import argparse
import json
import multiprocessing
import os
import signal
import time
from pathlib import Path


def independent_stop(start, ready, finished, duration):
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Bool, Int32MultiArray

    rclpy.init()
    node = Node("wheel_bench_stop")
    wheels = node.create_publisher(Int32MultiArray, "/wheel_pwm", 1)
    arm = node.create_publisher(Bool, "/drive_arm", 1)
    ready.set()
    start.wait(timeout=30)
    deadline = time.monotonic() + duration + 0.15
    while time.monotonic() < deadline and not finished.is_set():
        rclpy.spin_once(node, timeout_sec=0.02)
    deadline = time.monotonic() + 2
    while time.monotonic() < deadline:
        wheels.publish(Int32MultiArray(data=[0]*4))
        arm.publish(Bool(data=False))
        rclpy.spin_once(node, timeout_sec=0.02)
    node.destroy_node()
    rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--wheels-raised", action="store_true")
    parser.add_argument("--unloaded", action="store_true")
    parser.add_argument("--pwm", required=True, help="M1,M2,M3,M4 signed duty, each -600..600 /1000")
    parser.add_argument("--duration", type=float, default=15)
    parser.add_argument("--countdown", type=float, default=5)
    parser.add_argument("--quiet", type=float, default=0, help="command silence after pulse, for watchdog measurement")
    parser.add_argument("--startup-pwm", type=int, default=0, help="optional 0.3s starting duty, 0..600")
    parser.add_argument("--pause-agent", action="store_true", help="pause our local agent during the silence interval")
    parser.add_argument("--report", type=Path, required=True)
    args = parser.parse_args()
    try:
        pwm = [int(value) for value in args.pwm.split(",")]
    except ValueError:
        parser.error("PWM 必须是四个整数")
    if len(pwm) != 4 or any(abs(value) > 600 for value in pwm):
        parser.error("四路 PWM 限制为 -600..600")
    if not args.wheels_raised or not args.unloaded or os.environ.get("ROS_DOMAIN_ID") != "20":
        parser.error("必须确认悬空、空载且 ROS_DOMAIN_ID=20")
    if not 0.1 <= args.duration <= 15 or not 0 <= args.countdown <= 5 or not 0 <= args.quiet <= 2:
        parser.error("测试时间0.1..15秒，倒计时0..5秒，静默0..2秒")
    if not 0 <= args.startup_pwm <= 600:
        parser.error("起转PWM限制为0..600")
    agent_pid = None
    if args.pause_agent:
        if args.quiet < 1:
            parser.error("暂停代理测试需要至少1秒静默")
        for proc in Path("/proc").iterdir():
            if not proc.name.isdigit():
                continue
            try:
                command = (proc/"cmdline").read_bytes().split(b"\0")
                if (command and Path(os.fsdecode(command[0])).name == "micro_ros_agent"
                        and b"serial" in command and proc.stat().st_uid == os.getuid()):
                    if agent_pid is not None:
                        parser.error("发现多个串口代理，拒绝暂停")
                    agent_pid = int(proc.name)
            except (OSError, ProcessLookupError):
                pass
        if agent_pid is None:
            parser.error("未找到属于当前用户的串口代理")

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, qos_profile_sensor_data
    from std_msgs.msg import Bool, Int32MultiArray, String

    ctx = multiprocessing.get_context("spawn")
    start, ready, finished = ctx.Event(), ctx.Event(), ctx.Event()
    stopper = ctx.Process(target=independent_stop, args=(start, ready, finished,
        args.duration+args.quiet+(0.8 if args.pause_agent else 0)))
    stopper.start()
    rclpy.init()
    node = Node("wheel_bench")
    wheels = node.create_publisher(Int32MultiArray, "/wheel_pwm", 1)
    arm = node.create_publisher(Bool, "/drive_arm", 1)
    samples = []
    report = {"pwm": pwm, "startup_pwm": args.startup_pwm, "duration": args.duration,
              "quiet": args.quiet, "completed": False}
    agent_paused = False

    def receive(msg):
        try:
            status = json.loads(msg.data)
            if str(status.get("firmware", "")).startswith("dase-rc-"):
                samples.append(dict(status, received=time.monotonic()))
        except (ValueError, TypeError):
            pass

    # Keep telemetry replayed in a burst after the agent resumes; depth 5 can
    # discard the samples that document what happened during the pause.
    node.create_subscription(String, "/drive/status", receive,
        QoSProfile(depth=100, reliability=qos_profile_sensor_data.reliability))

    def spin_for(seconds, command=None, armed=None):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            if command is not None:
                wheels.publish(Int32MultiArray(data=command))
            if armed is not None:
                arm.publish(Bool(data=armed))
            rclpy.spin_once(node, timeout_sec=0.02)

    try:
        if not ready.wait(timeout=5):
            raise RuntimeError("独立停车进程未就绪")
        deadline = time.monotonic()+8
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            endpoints = node.get_subscriptions_info_by_topic("/wheel_pwm")
            if samples and any(info.node_name == "dase_rc_controller" for info in endpoints):
                break
        else:
            raise RuntimeError("新固件订阅或板端状态未就绪")
        for topic in ("/wheel_pwm", "/drive_arm"):
            others = [info.node_name for info in node.get_publishers_info_by_topic(topic)
                      if info.node_name not in ("wheel_bench", "wheel_bench_stop")]
            if others:
                raise RuntimeError(f"存在其他控制发布者: {topic} {others}")
        spin_for(0.3, [0]*4, False)
        print(f"{args.countdown:.0f}秒后测试 M1..M4 PWM={pwm}，持续{args.duration:.0f}秒。", flush=True)
        spin_for(args.countdown, [0]*4)
        spin_for(0.15, [0]*4, True)
        if not samples[-1]["armed"] or time.monotonic()-samples[-1]["received"] > 0.4:
            raise RuntimeError("板端未确认解锁或反馈超时")
        report["started"] = time.monotonic()
        report["initial_encoders"] = samples[-1]["encoders"]
        start.set()
        deadline = time.monotonic()+args.duration
        while time.monotonic() < deadline:
            if time.monotonic()-samples[-1]["received"] > 0.4 or not samples[-1]["armed"]:
                raise RuntimeError("运动期间板端反馈丢失或解锁已撤销")
            if time.monotonic()-report["started"] > 1.2:
                recent = [s for s in samples if s["received"] >= time.monotonic()-0.8]
                missing = [i+1 for i, value in enumerate(pwm)
                           if value and max(abs(value), args.startup_pwm) >= 350
                           and (len(recent) < 2 or abs(recent[-1]["encoders"][i]-recent[0]["encoders"][i]) < 3)]
                if missing:
                    raise RuntimeError(f"起转后仍无编码器脉冲，停车检查端口 {missing}")
            command = pwm
            if args.startup_pwm and time.monotonic()-report["started"] < 0.3:
                command = [max(abs(value), args.startup_pwm)*(1 if value > 0 else -1) if value else 0 for value in pwm]
            wheels.publish(Int32MultiArray(data=command))
            report["last_command"] = time.monotonic()
            rclpy.spin_once(node, timeout_sec=0.02)
        if args.pause_agent:
            report["pause_board_us_estimate"] = (samples[-1]["board_us"]+
                (time.monotonic()-samples[-1]["received"])*1e6)
            os.kill(agent_pid, signal.SIGSTOP)
            agent_paused = True
        spin_for(args.quiet)
        if agent_paused:
            os.kill(agent_pid, signal.SIGCONT)
            agent_paused = False
            spin_for(0.2)
            report["resume_encoders"] = samples[-1]["encoders"]
            # Deliberately repeat motion without a new arm: it must be ignored.
            spin_for(0.2, pwm)
            report["motion_without_rearm_rejected"] = bool(
                not samples[-1]["armed"] and samples[-1]["pwm"] == [0]*4
                and samples[-1]["encoders"] == report["resume_encoders"])
        report["completed"] = True
    except BaseException as error:
        report["error"] = str(error)
        raise
    finally:
        if agent_paused:
            os.kill(agent_pid, signal.SIGCONT)
        finished.set()
        start.set()
        spin_for(1.2, [0]*4, False)
        stopper.join(timeout=4)
        spin_for(0.2, [0]*4, False)
        report["samples"] = samples
        report["final_status"] = samples[-1] if samples else None
        if samples and "initial_encoders" in report:
            report["encoder_delta"] = [last-first for first, last in
                zip(report["initial_encoders"], samples[-1]["encoders"])]
        report["stopped_feedback"] = bool(samples and samples[-1]["pwm"] == [0]*4
            and not samples[-1]["armed"] and time.monotonic()-samples[-1]["received"] < 0.5)
        if args.pause_agent and "pause_board_us_estimate" in report:
            origin = report["pause_board_us_estimate"]
            tail = [s for s in samples if args.quiet-0.35 <= (s["board_us"]-origin)/1e6 <= args.quiet-0.05]
            record = samples[-1]
            wd_command = record.get("wd_command_us", 0)
            wd_stop = record.get("wd_stop_us", 0)
            wd_settled = record.get("wd_settled_us", 0)
            delay = (wd_stop-wd_command)/1e6 if wd_command and wd_stop else None
            settled_during_pause = bool(wd_stop and wd_stop < wd_settled < origin+args.quiet*1e6
                and record.get("wd_encoders") == report.get("resume_encoders") == record["encoders"])
            motion_observed = bool(record.get("wd_encoders") and all(
                abs(record["wd_encoders"][i]-report["initial_encoders"][i]) >= 3
                for i, value in enumerate(pwm) if value))
            report["agent_interruption"] = {
                "test": "agent_process_suspended",
                "pause_seconds": args.quiet,
                "stopped_samples_from_pause": len(tail),
                "evidence_source": "independent_motor_task_timestamps_and_encoder_snapshot",
                "output_zero_delay_s": delay,
                "encoder_settled_during_agent_pause": settled_during_pause,
                "verified": bool(delay is not None and 0.25 <= delay <= 0.28
                    and origin-100000 <= wd_command < origin+10000 and settled_during_pause
                    and motion_observed and record["watchdog_timeout"]
                    and report.get("motion_without_rearm_rejected")),
                "does_not_verify": "physical_USB_disconnect_or_agent_restart",
            }
        elif args.quiet and "last_command" in report:
            last = report["last_command"]
            quiet = [sample for sample in samples if 0 <= sample["received"]-last <= args.quiet]
            stopped = [sample for sample in quiet if sample["pwm"] == [0]*4
                       and sample["watchdog_timeout"] and not sample["armed"]]
            moving = [sample for sample in samples
                      if report["started"] <= sample["received"] <= last and any(sample["pwm"])]
            tail = [sample for sample in quiet if sample["received"]-last >= args.quiet-0.25]
            encoder_motion = bool(moving and moving[0]["encoders"] != moving[-1]["encoders"])
            settled = len(tail) >= 3 and all(sample["encoders"] == tail[-1]["encoders"] for sample in tail)
            delay = stopped[0]["received"]-last if stopped else None
            report["watchdog"] = {
                "test": "command_silence_with_agent_running",
                "output_zero_observed_delay_s": delay,
                "encoder_motion_observed": encoder_motion,
                "encoder_settled_during_silence": settled,
                "verified": bool(encoder_motion and settled and delay is not None and delay <= 0.45),
                "does_not_verify": "USB_disconnection_or_agent_crash",
            }
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(report, indent=2)+"\n")
        print(json.dumps({key: value for key, value in report.items() if key != "samples"}), flush=True)
        node.destroy_node()
        rclpy.shutdown()
        if stopper.is_alive() or not report["stopped_feedback"]:
            raise RuntimeError("未确认板端已停车，请观察并按需切断电机电源")


if __name__ == "__main__":
    main()
