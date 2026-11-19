#!/usr/bin/env python3
import argparse
import glob
import importlib.util
import json
import os
import platform
import shutil
import subprocess
from datetime import datetime, timezone
from pathlib import Path


def command_output(arguments):
    try:
        result = subprocess.run(arguments, capture_output=True, text=True, timeout=10, check=False)
        return {"returncode": result.returncode, "output": (result.stdout + result.stderr).strip()}
    except (OSError, subprocess.TimeoutExpired) as error:
        return {"error": str(error)}


def inspect():
    cameras = {}
    for filename in glob.glob("/sys/class/video4linux/video*/name"):
        cameras[Path(filename).parent.name] = Path(filename).read_text().strip()
    sensors = {}
    for filename in glob.glob("/sys/bus/i2c/devices/*/name"):
        name = Path(filename).read_text().strip()
        if any(sensor in name.lower() for sensor in ("imx", "ov", "arducam")):
            sensors[Path(filename).parent.name] = name
    report = {
        "timestamp_utc": datetime.now(timezone.utc).isoformat(),
        "architecture": platform.machine(), "kernel": platform.release(),
        "ros_setup_exists": Path("/opt/ros/jazzy/setup.bash").exists(),
        "ros_distro": os.environ.get("ROS_DISTRO"),
        "groups": command_output(["id", "-nG"]),
        "usb": command_output(["lsusb"]),
        "serial_devices": sorted(glob.glob("/dev/serial/by-id/*")),
        "serial_fallback": sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")),
        "camera_sensors": sensors, "video_devices": cameras,
        "python_modules": {name: importlib.util.find_spec(name) is not None
                           for name in ("yaml", "cv2", "numpy", "serial", "rclpy", "pyzbar")},
        "programs": {name: shutil.which(name)
                     for name in ("colcon", "ros2", "cam", "rpicam-hello", "git", "cmake", "meson")},
    }
    if shutil.which("cam"):
        report["libcamera_list"] = command_output(["cam", "-l"])
    return report


def main():
    parser = argparse.ArgumentParser(description="只读检查；不打开串口、不发送电机或舵机命令。")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    report = json.dumps(inspect(), ensure_ascii=False, indent=2)
    print(report)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(report + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
