#!/usr/bin/env python3
import os
import selectors
import signal
import subprocess
import time
from pathlib import Path


def main():
    root = Path(__file__).resolve().parents[1]
    environment = dict(os.environ, ROS_DOMAIN_ID="231", OPENBLAS_NUM_THREADS="1",
                       ROS_AUTOMATIC_DISCOVERY_RANGE="LOCALHOST")
    process = subprocess.Popen(
        ["ros2", "launch", "logistics_robot", "observe.launch.py"],
        cwd=root, env=environment, stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT, start_new_session=True, bufsize=0,
    )
    selector = selectors.DefaultSelector()
    selector.register(process.stdout, selectors.EVENT_READ)
    output = bytearray()
    markers = (b"emergency_stop", b"QR scanner ready", b"Odometry bridge ready")
    ready = False
    deadline = time.monotonic() + 30
    try:
        while time.monotonic() < deadline and process.poll() is None:
            if selector.select(timeout=0.5):
                output.extend(os.read(process.stdout.fileno(), 65536))
                if all(marker in output for marker in markers):
                    ready = True
                    break
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
        try:
            remaining, _ = process.communicate(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            remaining, _ = process.communicate(timeout=5)
        output.extend(remaining)
        transcript = output.decode("utf-8", errors="replace")
        (root / "docs/evidence/observe_smoke.log").write_text(transcript, encoding="utf-8")
        success = (ready and process.returncode == 0
                   and "Traceback" not in transcript and "[ERROR]" not in transcript)
        print({"all_nodes_ready": ready, "returncode": process.returncode, "success": success})
        return 0 if success else 1
    finally:
        selector.close()
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=5)


if __name__ == "__main__":
    raise SystemExit(main())
