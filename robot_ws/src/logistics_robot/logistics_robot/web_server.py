"""LAN HTTP interface. All control and sensor access shares one lock with ROS."""
import json
import socket
import secrets
import threading
import time
from collections import deque
from http.cookies import SimpleCookie
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlsplit

from .remote_control import ControlError, RemoteControl


def verified_features(profile):
    controller = profile.get("controller", {})
    required = ("wheel_directions_verified", "forward_direction_verified",
                "backward_direction_verified", "positive_yaw_direction_verified",
                "negative_yaw_direction_verified", "command_watchdog_verified")
    drive = all(controller.get(name) is True for name in required)
    # The new PWM firmware has a different interface from the factory Twist
    # bridge. Hardware bench results alone must not enable that old bridge.
    if str(controller.get("firmware", "")).startswith("dase-rc-"):
        drive = drive and controller.get("web_drive_validated") is True
    # Servo angles are intentionally supplied only after an explicit calibration.
    servo = profile.get("servo", {})
    pwm_mode = str(controller.get('firmware', '')).startswith('dase-rc-')
    angles = servo.get('calibrated_pulses_us' if pwm_mode else 'calibrated_angles', {})
    lift = {}
    if drive and servo.get("safe_range_verified") is True:
        low, high = (400, 2100) if pwm_mode else (-90, 90)
        if (set(angles) == {"up", "down"} and angles['up'] != angles['down'] and
                all(type(a) is int and low <= a <= high for a in angles.values())):
            lift = angles
    return drive, drive and controller.get("lateral_response_verified") is True, lift


class WebState:
    def __init__(self, password, profile, evidence_dir):
        self.lock = threading.RLock()
        drive, lateral, lift = verified_features(profile)
        controller = profile.get('controller', {})
        self.bench_only = (str(controller.get('firmware', '')).startswith('dase-rc-')
                           and controller.get('ground_drive_verified') is not True)
        self.control = RemoteControl(drive_enabled=drive, lateral_enabled=lateral, lift_angles=lift)
        self.password = password
        self.sessions = {}
        self.login_attempts = {}
        self.owner = None
        self.jpeg = None
        self.frame_time = None
        self.battery = None
        self.board_status = None
        self.evidence_dir = Path(evidence_dir).expanduser().resolve()
        self.evidence_dir.mkdir(parents=True, exist_ok=True)
        self.qr_events = deque(maxlen=30)
        self.last_snapshot = -float("inf")

    def authenticated(self, cookie):
        try:
            jar = SimpleCookie(cookie or "")
            token = jar["robot_session"].value if "robot_session" in jar else ""
        except Exception:
            return None
        with self.lock:
            expires = self.sessions.get(token, 0)
            if time.monotonic() >= expires:
                self.sessions.pop(token, None)
                if self.owner == token:
                    self.control.stop()
                    self.owner = None
                return None
        return token

    def login(self, address, password):
        now = time.monotonic()
        with self.lock:
            self.login_attempts = {ip: (t, n) for ip, (t, n) in self.login_attempts.items()
                                   if now - t < 60}
            first, count = self.login_attempts.get(address, (now, 0))
            if count >= 5:
                raise ControlError("尝试过多，请一分钟后重试")
            if not secrets.compare_digest(str(password).encode(), self.password.encode()):
                if len(self.login_attempts) > 1024:
                    self.login_attempts.pop(next(iter(self.login_attempts)))
                self.login_attempts[address] = (first, count + 1)
                raise ControlError("访问口令错误")
            self.sessions = {s: expiry for s, expiry in self.sessions.items() if expiry > now}
            if len(self.sessions) >= 32:
                raise ControlError("查看会话过多，请退出其他设备")
            token = secrets.token_urlsafe(32)
            self.sessions[token] = now + 8 * 3600
            self.login_attempts.pop(address, None)
            return token

    def status(self):
        with self.lock:
            now = time.monotonic()
            status = self.control.status(now)
            status.update(camera_age=None if self.frame_time is None else now - self.frame_time,
                          bench_only=self.bench_only,
                          battery_raw=self.battery, battery_unit_verified=False,
                          qr=list(self.qr_events))
            if self.board_status:
                status.update(board_armed=self.board_status['armed'],
                              wheel_pwm=self.board_status['pwm'],
                              lift_command_us=self.board_status['servo_us'])
            if not status["armed"]:
                self.owner = None
            return status

    def action(self, route, body, session):
        with self.lock:
            now = time.monotonic()
            if route == "/api/stop":
                self.control.stop()
                self.owner = None
            elif route == "/api/logout":
                self.control.stop()
                self.owner = None
                self.sessions.pop(session, None)
            elif route == "/api/arm":
                lease = self.control.arm(now)
                self.owner = session
                return {"lease": lease, "server_clock": now}
            elif route in ("/api/drive", "/api/lift"):
                if self.owner != session:
                    raise ControlError("当前设备没有控制权")
                (self.control.drive if route == "/api/drive" else self.control.lift)(body, now)
            elif route == "/api/snapshot":
                if self.jpeg is None or self.frame_time is None or now - self.frame_time > 1:
                    raise ControlError("相机画面未就绪或已经过期")
                if now - self.last_snapshot < 1:
                    raise ControlError("请稍候再截图")
                self.last_snapshot = now
                filename = "manual_" + str(time.time_ns()) + ".jpg"
                (self.evidence_dir / filename).write_bytes(self.jpeg)
                return {"image": "/evidence/" + filename}
            else:
                raise ControlError("未知操作")
            return self.status()

    def evidence(self):
        with self.lock:
            records = []
            path = self.evidence_dir / "qr_detections.jsonl"
            if path.is_file():
                with path.open(encoding="utf-8") as stream:
                    for line in deque(stream, maxlen=100):
                        try:
                            record = json.loads(line)
                            name = record.get("image", "")
                            if Path(name).name == name and name.endswith((".png", ".jpg")):
                                record["url"] = "/evidence/" + name
                            records.append(record)
                        except (ValueError, AttributeError, TypeError):
                            continue
            return records


class RobotHTTPServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, address, state):
        self.state = state
        self.slots = threading.BoundedSemaphore(16)
        super().__init__(address, RobotHandler)
        port = self.server_address[1]
        addresses = {"localhost", "127.0.0.1", address[0]}
        # Linux ioctl reads interface addresses without contacting a remote host.
        import fcntl
        import struct
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            for _, name in socket.if_nameindex():
                try:
                    result = fcntl.ioctl(sock, 0x8915, struct.pack('256s', name.encode()[:15]))
                    addresses.add(socket.inet_ntoa(result[20:24]))
                except OSError:
                    continue
        self.allowed_hosts = {f"{ip}:{port}" for ip in addresses if ip != "0.0.0.0"}
        if port == 80:
            self.allowed_hosts.update(addresses - {"0.0.0.0"})

    def process_request(self, request, client_address):
        if not self.slots.acquire(blocking=False):
            request.close()
            return
        try:
            super().process_request(request, client_address)
        except Exception:
            self.slots.release()
            raise

    def process_request_thread(self, request, client_address):
        try:
            request.settimeout(3)
            super().process_request_thread(request, client_address)
        finally:
            self.slots.release()


class RobotHandler(BaseHTTPRequestHandler):
    def log_message(self, *_):
        pass  # Do not log cookies, passwords or control bodies.

    def reply(self, status, data, content_type="application/json", cookie=None):
        if content_type == "application/json":
            data = json.dumps(data, ensure_ascii=False, allow_nan=False).encode()
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("Content-Security-Policy", "default-src 'self'; frame-ancestors 'none'; object-src 'none'")
        self.send_header("Referrer-Policy", "no-referrer")
        if cookie:
            self.send_header("Set-Cookie", cookie)
        self.end_headers()
        try:
            self.wfile.write(data)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def same_origin(self):
        if self.headers.get("Sec-Fetch-Site") == "cross-site":
            return False
        origin = self.headers.get("Origin")
        host = self.headers.get("Host", "")
        # Reject DNS rebinding: permit only loopback or a local interface address.
        if host not in self.server.allowed_hosts:
            return False
        return not origin or origin in ("http://" + host, "https://" + host)

    def do_GET(self):
        if not self.same_origin():
            self.reply(403, {"error": "请使用树莓派本机地址访问"})
            return
        path = urlsplit(self.path).path
        static = {"/": ("index.html", "text/html; charset=utf-8"),
                  "/app.js": ("app.js", "text/javascript; charset=utf-8"),
                  "/style.css": ("style.css", "text/css; charset=utf-8")}
        if path in static:
            name, kind = static[path]
            self.reply(200, (Path(__file__).parent / "web" / name).read_bytes(), kind)
            return
        if not self.server.state.authenticated(self.headers.get("Cookie")):
            self.reply(401, {"error": "请先登录"})
            return
        state = self.server.state
        if path == "/api/status":
            self.reply(200, state.status())
        elif path == "/api/evidence":
            self.reply(200, state.evidence())
        elif path == "/api/frame":
            with state.lock:
                frame, stamp = state.jpeg, state.frame_time
            if frame is None or stamp is None or time.monotonic() - stamp > 1:
                self.reply(503, {"error": "等待实时相机画面"})
            else:
                self.reply(200, frame, "image/jpeg")
        elif path.startswith("/evidence/"):
            name = path.removeprefix("/evidence/")
            target = state.evidence_dir / name
            if (name != Path(name).name or target.resolve().parent != state.evidence_dir
                    or target.suffix not in (".png", ".jpg") or not target.is_file()):
                self.reply(404, {"error": "截图不存在"})
            else:
                self.reply(200, target.read_bytes(), "image/png" if target.suffix == ".png" else "image/jpeg")
        else:
            self.reply(404, {"error": "不存在"})

    def do_POST(self):
        if not self.same_origin() or self.headers.get_content_type() != "application/json":
            self.reply(403, {"error": "请求来源或格式不符"})
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
            if not 0 < length <= 4096 or self.headers.get("Transfer-Encoding"):
                raise ValueError("请求大小不符")
            body = json.loads(self.rfile.read(length))
            if not isinstance(body, dict):
                raise ValueError("需要 JSON 对象")
        except (ValueError, UnicodeError):
            self.reply(400, {"error": "无效请求"})
            return
        state = self.server.state
        path = urlsplit(self.path).path
        if path == "/api/login":
            try:
                token = state.login(self.client_address[0], body.get("password", ""))
                self.reply(200, {"ok": True}, cookie="robot_session=" + token + "; HttpOnly; SameSite=Strict; Path=/; Max-Age=28800")
            except ControlError as error:
                self.reply(403, {"error": str(error)})
            return
        session = state.authenticated(self.headers.get("Cookie"))
        if not session:
            self.reply(401, {"error": "请先登录"})
            return
        try:
            self.reply(200, state.action(path, body, session))
        except ControlError as error:
            self.reply(409, {"error": str(error)})
        except OSError:
            self.reply(500, {"error": "截图保存失败，请检查磁盘"})
