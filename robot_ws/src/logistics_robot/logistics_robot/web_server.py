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
from .durability import atomic_write


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
    elif drive and pwm_mode and servo.get('manual_lift_authorized') is True:
        # Explicit operator authorization of the previously exercised small
        # working band is distinct from verified mechanical travel.
        candidate = servo.get('manual_pulses_us', {})
        if (set(candidate) == {'down', 'up'} and type(candidate['down']) is int
                and 300 <= candidate['down'] <= 400
                and candidate['up'] in (540, 750) and all(type(v) is int for v in candidate.values())):
            lift = candidate
    lateral = controller.get("lateral_response_verified") is True or (
        pwm_mode and controller.get('wheel_type') == 'mecanum'
        and controller.get('manual_lateral_authorized') is True)
    return drive, drive and lateral, lift


class WebState:
    def __init__(self, password, profile, evidence_dir):
        self.lock = threading.RLock()
        self.frame_ready = threading.Condition(self.lock)
        drive, lateral, lift = verified_features(profile)
        controller = profile.get('controller', {})
        self.bench_only = (str(controller.get('firmware', '')).startswith('dase-rc-')
                           and controller.get('ground_drive_verified') is not True)
        self.control = RemoteControl(drive_enabled=drive, lateral_enabled=lateral, lift_angles=lift,
                                     direct_mode=controller.get('direct_control') is True,
                                     manual_lock_mode=controller.get('manual_lock_control') is True)
        self.lift_range_verified = profile.get('servo', {}).get('safe_range_verified') is True
        self.password = password
        self.sessions = {}
        self.control_versions = {}
        self.login_attempts = {}
        self.owner = None
        self.jpeg = None
        self.frame_time = None
        self.frame_sequence = 0
        self.frame_times = deque(maxlen=30)
        self.battery = None
        self.board_status = None
        self.evidence_dir = Path(evidence_dir).expanduser().resolve()
        self.evidence_dir.mkdir(parents=True, exist_ok=True)
        self.qr_events = deque(maxlen=30)
        self.last_snapshot = -float("inf")
        self.tick_time = None

    def set_frame(self, jpeg, stamp=None):
        with self.frame_ready:
            self.jpeg = bytes(jpeg)
            self.frame_time = time.monotonic() if stamp is None else stamp
            self.frame_sequence += 1
            if self.frame_times and self.frame_time-self.frame_times[-1] > 1:
                self.frame_times.clear()
            self.frame_times.append(self.frame_time)
            self.frame_ready.notify_all()

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
            self.control_versions = {s:v for s,v in self.control_versions.items() if s in self.sessions}
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
                          video_fps=((len(self.frame_times)-1)/(self.frame_times[-1]-self.frame_times[0])
                              if len(self.frame_times) > 1 and self.frame_times[-1] > self.frame_times[0] else 0),
                          bench_only=self.bench_only,
                          lift_range_verified=self.lift_range_verified,
                          lift_pulses_us=self.control.lift_angles,
                          battery_raw=self.battery, battery_unit_verified=False,
                          qr=list(self.qr_events))
            if self.board_status:
                status.update(board_armed=self.board_status['armed'],
                              wheel_pwm=self.board_status['pwm'],
                              lift_command_us=self.board_status['servo_us'],
                              lift_holding=self.board_status['servo_active'])
            if not status["armed"]:
                self.owner = None
            return status

    def action(self, route, body, session):
        with self.lock:
            now = time.monotonic()
            if self.control.simple_control and route in ('/api/drive', '/api/lift', '/api/stop', '/api/logout', '/api/arm', '/api/lock'):
                version = body.get('control_version')
                if type(version) is int and version >= 0:
                    if version < self.control_versions.get(session, -1):
                        return self.status()  # An old gesture must not undo release/stop.
                    self.control_versions[session] = version
                elif self.control.manual_lock_mode:
                    raise ControlError("缺少有效操作编号，请刷新页面")
            if route == "/api/stop":
                self.control.stop()
                if not self.control.manual_lock_mode:
                    self.owner = None
            elif route == "/api/lock":
                self.control.lock()
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
                if self.control.simple_control:
                    self.owner = session
                elif self.owner != session:
                    raise ControlError("当前设备没有控制权")
                (self.control.drive if route == "/api/drive" else self.control.lift)(body, now)
            elif route == "/api/snapshot":
                if self.jpeg is None or self.frame_time is None or now - self.frame_time > 1:
                    raise ControlError("相机画面未就绪或已经过期")
                if now - self.last_snapshot < 1:
                    raise ControlError("请稍候再截图")
                self.last_snapshot = now
                filename = "manual_" + str(time.time_ns()) + ".jpg"
                atomic_write(self.evidence_dir / filename, self.jpeg)
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
        self.stream_slots = threading.BoundedSemaphore(3)
        self.stream_stop = threading.Event()
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

    def server_close(self):
        self.stream_stop.set()
        with self.state.frame_ready:
            self.state.frame_ready.notify_all()
        super().server_close()

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
        self.send_header("Content-Security-Policy", "default-src 'self'; img-src 'self' blob:; frame-ancestors 'none'; object-src 'none'")
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
                  "/video.js": ("video.js", "text/javascript; charset=utf-8"),
                  "/style.css": ("style.css", "text/css; charset=utf-8")}
        if path in static:
            name, kind = static[path]
            self.reply(200, (Path(__file__).parent / "web" / name).read_bytes(), kind)
            return
        if path == '/healthz':
            with self.server.state.lock:
                stamp = self.server.state.tick_time
            self.reply(200, {'tick_age': None if stamp is None else time.monotonic()-stamp})
            return
        if not self.server.state.authenticated(self.headers.get("Cookie")):
            self.reply(401, {"error": "请先登录"})
            return
        state = self.server.state
        if path == "/api/status":
            self.reply(200, state.status())
        elif path == "/api/evidence":
            self.reply(200, state.evidence())
        elif path in ("/api/stream", "/api/video"):
            self.stream(binary=path == "/api/video")
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

    def stream(self, binary=False):
        state = self.server.state
        if not self.server.stream_slots.acquire(blocking=False):
            self.reply(503, {"error": "视频连接过多，请关闭其他视频页面"})
            return
        try:
            with state.lock:
                if state.jpeg is None or state.frame_time is None or time.monotonic()-state.frame_time > 1:
                    self.reply(503, {"error": "等待实时相机画面"})
                    return
            self.send_response(200)
            self.send_header('Content-Type', 'application/octet-stream' if binary else
                             'multipart/x-mixed-replace; boundary=frame')
            self.send_header('X-Accel-Buffering', 'no')
            self.send_header('Cache-Control', 'no-store, no-cache, must-revalidate')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.send_header('Connection', 'close')
            self.end_headers()
            self.close_connection = True
            last_sequence = -1
            while not self.server.stream_stop.is_set():
                if not state.authenticated(self.headers.get('Cookie')):
                    break
                with state.frame_ready:
                    state.frame_ready.wait_for(lambda: state.frame_sequence != last_sequence
                        or self.server.stream_stop.is_set(), timeout=.5)
                    if self.server.stream_stop.is_set():
                        break
                    if state.frame_time is None or time.monotonic()-state.frame_time > 1:
                        break
                    if state.frame_sequence == last_sequence:
                        continue
                    jpeg, last_sequence = state.jpeg, state.frame_sequence
                if not state.authenticated(self.headers.get('Cookie')):
                    break
                # Never hold the control lock during a slow network write.
                header = (f'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: {len(jpeg)}\r\n'
                          f'X-Frame-Sequence: {last_sequence}\r\n\r\n').encode()
                self.wfile.write(header + jpeg + b'\r\n')
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, TimeoutError, OSError):
            pass
        finally:
            self.server.stream_slots.release()

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
