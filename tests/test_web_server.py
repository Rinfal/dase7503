import http.client
import json
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "robot_ws/src/logistics_robot"))
from logistics_robot.web_server import RobotHTTPServer, WebState, verified_features


class WebTests(unittest.TestCase):
    def test_manual_lock_stop_timeout_and_old_gestures(self):
        control = self.state.control
        control.manual_lock_mode = True
        control.drive_enabled = True
        control.gate.update_feedback(True, time.monotonic())
        self.login()
        self.assertEqual(self.request('POST','/api/drive',{'forward':1,'control_version':1})[0],409)
        self.assertEqual(self.request('POST','/api/arm',{'control_version':2})[0],200)
        self.request('POST','/api/drive',{'forward':1,'control_version':3})
        self.request('POST','/api/stop',{'control_version':4})
        self.assertTrue(control.status(time.monotonic())['armed'])
        self.assertFalse(control.gate.armed)
        self.request('POST','/api/drive',{'forward':1,'control_version':3})
        self.assertFalse(control.gate.armed)
        self.request('POST','/api/lock',{'control_version':5})
        self.request('POST','/api/arm',{'control_version':2})
        self.assertFalse(control.status(time.monotonic())['armed'])
        self.assertEqual(self.request('POST','/api/drive',{'forward':1})[0],409)

    def test_direct_stop_and_release_cannot_be_undone_by_delayed_gesture(self):
        self.state.control.direct_mode = True
        self.state.control.drive_enabled = True
        self.state.control.gate.update_feedback(True, time.monotonic())
        self.login()
        self.assertEqual(self.request('POST', '/api/drive', {'forward':1,'control_version':1})[0], 200)
        self.assertEqual(self.request('POST', '/api/stop', {'control_version':2})[0], 200)
        self.assertEqual(self.request('POST', '/api/drive', {'forward':1,'control_version':1})[0], 200)
        self.assertFalse(self.state.control.gate.armed)
        self.assertEqual(self.request('POST', '/api/drive', {'forward':1,'control_version':3})[0], 200)
        self.assertTrue(self.state.control.gate.armed)
        self.request('POST', '/api/drive', {'forward':0,'control_version':4})
        self.request('POST', '/api/drive', {'forward':1,'control_version':3})
        self.assertEqual(self.state.control.gate.command.forward, 0)

    def test_explicit_manual_lift_band_does_not_claim_verified_travel(self):
        flags = {name: True for name in ('wheel_directions_verified',
            'forward_direction_verified', 'backward_direction_verified',
            'positive_yaw_direction_verified', 'negative_yaw_direction_verified',
            'command_watchdog_verified', 'web_drive_validated')}
        flags['firmware'] = 'dase-rc-0.9'
        servo = {'safe_range_verified': False, 'manual_lift_authorized': True,
                 'manual_pulses_us': {'down': 400, 'up': 540}}
        profile = {'controller': flags, 'servo': servo}
        self.assertEqual(verified_features(profile)[2], servo['manual_pulses_us'])
        with tempfile.TemporaryDirectory() as root:
            status = WebState('test-password', profile, root).status()
        self.assertFalse(status['lift_range_verified'])
        self.assertTrue(status['lift_enabled'])
        requested = dict(servo, manual_pulses_us={'down': 400, 'up': 750})
        self.assertEqual(verified_features(dict(profile, servo=requested))[2], requested['manual_pulses_us'])
        requested = dict(servo, manual_pulses_us={'down': 300, 'up': 750})
        self.assertEqual(verified_features(dict(profile, servo=requested))[2], requested['manual_pulses_us'])
        for down in (325,350,375):
            requested = dict(servo, manual_pulses_us={'down':down, 'up':750})
            self.assertEqual(verified_features(dict(profile, servo=requested))[2],requested['manual_pulses_us'])
        for down in (299,401,350.0,True):
            requested = dict(servo, manual_pulses_us={'down':down, 'up':750})
            self.assertFalse(verified_features(dict(profile,servo=requested))[2])
        for pulse in (540.0, True, 1500, 250):
            bad = dict(servo, manual_pulses_us={'down': 400, 'up': pulse})
            self.assertFalse(verified_features(dict(profile, servo=bad))[2])
        self.assertFalse(verified_features(dict(profile,
            servo=dict(servo, manual_lift_authorized=False)))[2])

    def test_pwm_firmware_requires_validated_web_bridge(self):
        controller = {name: True for name in (
            "wheel_directions_verified", "forward_direction_verified",
            "backward_direction_verified", "positive_yaw_direction_verified",
            "negative_yaw_direction_verified", "command_watchdog_verified")}
        controller["firmware"] = "dase-rc-0.2"
        self.assertFalse(verified_features({"controller": controller})[0])

    def test_pwm_lift_uses_calibrated_microseconds(self):
        controller = {name: True for name in (
            'wheel_directions_verified', 'forward_direction_verified',
            'backward_direction_verified', 'positive_yaw_direction_verified',
            'negative_yaw_direction_verified', 'command_watchdog_verified', 'web_drive_validated')}
        controller['firmware'] = 'dase-rc-0.4'
        servo = {'safe_range_verified': True, 'calibrated_pulses_us': {'up': 1540, 'down': 1500}}
        self.assertEqual(verified_features({'controller': controller, 'servo': servo})[2], servo['calibrated_pulses_us'])
        for pulses in ({'up': 40, 'down': 0}, {'up': 1500, 'down': 1500}, {'up': 2301, 'down': 1500}):
            self.assertFalse(verified_features({'controller': controller,
                'servo': dict(servo, calibrated_pulses_us=pulses)})[2])

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.state = WebState("local-test-passphrase", {}, self.temp.name)
        self.server = RobotHTTPServer(("127.0.0.1", 0), self.state)
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.cookie = None

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join()
        self.temp.cleanup()

    def request(self, method, path, body=None, headers=None):
        connection = http.client.HTTPConnection("127.0.0.1", self.server.server_port, timeout=3)
        h = {"Content-Type": "application/json"}
        if self.cookie:
            h["Cookie"] = self.cookie
        h.update(headers or {})
        connection.request(method, path, body=None if body is None else json.dumps(body), headers=h)
        response = connection.getresponse()
        data = response.read()
        cookie = response.getheader("Set-Cookie")
        if cookie:
            self.cookie = cookie.split(";")[0]
        status = response.status
        connection.close()
        return status, data

    def login(self):
        status, _ = self.request("POST", "/api/login", {"password": self.state.password})
        self.assertEqual(status, 200)

    def test_login_and_locked_hardware(self):
        self.assertEqual(self.request("GET", "/api/status")[0], 401)
        self.login()
        status, data = self.request("GET", "/api/status")
        self.assertEqual(status, 200)
        self.assertFalse(json.loads(data)["drive_enabled"])
        self.assertEqual(self.request("POST", "/api/arm", {})[0], 409)

    def test_reject_cross_site_and_dns_rebinding(self):
        for headers in ({"Origin": "https://example.org"}, {"Host": "example.org"},
                        {"Sec-Fetch-Site": "cross-site"}):
            self.assertEqual(self.request("POST", "/api/login", {"password": self.state.password}, headers)[0], 403)

    def test_login_rate_limit_and_invalid_body(self):
        for _ in range(5):
            self.assertEqual(self.request("POST", "/api/login", {"password": "wrong"})[0], 403)
        self.assertEqual(self.request("POST", "/api/login", {"password": self.state.password})[0], 403)
        self.assertEqual(self.request("POST", "/api/login", ["array"])[0], 400)

    def test_expired_session_denies_camera(self):
        self.login()
        token = self.cookie.split("=", 1)[1]
        self.state.sessions[token] = time.monotonic() - 1
        self.assertEqual(self.request("GET", "/api/frame")[0], 401)

    def test_mjpeg_authentication_new_frames_and_session_revocation(self):
        self.exercise_video_stream('/api/stream', 'multipart/x-mixed-replace')

    def test_safari_binary_video_authentication_frames_and_revocation(self):
        self.exercise_video_stream('/api/video', 'application/octet-stream')

    def exercise_video_stream(self, path, content_type):
        self.state.set_frame(b'first-jpeg')
        self.assertEqual(self.request('GET', path)[0], 401)
        self.login()
        connection = http.client.HTTPConnection('127.0.0.1', self.server.server_port, timeout=2)
        connection.request('GET', path, headers={'Cookie': self.cookie})
        response = connection.getresponse()
        self.assertEqual(response.status, 200)
        self.assertIn(content_type, response.getheader('Content-Type'))

        def frame():
            self.assertEqual(response.readline(), b'--frame\r\n')
            headers = {}
            while True:
                line = response.readline()
                if line == b'\r\n':
                    break
                key, value = line.decode().split(':', 1)
                headers[key] = value.strip()
            data = response.read(int(headers['Content-Length']))
            self.assertEqual(response.read(2), b'\r\n')
            return data

        try:
            self.assertEqual(frame(), b'first-jpeg')
            self.state.set_frame(b'second-jpeg')
            self.assertEqual(frame(), b'second-jpeg')
            self.assertEqual(self.request('GET', '/api/status')[0], 200)
            with self.state.lock:
                self.state.sessions.clear()
            self.state.set_frame(b'after-logout')
            self.assertEqual(response.read(), b'')
        finally:
            connection.close()

    def test_mjpeg_stale_frame_rejected_and_stream_cap_reserves_control_slots(self):
        self.login()
        self.state.set_frame(b'stale', time.monotonic()-2)
        self.assertEqual(self.request('GET', '/api/stream')[0], 503)
        self.state.set_frame(b'fresh')
        for _ in range(3):
            self.assertTrue(self.server.stream_slots.acquire(False))
        try:
            self.assertEqual(self.request('GET', '/api/stream')[0], 503)
            self.assertEqual(self.request('GET', '/api/status')[0], 200)
        finally:
            for _ in range(3):
                self.server.stream_slots.release()

    def test_stale_camera_and_snapshot_path(self):
        self.login()
        self.assertEqual(self.request("GET", "/api/frame")[0], 503)
        self.assertEqual(self.request("POST", "/api/snapshot", {})[0], 409)
        self.state.jpeg = b"test-frame"
        self.state.frame_time = time.monotonic()
        status, data = self.request("POST", "/api/snapshot", {})
        self.assertEqual(status, 200)
        image_path = json.loads(data)["image"]
        self.assertEqual(self.request("GET", image_path), (200, b"test-frame"))
        self.assertEqual(self.request("GET", "/evidence/../../README.md")[0], 404)
        self.assertEqual(self.request("POST", "/api/snapshot", {})[0], 409)

    def test_two_sessions_cannot_replace_active_driver(self):
        self.state.control.drive_enabled = True
        self.state.control.gate.update_feedback(True, time.monotonic())
        self.login()
        status, data = self.request("POST", "/api/arm", {})
        self.assertEqual(status, 200)
        lease = json.loads(data)["lease"]
        first_cookie = self.cookie
        self.login()
        self.assertNotEqual(first_cookie, self.cookie)
        self.assertEqual(self.request("POST", "/api/arm", {})[0], 409)
        self.assertEqual(self.request("POST", "/api/drive", {"lease": lease, "sequence": 1,
                         "server_clock": time.monotonic(), "forward": 1})[0], 409)
        self.assertEqual(self.request("POST", "/api/stop", {})[0], 200)
        self.assertFalse(self.state.control.gate.armed)

    def test_logout_revokes_cookie(self):
        self.login()
        self.request("POST", "/api/logout", {})
        self.assertEqual(self.request("GET", "/api/status")[0], 401)

    def test_all_drive_acceptance_flags_required(self):
        flags = {key: True for key in ("wheel_directions_verified", "forward_direction_verified",
                 "backward_direction_verified", "positive_yaw_direction_verified",
                 "negative_yaw_direction_verified", "command_watchdog_verified")}
        self.assertTrue(verified_features({"controller": flags})[0])
        for key in flags:
            self.assertFalse(verified_features({"controller": dict(flags, **{key: False})})[0])

    def test_lateral_is_rejected_and_latched(self):
        self.state.control.drive_enabled = True
        now = time.monotonic()
        self.state.control.gate.update_feedback(True, now)
        self.login()
        _, data = self.request("POST", "/api/arm", {})
        lease = json.loads(data)["lease"]
        self.assertEqual(self.request("POST", "/api/drive", {"lease": lease, "sequence": 1,
                         "server_clock": time.monotonic(), "lateral": 1})[0], 409)
        self.assertFalse(self.state.control.gate.armed)


if __name__ == "__main__":
    unittest.main()
