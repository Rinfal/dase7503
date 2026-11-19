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
