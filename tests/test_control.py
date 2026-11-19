import math
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "robot_ws/src/logistics_robot"))

from logistics_robot.mission import Job, Mission, Pose, State
from logistics_robot.qr import DetectionGate, EvidenceStore, parse_qr
from logistics_robot.safety import SafetyGate, SafetyLimits, Velocity


class QRTests(unittest.TestCase):
    def test_official_samples(self):
        samples = ("START", "END", "RACKA_4X6M", "RACKB_AM9L", "RACKC_K8Y3", "RACKD_R983")
        for payload in samples:
            self.assertEqual(parse_qr(payload).payload, payload)

    def test_invalid_payloads_rejected(self):
        for payload in ("start", " START", "START\n", "RACKE_1234", "RACKA_12345",
                        "RACKA_12!4", "RACKA_", None, 12):
            with self.subTest(payload=payload):
                self.assertIsNone(parse_qr(payload))

    def test_consecutive_frames_required(self):
        gate = DetectionGate()
        self.assertFalse(gate.observe(["START", "START"], 0))
        self.assertFalse(gate.observe(["START"], 0.1))
        self.assertEqual(gate.observe(["START"], 0.2)[0].kind, "START")

    def test_missing_frame_resets_confirmation(self):
        gate = DetectionGate(count=2)
        gate.observe(["START"], 0)
        gate.observe([], 0.1)
        self.assertFalse(gate.observe(["START"], 0.2))

    def test_old_frames_not_combined(self):
        gate = DetectionGate(count=2)
        gate.observe(["START"], 0)
        self.assertFalse(gate.observe(["START"], 2))

    def test_evidence_only_deduplicated_after_success(self):
        with tempfile.TemporaryDirectory() as directory:
            store = EvidenceStore(directory)
            code = parse_qr("START")
            with self.assertRaises(OSError):
                store.record(code, lambda filename: False)
            self.assertTrue(store.record(code))
            self.assertFalse(store.record(code))
            self.assertEqual(len((Path(directory) / "qr_detections.jsonl").read_text().splitlines()), 1)


class SafetyTests(unittest.TestCase):
    def ready_gate(self):
        gate = SafetyGate()
        gate.emergency_stop(False)
        gate.update_scan([2.0] * 360, math.pi / 180, 0.05, 20.0, 1.0)
        gate.update_odom(True, 1.0)
        self.assertTrue(gate.arm(1.0)[0])
        gate.update_command(Velocity(0.1, 0, 0), 1.0)
        return gate

    def test_startup_never_moves(self):
        gate = SafetyGate()
        gate.update_command(Velocity(1, 1, 1), 0)
        self.assertEqual(gate.output(0), Velocity())
        self.assertFalse(gate.arm(0)[0])

    def test_fresh_sensors_and_arm_enable_command(self):
        gate = self.ready_gate()
        self.assertEqual(gate.output(1.1), Velocity(0.1, 0, 0))

    def test_vector_speed_is_limited(self):
        gate = self.ready_gate()
        gate.update_command(Velocity(1, 1, 10), 1)
        output = gate.output(1.1)
        self.assertAlmostEqual(math.hypot(output.forward, output.lateral), 0.15)
        self.assertEqual(output.angular, 0.5)

    def test_command_watchdog_latches_disarm(self):
        gate = self.ready_gate()
        self.assertEqual(gate.output(1.26), Velocity())
        self.assertFalse(gate.armed)
        gate.update_command(Velocity(0.1, 0, 0), 1.27)
        self.assertEqual(gate.output(1.27), Velocity())

    def test_sensor_dropout_latches_disarm(self):
        gate = self.ready_gate()
        gate.update_command(Velocity(0.1, 0, 0), 1.32)
        self.assertEqual(gate.output(1.32), Velocity())
        self.assertFalse(gate.armed)

    def test_nonfinite_commands_stop(self):
        for value in (math.nan, math.inf, -math.inf):
            gate = self.ready_gate()
            gate.update_command(Velocity(value, 0, 0), 1)
            self.assertEqual(gate.output(1.01), Velocity())
            self.assertFalse(gate.armed)

    def test_incomplete_scan_stops(self):
        gate = self.ready_gate()
        gate.update_scan([2.0] * 180, math.pi / 180, 0.05, 20.0, 1.01)
        self.assertEqual(gate.output(1.02), Velocity())

    def test_invalid_scan_cannot_arm(self):
        for ranges in ([math.nan] * 360, [0] * 360, [-math.inf] * 360, []):
            gate = self.ready_gate()
            gate.update_scan(ranges, math.pi / 180, 0.05, 20.0, 1.01)
            self.assertEqual(gate.output(1.02), Velocity())

    def test_estop_cannot_auto_resume(self):
        gate = self.ready_gate()
        gate.emergency_stop(True)
        self.assertEqual(gate.output(1.01), Velocity())
        gate.emergency_stop(False)
        self.assertEqual(gate.output(1.02), Velocity())

    def test_lateral_or_rotation_near_obstacle_stops(self):
        for command in (Velocity(0, 0.1, 0), Velocity(0, 0, 0.2)):
            gate = self.ready_gate()
            ranges = [math.inf] * 360
            ranges[90] = 0.25
            gate.update_scan(ranges, math.pi / 180, 0.05, 20, 1.01)
            gate.update_command(command, 1.01)
            self.assertEqual(gate.output(1.02), Velocity())

    def test_braking_margin_is_enforced(self):
        gate = self.ready_gate()
        gate.update_scan([0.35] * 360, math.pi / 180, 0.05, 20, 1)
        gate.update_command(Velocity(0.15, 0, 0), 1)
        self.assertEqual(gate.output(1.01), Velocity())
        self.assertEqual(gate.reason, "braking_distance")

    def test_stale_command_not_replayed_after_rearm(self):
        gate = self.ready_gate()
        gate.disarm()
        self.assertTrue(gate.arm(1.05)[0])
        self.assertEqual(gate.output(1.06), Velocity())
        self.assertTrue(gate.armed)

    def test_safety_parameters_reject_nan(self):
        with self.assertRaises(ValueError):
            SafetyLimits(max_linear=math.nan)


class MissionTests(unittest.TestCase):
    def mission(self, duration=180):
        pose = Pose(1, 1, 0)
        jobs = [Job("A", "RACKA_4X6M", pose, pose, pose),
                Job("B", "RACKB_AM9L", pose, pose, pose)]
        return Mission(jobs, duration=duration)

    def reach_identify(self, mission):
        actions = mission.qr("START", 0)
        mission.complete(actions[0].token, True, 1)
        self.assertEqual(mission.state, State.IDENTIFY)

    def test_no_movement_before_start(self):
        mission = self.mission()
        self.assertFalse(mission.qr("END", 0))
        self.assertFalse(mission.qr("RACKA_4X6M", 0))
        self.assertFalse(mission.tick(500))
        self.assertEqual(mission.state, State.WAIT_START)

    def test_wrong_team_or_wrong_rack_ignored(self):
        mission = self.mission()
        self.reach_identify(mission)
        for payload in ("RACKA_0000", "RACKB_AM9L", "START"):
            self.assertFalse(mission.qr(payload, 2))
        self.assertEqual(mission.qr("RACKA_4X6M", 2)[0].kind, "dock")

    def test_old_action_completion_ignored(self):
        mission = self.mission()
        self.reach_identify(mission)
        mission.qr("RACKA_4X6M", 2)
        self.assertFalse(mission.complete(1, True, 3))
        self.assertEqual(mission.state, State.DOCK)

    def test_failed_action_cancels_mission(self):
        mission = self.mission()
        mission.qr("START", 0)
        self.assertEqual(mission.complete(mission.token, False, 1)[0].kind, "stop")
        self.assertEqual(mission.state, State.FAULT)
        self.assertFalse(mission.qr("START", 2))

    def test_timeout_holds_loaded_rack(self):
        mission = self.mission(duration=8)
        self.reach_identify(mission)
        mission.qr("RACKA_4X6M", 2)
        mission.complete(mission.token, True, 3)
        mission.complete(mission.token, True, 4)
        self.assertTrue(mission.carrying)
        self.assertEqual(mission.tick(8)[0].kind, "stop")
        self.assertTrue(mission.carrying)
        self.assertFalse(mission.delivered)

    def test_no_delivery_without_end_and_retreat(self):
        mission = self.mission()
        self.reach_identify(mission)
        mission.qr("RACKA_4X6M", 2)
        for now in (3, 4, 5):
            mission.complete(mission.token, True, now)
        self.assertEqual(mission.state, State.VERIFY_END)
        self.assertFalse(mission.complete(mission.token, True, 6))
        mission.qr("END", 6)
        mission.complete(mission.token, True, 7)
        self.assertFalse(mission.delivered)
        mission.complete(mission.token, True, 8)
        self.assertEqual(mission.delivered, ["A"])

    def test_full_two_rack_flow(self):
        mission = self.mission()
        mission.qr("START", 0)
        for now in range(1, 30):
            if mission.state == State.IDENTIFY:
                mission.qr(mission.job.payload, now)
            elif mission.state == State.VERIFY_END:
                mission.qr("END", now)
            else:
                mission.complete(mission.token, True, now)
            if mission.state == State.FINISHED:
                break
        self.assertEqual(mission.state, State.FINISHED)
        self.assertEqual(mission.delivered, ["A", "B"])
        self.assertFalse(mission.carrying)

    def test_stage_timeout(self):
        mission = self.mission()
        mission.qr("START", 0)
        mission.tick(25)
        self.assertEqual(mission.state, State.FAULT)

    def test_invalid_team_configuration(self):
        pose = Pose(1, 1, 0)
        with self.assertRaises(ValueError):
            Job("A", "RACKB_AM9L", pose, pose, pose)


if __name__ == "__main__":
    unittest.main()
