import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "robot_ws/src/logistics_robot"))

from logistics_robot.remote_control import ControlError, RemoteControl
from logistics_robot.safety import Velocity


class RemoteTests(unittest.TestCase):
    def ready(self, lift_angles=None):
        control = RemoteControl(drive_enabled=True, lift_angles=lift_angles)
        control.gate.update_feedback(True, 10)
        lease = control.arm(10)
        return control, {"lease": lease, "sequence": 1, "server_clock": 10}

    def test_default_cannot_arm(self):
        with self.assertRaises(ControlError):
            RemoteControl().arm(10)

    def test_stale_network_packet_stops(self):
        control, body = self.ready()
        with self.assertRaises(ControlError):
            control.drive(body, 10.31)
        self.assertFalse(control.gate.armed)

    def test_old_lease_cannot_control_after_rearm(self):
        control, body = self.ready()
        control.stop()
        control.arm(10.1)
        with self.assertRaises(ControlError):
            control.drive(body, 10.1)
        self.assertEqual(control.output(10.1), Velocity())

    def test_out_of_order_packet_rejected(self):
        control, body = self.ready()
        control.drive(body, 10.01)
        with self.assertRaises(ControlError):
            control.drive(body, 10.02)

    def test_valid_move_and_timeout(self):
        control, body = self.ready()
        control.drive(dict(body, forward=1), 10.01)
        self.assertEqual(control.output(10.1).forward, 0.08)
        self.assertEqual(control.output(10.3), Velocity())

    def test_nonfinite_command_stops(self):
        control, body = self.ready()
        with self.assertRaises(ControlError):
            control.drive(dict(body, angular=float("nan")), 10.01)
        self.assertFalse(control.gate.armed)

    def test_uncalibrated_lift_cannot_move(self):
        control, body = self.ready()
        with self.assertRaises(ControlError):
            control.lift(dict(body, direction="up"), 10.01)
        self.assertIsNone(control.pending_lift)

    def test_lift_blocks_driving(self):
        control, body = self.ready({"up": 5, "down": -5})
        control.lift(dict(body, direction="up"), 10.01)
        self.assertEqual(control.pending_lift, 5)
        with self.assertRaises(ControlError):
            control.drive(dict(body, sequence=2, forward=1), 10.02)
        self.assertEqual(control.output(10.02), Velocity())

    def test_stop_cancels_unpublished_lift(self):
        control, body = self.ready({"up": 5})
        control.lift(dict(body, direction="up"), 10.01)
        control.stop()
        self.assertIsNone(control.pending_lift)


if __name__ == "__main__":
    unittest.main()
