import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "robot_ws/src/logistics_robot"))

from logistics_robot.manual import ManualGate, ManualLimits
from logistics_robot.safety import Velocity


class ManualTests(unittest.TestCase):
    def ready_gate(self):
        gate = ManualGate()
        gate.update_feedback(True, 10.0)
        gate.emergency_stop(False)
        self.assertTrue(gate.arm(10.0)[0])
        return gate

    def test_starts_locked(self):
        gate = ManualGate()
        gate.update_feedback(True, 10)
        self.assertFalse(gate.arm(10)[0])
        self.assertEqual(gate.output(10), Velocity())

    def test_requires_controller_feedback(self):
        gate = ManualGate()
        gate.emergency_stop(False)
        self.assertFalse(gate.arm(10)[0])

    def test_arm_discards_old_command(self):
        gate = self.ready_gate()
        gate.update_command(Velocity(0.08), 10.0)
        gate.disarm()
        self.assertTrue(gate.arm(10.1)[0])
        self.assertEqual(gate.output(10.1), Velocity())

    def test_diagonal_speed_is_bounded(self):
        gate = self.ready_gate()
        gate.update_command(Velocity(5, 5, 10), 10)
        output = gate.output(10.1)
        self.assertAlmostEqual(math.hypot(output.forward, output.lateral), 0.08)
        self.assertEqual(output.angular, 0.35)

    def test_command_loss_latches_disarm(self):
        gate = self.ready_gate()
        gate.update_command(Velocity(0.05), 10)
        gate.update_feedback(True, 10.3)
        self.assertEqual(gate.output(10.3), Velocity())
        self.assertFalse(gate.armed)
        gate.update_command(Velocity(0.05), 10.31)
        self.assertEqual(gate.output(10.31), Velocity())

    def test_feedback_loss_latches_disarm(self):
        gate = self.ready_gate()
        gate.update_command(Velocity(0.05), 10.6)
        self.assertEqual(gate.output(10.6), Velocity())
        self.assertEqual(gate.reason, "controller_feedback_stale")
        self.assertFalse(gate.armed)

    def test_emergency_stop_requires_new_arm(self):
        gate = self.ready_gate()
        gate.emergency_stop(True)
        gate.emergency_stop(False)
        gate.update_command(Velocity(0.05), 10)
        self.assertEqual(gate.output(10), Velocity())

    def test_invalid_command_disarms(self):
        gate = self.ready_gate()
        self.assertFalse(gate.update_command(Velocity(math.nan), 10))
        self.assertFalse(gate.armed)
        self.assertEqual(gate.output(10), Velocity())

    def test_negative_clock_age_is_not_fresh(self):
        gate = self.ready_gate()
        self.assertEqual(gate.output(9.9), Velocity())
        self.assertFalse(gate.armed)

    def test_invalid_limits(self):
        for value in (0, -1, math.nan, math.inf):
            with self.assertRaises(ValueError):
                ManualLimits(command_timeout=value)


if __name__ == "__main__":
    unittest.main()
