import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "robot_ws/src/logistics_robot"))
from logistics_robot.drive_pwm import DrivePWM


class DrivePWMTests(unittest.TestCase):
    def setUp(self):
        # Synthetic calibration, not an assertion about the actual vehicle.
        self.drive = DrivePWM((1, -1, 1, -1))

    def physical(self, command):
        return [value*sign for value, sign in zip(command, self.drive.signs)]

    def test_forward_and_reverse_have_consistent_physical_directions(self):
        self.assertEqual(self.physical(self.drive.output(0.08, 0, 0)), [180]*4)
        self.assertEqual(self.physical(self.drive.output(-0.08, 0, 0)), [-180]*4)

    def test_positive_yaw_moves_right_side_forward(self):
        self.assertEqual(self.physical(self.drive.output(0, 0, 0.35)), [180, -180, 180, -180])

    def test_combined_motion_preserves_ratio_under_saturation(self):
        self.assertEqual(self.physical(self.drive.output(0.08, 0, 0.175)), [180, 60, 180, 60])

    def test_stop_is_zero(self):
        self.assertEqual(self.drive.output(0, 0, 0), [0]*4)

    def test_mecanum_left_right_patterns_and_combined_saturation(self):
        drive = DrivePWM((-1,1,-1,1), duty=600, lateral_enabled=True)
        self.assertEqual(drive.output(0,.08,0), [-600,-600,600,600])
        self.assertEqual(drive.output(0,-.08,0), [600,600,-600,-600])
        self.assertEqual(drive.output(.08,.08,0), [-600,0,0,600])
        self.assertTrue(all(abs(v)<=600 for v in drive.output(.08,.08,.35)))

    def test_assembled_vehicle_calibration_and_board_limit(self):
        drive = DrivePWM((-1, 1, -1, 1), duty=600)
        self.assertEqual(drive.output(0.08, 0, 0), [-600, 600, -600, 600])
        self.assertEqual(drive.output(0, 0, 0.35), [-600]*4)
        with self.assertRaises(ValueError):
            DrivePWM((-1, 1, -1, 1), duty=601)

    def test_invalid_or_unverified_requests_are_rejected(self):
        for values in ((math.nan, 0, 0), (0, 0, math.inf), (0, 0.01, 0)):
            with self.assertRaises(ValueError):
                self.drive.output(*values)
        with self.assertRaises(ValueError):
            DrivePWM((1, 0, 1, -1))


if __name__ == "__main__":
    unittest.main()
