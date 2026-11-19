import json
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'robot_ws/src/logistics_robot'))
from logistics_robot.drive_pwm import DrivePWM
from logistics_robot.pwm_bridge import PWMBridge, decode_status
from logistics_robot.remote_control import RemoteControl
from logistics_robot.safety import Velocity


class BridgeTests(unittest.TestCase):
    def setUp(self):
        self.bridge = PWMBridge(DrivePWM([-1, 1, -1, 1], 600))
        self.control = RemoteControl(drive_enabled=True, lift_angles={'up': 1540, 'down': 1500})
        self.clock = 1000000

    def feedback(self, now, armed=False, timeout=False, encoders=None):
        self.clock += 50000
        status = {'firmware': 'dase-rc-0.4', 'armed': armed, 'watchdog_timeout': timeout,
                  'servo_active': False, 'board_us': self.clock,
                  'pwm': [0]*4, 'encoders': encoders or [0]*4}
        valid, reboot = self.bridge.feedback(status, now)
        self.control.gate.update_feedback(valid, now)
        return reboot

    def ready(self):
        self.feedback(10)
        self.control.arm(10)
        self.assertEqual(self.bridge.step(self.control, Velocity(.08), 10), ([0]*4, False, None))
        self.feedback(10.13)
        self.assertEqual(self.bridge.step(self.control, Velocity(.08), 10.13), ([0]*4, True, None))
        self.feedback(10.18, armed=True)

    def test_requires_actual_arm_ack_before_nonzero(self):
        self.ready()
        self.assertEqual(self.bridge.step(self.control, Velocity(.08), 10.18), ([-600, 600, -600, 600], None, None))

    def test_board_timeout_locks_instead_of_auto_rearming(self):
        self.ready()
        self.bridge.step(self.control, Velocity(.08), 10.18)
        self.feedback(10.2, timeout=True)
        self.assertEqual(self.bridge.step(self.control, Velocity(.08), 10.2), ([0]*4, False, None))
        self.assertFalse(self.control.gate.armed)
        self.assertFalse(self.control.lease)

    def test_feedback_loss_sends_zero_and_disarm(self):
        self.ready()
        self.assertEqual(self.bridge.step(self.control, Velocity(.08), 10.6), ([0]*4, False, None))
        self.assertFalse(self.control.gate.armed)

    def test_lift_waits_for_stationary_encoder_feedback(self):
        self.ready()
        self.control.pending_lift = 1540
        self.assertIsNone(self.bridge.step(self.control, Velocity(), 10.18)[2])
        self.feedback(10.4, armed=True)
        self.assertEqual(self.bridge.step(self.control, Velocity(), 10.4), ([0]*4, None, 1540))
        self.assertIsNone(self.control.pending_lift)
        self.assertEqual(self.bridge.step(self.control, Velocity(.08), 10.41)[0], [0]*4)

    def test_reboot_and_duplicate_clock_are_not_fresh_feedback(self):
        self.feedback(10)
        status = dict(self.bridge.board)
        self.assertEqual(self.bridge.feedback(status, 11), (False, False))
        status['board_us'] = 100
        self.assertEqual(self.bridge.feedback(status, 11), (False, True))

    def test_malformed_status_does_not_enable_controller(self):
        self.feedback(10)
        status = dict(self.bridge.board)
        self.assertEqual(decode_status(json.dumps(status), 'dase-rc-0.4'), status)
        for changes in ({'pwm': [0]}, {'encoders': [False]*4}, {'board_us': 0},
                        {'armed': 1}, {'pwm': [601]*4}, {'firmware': 'factory'}):
            with self.assertRaises(ValueError):
                decode_status(json.dumps(dict(status, **changes)), 'dase-rc-0.4')


if __name__ == '__main__':
    unittest.main()
