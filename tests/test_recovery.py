import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'robot_ws/src/logistics_robot'))
from logistics_robot.durability import atomic_path, atomic_write
from logistics_robot.qr import EvidenceStore, parse_qr
from logistics_robot.recovery import RecoveryPolicy


class RecoveryTests(unittest.TestCase):
    def test_stalled_camera_recovers_without_restarting_healthy_dependents(self):
        policy = RecoveryPolicy()
        units = {kind: ('active', 0) for kind in ('camera','vision','agent','remote')}
        seen = {'camera':10, 'vision':10, 'agent':100}
        self.assertEqual(policy.choose(100,units,seen,True,True), ['camera'])
        self.assertEqual(policy.choose(110,units,dict(seen,agent=110),True,True), [])
        self.assertEqual(policy.choose(120,units,{'camera':120,'agent':120},True,True), ['vision'])
        self.assertEqual(policy.choose(125,units,{'camera':125,'vision':125,'agent':125},True,True), [])

    def test_startup_manual_stop_and_missing_usb_are_respected(self):
        policy = RecoveryPolicy()
        units = {kind: ('active', 0) for kind in ('camera','vision','agent','remote')}
        self.assertEqual(policy.choose(20,units,{},False,True), [])
        units['camera'] = ('inactive',0)
        units['vision'] = ('inactive',0)
        self.assertEqual(policy.choose(100,units,{},True,False), [])
        self.assertEqual(policy.choose(105,units,{},False,True), ['agent','remote'])

    def test_interrupted_atomic_file_keeps_previous_complete_version(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'profile.yaml'
            atomic_write(path,b'previous complete profile')
            with self.assertRaises(RuntimeError):
                with atomic_path(path) as temporary:
                    temporary.write_bytes(b'incomplete')
                    raise RuntimeError('simulated interruption')
            self.assertEqual(path.read_bytes(),b'previous complete profile')
            self.assertEqual(list(path.parent.iterdir()),[path])
            atomic_write(path,b'next complete profile')
            self.assertEqual(path.read_bytes(),b'next complete profile')

    def test_qr_append_recovers_after_torn_last_record(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'qr_detections.jsonl'
            store = EvidenceStore(directory)
            store.record(parse_qr('START'))
            with path.open('ab') as stream:
                stream.write(b'{"payload":"partially_written')
            EvidenceStore(directory).record(parse_qr('END'))
            self.assertEqual(EvidenceStore(directory).seen,{'START','END'})

    def test_services_only_install_preserves_custom_profile(self):
        with tempfile.TemporaryDirectory() as home:
            config = Path(home)/'.config/dase7503/phone-profile.yaml'
            config.parent.mkdir(parents=True)
            content = b'# custom hardware authorization\nservo: {manual_pulses_us: {down: 350, up: 750}}\n'
            config.write_bytes(content)
            subprocess.run([sys.executable,str(ROOT/'tools/install_phone_remote.py'),'--services-only'],
                env=dict(os.environ,HOME=home),capture_output=True,check=True)
            self.assertEqual(config.read_bytes(),content)
            units = Path(home)/'.config/systemd/user'
            self.assertIn('dase7503-health.service',(units/'dase7503.target').read_text())
            self.assertIn('PartOf=dase7503.target',(units/'dase7503-camera.service').read_text())


if __name__ == '__main__':
    unittest.main()
