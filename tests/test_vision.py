"""Decode actual generated QR pixels; never connect to a real robot."""
import json
import sys
import tempfile
import unittest
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'robot_ws/src/logistics_robot'))
from logistics_robot.vision import VisionProcessor
from logistics_robot.qr import parse_qr


def qr_frame(payload):
    qr = cv2.QRCodeEncoder_create().encode(payload)
    qr = cv2.copyMakeBorder(qr, 4, 4, 4, 4, cv2.BORDER_CONSTANT, value=255)
    qr = cv2.resize(qr, (350, 350), interpolation=cv2.INTER_NEAREST)
    frame = np.full((540, 960, 3), 255, dtype=np.uint8)
    frame[90:440, 300:650] = cv2.cvtColor(qr, cv2.COLOR_GRAY2BGR)
    return frame


class VisionTests(unittest.TestCase):
    def test_real_qr_confirmation_jpeg_evidence_and_restart_deduplication(self):
        with tempfile.TemporaryDirectory() as root:
            processor = VisionProcessor(root)
            for payload, kind in [('RACKA_4X6M', 'RACK'), ('https://example.com/../扫码?x=<script>', 'TEXT')]:
                image = qr_frame(payload)
                before = image.copy()
                for stamp in (0, .25):
                    self.assertFalse(processor.process(image, stamp)[1])
                jpeg, events, annotated = processor.process(image, .5)
                self.assertEqual(len(events), 1)
                self.assertEqual(events[0]['payload'], payload)
                self.assertEqual(events[0]['kind'], kind)
                self.assertEqual(cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR).shape, (540, 960, 3))
                self.assertTrue(np.array_equal(before, image), 'original pixels must not be modified')
                self.assertFalse(np.array_equal(annotated, image), 'QR box must be drawn')
                self.assertEqual(Path(events[0]['image']).name, events[0]['image'])
                self.assertTrue((Path(root)/events[0]['image']).is_file())
                self.assertFalse(processor.process(image, .75)[1])
                processor = VisionProcessor(root)
            records = [json.loads(line) for line in (Path(root)/'qr_detections.jsonl').read_text().splitlines()]
            self.assertEqual(len(records), 2)
            self.assertIsNone(parse_qr(records[-1]['payload']), 'generic text must not become a course navigation code')
            for stamp in (0, .25):
                self.assertFalse(processor.process(image, stamp)[1])
            repeated = processor.process(image, .5)[1]
            self.assertEqual(repeated[0]['payload'], records[-1]['payload'])
            self.assertFalse(repeated[0]['new_evidence'])
            self.assertEqual(len((Path(root)/'qr_detections.jsonl').read_text().splitlines()), 2)

    def test_missing_detection_resets_confirmation(self):
        with tempfile.TemporaryDirectory() as root:
            processor = VisionProcessor(root)
            image = qr_frame('START')
            self.assertFalse(processor.process(image, 0)[1])
            self.assertFalse(processor.process(np.full_like(image, 255), .25)[1])
            for stamp in (.5, .75):
                self.assertFalse(processor.process(image, stamp)[1])
            self.assertEqual(processor.process(image, 1)[1][0]['payload'], 'START')


if __name__ == '__main__':
    unittest.main()
