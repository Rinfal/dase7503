"""Camera-only processing. No access to drive or lift commands."""
import time
from datetime import datetime, timezone

import cv2
from pyzbar.pyzbar import ZBarSymbol, decode

from .qr import DetectionGate, EvidenceStore


class VisionProcessor:
    def __init__(self, directory, confirmations=3, scan_rate=5, width=960, quality=70):
        self.gate = DetectionGate(count=confirmations, max_gap=.65, allow_generic=True)
        self.store = EvidenceStore(directory)
        self.scan_period = 1/scan_rate
        self.width = width
        self.quality = quality
        self.last_scan = -float('inf')
        self.boxes = []
        self.last_confirmed = set()

    def process(self, original, now=None):
        now = time.monotonic() if now is None else now
        frame = original.copy()
        events = []
        scanning = now-self.last_scan >= self.scan_period
        if scanning:
            self.last_scan = now
            self.boxes = []
            gray = cv2.cvtColor(original, cv2.COLOR_BGR2GRAY)
            payloads = []
            for item in decode(gray, symbols=[ZBarSymbol.QRCODE]):
                try:
                    payload = item.data.decode('utf-8', errors='strict')
                except UnicodeDecodeError:
                    continue
                payloads.append(payload)
                self.boxes.append((item.rect, payload))
            codes = self.gate.observe(payloads, now)
        else:
            codes = []
        for bounds, payload in self.boxes:
            cv2.rectangle(frame, (bounds.left, bounds.top),
                (bounds.left+bounds.width, bounds.top+bounds.height), (74, 230, 125), 2)
            label = payload[:48] if payload.isascii() else 'QR detected'
            cv2.putText(frame, label, (bounds.left, max(22, bounds.top-8)),
                cv2.FONT_HERSHEY_SIMPLEX, .55, (74, 230, 125), 2)
        for code in codes:
            saved = self.store.record(code, lambda filename: cv2.imwrite(filename, frame))
            if code.payload not in self.last_confirmed:
                event = dict(saved or self.store.records[code.payload])
                event.update(observed_at_utc=datetime.now(timezone.utc).isoformat(), new_evidence=bool(saved))
                events.append(event)
        if scanning:
            self.last_confirmed = {code.payload for code in codes}
        if frame.shape[1] > self.width:
            frame = cv2.resize(frame, (self.width, round(frame.shape[0]*self.width/frame.shape[1])),
                               interpolation=cv2.INTER_AREA)
        success, encoded = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, self.quality])
        if not success:
            raise OSError('无法编码预览画面')
        return encoded.tobytes(), events, frame
