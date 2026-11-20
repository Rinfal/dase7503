import json
import re
import hashlib
import os
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path

from .durability import atomic_path, sync_directory


@dataclass(frozen=True)
class QRCode:
    payload: str
    kind: str
    rack: str = ""
    team_code: str = ""


def parse_qr(payload):
    if not isinstance(payload, str):
        return None
    if payload in ("START", "END"):
        return QRCode(payload, payload)
    match = re.fullmatch(r"RACK([A-D])_([A-Za-z0-9]{4})", payload)
    if match:
        return QRCode(payload, "RACK", match.group(1), match.group(2))
    return None


def parse_display_qr(payload):
    """Recognize course codes, or display bounded UTF-8 text without executing it."""
    if not isinstance(payload, str) or not payload.strip() or len(payload.encode('utf-8')) > 4096:
        return None
    return parse_qr(payload) or QRCode(payload, 'TEXT')


class DetectionGate:
    def __init__(self, count=3, max_gap=0.5, allow_generic=False):
        if type(count) is not int or count < 1 or max_gap <= 0:
            raise ValueError("连续确认帧数和最大帧间隔必须为正数")
        self.count = count
        self.max_gap = max_gap
        self.history = {}
        self.parser = parse_display_qr if allow_generic else parse_qr

    def observe(self, payloads, now):
        valid = {payload for payload in payloads if self.parser(payload)}
        self.history = {
            payload: history for payload, history in self.history.items()
            if payload in valid and now - history[1] <= self.max_gap
        }
        result = []
        for payload in sorted(valid):
            previous_count, _ = self.history.get(payload, (0, now))
            self.history[payload] = (previous_count + 1, now)
            if previous_count + 1 >= self.count:
                result.append(self.parser(payload))
        return result


class EvidenceStore:
    def __init__(self, directory):
        self.directory = Path(directory).expanduser()
        self.directory.mkdir(parents=True, exist_ok=True)
        self.seen = set()
        self.records = {}
        records = self.directory / 'qr_detections.jsonl'
        if records.is_file():
            with records.open(encoding='utf-8') as stream:
                for line in stream:
                    try:
                        record = json.loads(line)
                        payload = record.get('payload')
                        if parse_display_qr(payload):
                            self.seen.add(payload)
                            self.records[payload] = record
                    except (ValueError, AttributeError, TypeError):
                        continue

    def record(self, code, image_writer=None):
        if code.payload in self.seen:
            return False
        stamp = datetime.now(timezone.utc)
        identifier = hashlib.sha256(code.payload.encode('utf-8')).hexdigest()[:16]
        basename = stamp.strftime("%Y%m%dT%H%M%S.%fZ") + "_" + identifier
        evidence = asdict(code)
        evidence["timestamp_utc"] = stamp.isoformat()
        if image_writer is not None:
            filename = self.directory / (basename + ".png")
            with atomic_path(filename) as temporary:
                if not image_writer(str(temporary)):
                    raise OSError("无法保存二维码证据图片")
            evidence["image"] = filename.name
        with (self.directory / "qr_detections.jsonl").open("a+b") as stream:
            # Separate a torn last line from the next complete record after
            # power loss. Readers already ignore malformed/incomplete lines.
            stream.seek(0, os.SEEK_END)
            if stream.tell():
                stream.seek(-1, os.SEEK_END)
                if stream.read(1) != b'\n':
                    stream.write(b'\n')
            stream.write((json.dumps(evidence, ensure_ascii=False) + "\n").encode())
            stream.flush()
            os.fsync(stream.fileno())
        sync_directory(self.directory)
        self.seen.add(code.payload)
        self.records[code.payload] = evidence
        return evidence
