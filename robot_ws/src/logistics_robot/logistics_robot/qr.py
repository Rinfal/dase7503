import json
import re
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path


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


class DetectionGate:
    def __init__(self, count=3, max_gap=0.5):
        if type(count) is not int or count < 1 or max_gap <= 0:
            raise ValueError("连续确认帧数和最大帧间隔必须为正数")
        self.count = count
        self.max_gap = max_gap
        self.history = {}

    def observe(self, payloads, now):
        valid = {payload for payload in payloads if parse_qr(payload)}
        self.history = {
            payload: history for payload, history in self.history.items()
            if payload in valid and now - history[1] <= self.max_gap
        }
        result = []
        for payload in sorted(valid):
            previous_count, _ = self.history.get(payload, (0, now))
            self.history[payload] = (previous_count + 1, now)
            if previous_count + 1 >= self.count:
                result.append(parse_qr(payload))
        return result


class EvidenceStore:
    def __init__(self, directory):
        self.directory = Path(directory).expanduser()
        self.directory.mkdir(parents=True, exist_ok=True)
        self.seen = set()

    def record(self, code, image_writer=None):
        if code.payload in self.seen:
            return False
        stamp = datetime.now(timezone.utc)
        basename = stamp.strftime("%Y%m%dT%H%M%S.%fZ") + "_" + code.payload
        evidence = asdict(code)
        evidence["timestamp_utc"] = stamp.isoformat()
        if image_writer is not None:
            filename = self.directory / (basename + ".png")
            if not image_writer(str(filename)):
                raise OSError("无法保存二维码证据图片")
            evidence["image"] = filename.name
        with (self.directory / "qr_detections.jsonl").open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(evidence, ensure_ascii=False) + "\n")
        self.seen.add(code.payload)
        return True
