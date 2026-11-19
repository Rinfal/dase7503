#!/usr/bin/env python3
import json
import subprocess
import sys
import tempfile
from pathlib import Path

from PIL import Image
from pyzbar.pyzbar import ZBarSymbol, decode

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "robot_ws/src/logistics_robot"))

from logistics_robot.qr import parse_qr


def main():
    root = Path(__file__).resolve().parents[1]
    source = root / "4.+DASE7503_Appendix.Samples+of+QR+codes.pdf"
    found = set()
    with tempfile.TemporaryDirectory() as directory:
        subprocess.run(["pdftoppm", "-r", "180", "-png", str(source),
                        str(Path(directory) / "page")], check=True)
        for filename in sorted(Path(directory).glob("*.png")):
            with Image.open(filename) as image:
                for result in decode(image, symbols=[ZBarSymbol.QRCODE]):
                    payload = result.data.decode("utf-8")
                    if parse_qr(payload):
                        found.add(payload)
    expected = {"START", "END", "RACKA_4X6M", "RACKB_AM9L", "RACKC_K8Y3", "RACKD_R983"}
    report = {"source": source.name, "sample_validation_only": True,
              "found": sorted(found), "expected": sorted(expected),
              "success": found == expected,
              "note": "验证课程 PDF 样本解码，不是摄像头采集或本组比赛识别证据。"}
    text = json.dumps(report, ensure_ascii=False, indent=2)
    print(text)
    (root / "docs/evidence/sample_qr_validation.json").write_text(text + "\n", encoding="utf-8")
    return 0 if report["success"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
