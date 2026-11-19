#!/usr/bin/env python3
import json
import subprocess
from pathlib import Path


def run(arguments):
    return subprocess.run(arguments, check=True, text=True, capture_output=True).stdout.strip()


def main():
    root = Path(__file__).resolve().parents[1]
    manifest = json.loads((root / "tools/sources.lock.json").read_text())
    for name, source in manifest.items():
        source_root = root / source.get("workspace", "camera_ws") / "src"
        source_root.mkdir(parents=True, exist_ok=True)
        directory = source_root / name
        if directory.exists():
            if not (directory / ".git").is_dir():
                raise RuntimeError(f"目录已存在且不是 git 仓库：{directory}")
            actual = run(["git", "-C", str(directory), "rev-parse", "HEAD"])
            if actual != source["commit"]:
                raise RuntimeError(f"源码版本不同，保留原目录并停止：{directory}")
        else:
            run(["git", "init", str(directory)])
            run(["git", "-C", str(directory), "remote", "add", "origin", source["url"]])
            run(["git", "-C", str(directory), "fetch", "--depth", "1", "origin", source["commit"]])
            run(["git", "-C", str(directory), "checkout", "--detach", "FETCH_HEAD"])
        if run(["git", "-C", str(directory), "status", "--porcelain"]):
            raise RuntimeError(f"上游源码有本地修改，停止：{directory}")
        print(f"{name}: {source['commit']}", flush=True)


if __name__ == "__main__":
    main()
