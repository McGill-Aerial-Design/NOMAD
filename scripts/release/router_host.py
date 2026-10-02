# SPDX-License-Identifier: Apache-2.0
"""Foreground host for an operator-provisioned standalone router scheduled task."""

from __future__ import annotations

import argparse
import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from scripts.release import storage
from scripts.release.lifecycle import Deployment
from scripts.release.router import executable


def supervise(root: Path, config: Path) -> int:
    engine = Deployment(root, "router")
    current = storage.read_json(engine.root / "current.json")
    verified = engine.get_release(current["release_version"])
    if current != verified:
        raise ValueError("router active pointer differs from verified immutable release")
    storage.reject_links(config)
    if not config.is_file() or config.is_relative_to(engine.root):
        raise ValueError("authoritative router configuration must remain external")
    stop = engine.root / "stop.request"
    process = engine.root / "process.json"
    stop.unlink(missing_ok=True)
    with (engine.root / "supervisor.log").open("ab") as log:
        child = subprocess.Popen(
            [str(executable(verified)), str(config)], stdin=subprocess.PIPE, stdout=log, stderr=log
        )
        storage.write_json(process, {"status": "running", "release_version": verified["release_version"]})
        while child.poll() is None:
            if stop.exists():
                child.stdin.write(b"stop\n")
                child.stdin.flush()
                try:
                    result = child.wait(timeout=30)
                except subprocess.TimeoutExpired:
                    storage.write_json(
                        process, {"status": "stop_failed", "release_version": verified["release_version"]}
                    )
                    continue
                storage.write_json(
                    process,
                    {"status": "stopped" if result == 0 else "failed", "release_version": verified["release_version"]},
                )
                return result
            time.sleep(0.1)
        storage.write_json(process, {"status": "failed", "release_version": verified["release_version"]})
        return child.returncode or 1


def run(root: Path, config: Path) -> int:
    with storage.lock(root / "router" / "supervisor"):
        return supervise(root, config)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--config", type=Path, required=True)
    args = parser.parse_args()
    return run(args.root.absolute(), args.config.absolute())


if __name__ == "__main__":
    raise SystemExit(main())
