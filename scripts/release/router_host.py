# SPDX-License-Identifier: Apache-2.0
"""Foreground host for an operator-provisioned standalone router scheduled task."""

from __future__ import annotations

import argparse
import contextlib
import os
import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from scripts.release import storage
from scripts.release.lifecycle import Deployment
from scripts.release.router import executable


def stop_child(child) -> int:
    if child.poll() is None:
        try:
            child.stdin.write(b"stop\n")
            child.stdin.flush()
        except BrokenPipeError:
            pass
    return child.wait(timeout=30)


def wait_child(child, stop: Path, process: Path, version: str) -> int:
    while child.poll() is None:
        if stop.exists():
            try:
                return stop_child(child)
            except subprocess.TimeoutExpired:
                storage.write_json(process, {"status": "stop_failed", "release_version": version})
        time.sleep(0.1)
    return child.returncode or 1


def finish_child(child, process: Path, version: str) -> None:
    # Keep the lifetime lock until both process exit and its final record are established.
    while child.poll() is None:
        try:
            stop_child(child)
        except subprocess.TimeoutExpired:
            time.sleep(0.1)
    with contextlib.suppress(OSError):
        child.stdin.close()
    status = "stopped" if child.returncode == 0 else "failed"
    while True:
        try:
            storage.write_json(process, {"status": status, "release_version": version})
            return
        except (OSError, storage.RecordRecoveryRequired):
            time.sleep(0.1)


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
            [str(executable(verified)), str(config)],
            stdin=subprocess.PIPE,
            stdout=log,
            stderr=log,
            creationflags=subprocess.CREATE_NO_WINDOW if os.name == "nt" else 0,
        )
        version = verified["release_version"]
        try:
            storage.write_json(process, {"status": "running", "release_version": version})
            result = wait_child(child, stop, process, version)
            storage.write_json(process, {"status": "stopped" if result == 0 else "failed", "release_version": version})
            return result
        finally:
            finish_child(child, process, version)


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
