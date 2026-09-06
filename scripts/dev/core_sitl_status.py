# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Run the C++ telemetry smoke test against the Docker SITL UDP stream."""

from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def get_sitl_port() -> str:
    port = os.environ.get("NOMAD_CORE_SITL_PORT", "14570")
    if not port.isdecimal() or not 1 <= int(port) <= 65535:
        raise ValueError("NOMAD_CORE_SITL_PORT must be a UDP port from 1 to 65535")
    return port


def find_binary() -> Path | None:
    names = ("nomad.exe", "nomad")
    build_dirs = (ROOT / "build" / "core", ROOT / "build-core")
    configurations = tuple(
        directory for build_dir in build_dirs for directory in (build_dir, build_dir / "Debug", build_dir / "Release")
    )
    for directory in configurations:
        for name in names:
            candidate = directory / name
            if candidate.is_file():
                return candidate
    return None


def run_status(binary: Path, port: str) -> int:
    endpoint = f"udpin:0.0.0.0:{port}"
    print(f"Running {binary} status against {endpoint}", flush=True)
    result = subprocess.run([str(binary), "status", "--endpoint", endpoint], check=False)
    return result.returncode


def print_watch_hint() -> None:
    """Tell the operator how to watch this run live in Mission Planner.

    The dev stack publishes the SITL vehicle's operator MAVLink link on host
    TCP 5762 (docker-compose mp_bridge service). Mission Planner connects as
    a passive observer without disturbing the scenario: CONNECT -> TCP ->
    127.0.0.1:5762.
    """
    print(
        "Watch this run live: Mission Planner -> CONNECT -> TCP -> 127.0.0.1:5762 "
        "(the dev stack's SITL operator link; passive, does not disturb the test).",
        flush=True,
    )


def main() -> int:
    try:
        port = get_sitl_port()
    except ValueError as error:
        print(f"error: {error}", file=sys.stderr)
        return 2

    binary = find_binary()
    if binary is None:
        print("error: C++ core binary not found; run `pixi run build-core` first", file=sys.stderr)
        return 2
    print_watch_hint()
    return run_status(binary, port)


if __name__ == "__main__":
    raise SystemExit(main())
