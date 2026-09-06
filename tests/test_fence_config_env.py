# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Contract test for the CLI's NOMAD-side fence gate (SR-FEN-02).

The projected keep-in fence must reject an out-of-fence ``goto`` target
before any socket work, and must not change the behavior of an unset
configuration. Skipped when the C++ binary has not been built.
"""

from __future__ import annotations

import socket
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]


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


BINARY = find_binary()

pytestmark = pytest.mark.skipif(
    BINARY is None,
    reason="C++ core binary not built; run `pixi run build-core` first",
)

# A ~222 m box around (45.0, -73.0) with a 5 m margin.
FENCE = "45.001,-73.001;45.001,-72.999;45.0,-72.999;45.0,-73.0"
OUTSIDE_TARGET = "45.5"


def invoke(monkeypatch, polygon: str | None, *arguments: str) -> subprocess.CompletedProcess:
    monkeypatch.setenv("NOMAD_API_KEY", "nomad-dev-sitl-key")
    if polygon is None:
        monkeypatch.delenv("NOMAD_FENCE_POLYGON", raising=False)
    else:
        monkeypatch.setenv("NOMAD_FENCE_POLYGON", polygon)
    monkeypatch.setenv("NOMAD_FENCE_MARGIN_M", "5.0")
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
        probe.bind(("0.0.0.0", 0))
        port = int(probe.getsockname()[1])
    return subprocess.run(
        [str(BINARY), *arguments, "--endpoint", f"udpin:0.0.0.0:{port}"],
        capture_output=True,
        text=True,
        timeout=30,
        check=False,
    )


def test_fenced_out_goto_is_refused_before_any_socket_work(monkeypatch) -> None:
    result = invoke(monkeypatch, FENCE, "goto", OUTSIDE_TARGET, "-73.0", "10")

    assert result.returncode != 0
    assert "outside the geofence" in result.stderr
    assert "heartbeat" not in result.stderr


def test_unconfigured_fence_keeps_the_normal_goto_path(monkeypatch) -> None:
    result = invoke(monkeypatch, None, "goto", OUTSIDE_TARGET, "-73.0", "10")

    assert result.returncode != 0
    assert "outside the geofence" not in result.stderr
    assert "timed out waiting for ArduPilot heartbeat" in result.stderr


def test_malformed_fence_fails_closed(monkeypatch) -> None:
    result = invoke(monkeypatch, "garbage", "goto", "45.0005", "-73.0", "10")

    assert result.returncode != 0
    assert "geofence" in result.stderr
    assert "heartbeat" not in result.stderr
