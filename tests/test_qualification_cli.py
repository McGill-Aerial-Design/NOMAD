# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Safety boundary tests for the non-installed direct qualification driver."""

from __future__ import annotations

import socket
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]


def find_binary() -> Path | None:
    names = ("nomad-qualification.exe", "nomad-qualification")
    build_dirs = (ROOT / "build" / "core", ROOT / "build" / "mavsdk-phase-a")
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
    reason="qualification driver not built; run `pixi run build-qualification-cli` first",
)


def free_tcp_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as probe:
        probe.bind(("127.0.0.1", 0))
        return int(probe.getsockname()[1])


def invoke(*arguments: str) -> subprocess.CompletedProcess[str]:
    return subprocess.run([str(BINARY), *arguments], capture_output=True, text=True, timeout=10, check=False)


def test_direct_actuation_refused_without_key_before_transport(monkeypatch) -> None:
    monkeypatch.delenv("NOMAD_API_KEY", raising=False)
    monkeypatch.delenv("NOMAD_INTEGRATED_FLIGHT", raising=False)
    monkeypatch.setenv("NOMAD_RUNTIME_IPC_PORT", str(free_tcp_port()))

    result = invoke("arm", "--endpoint", "invalid")

    assert result.returncode != 0
    assert "audit command=arm result=refused auth=none reason=missing_api_key" in result.stderr
    assert "heartbeat" not in result.stderr


def test_direct_actuation_with_key_is_audited(monkeypatch) -> None:
    monkeypatch.setenv("NOMAD_API_KEY", "qualification-key")
    monkeypatch.delenv("NOMAD_INTEGRATED_FLIGHT", raising=False)
    monkeypatch.setenv("NOMAD_RUNTIME_IPC_PORT", str(free_tcp_port()))

    result = invoke("arm", "--endpoint", "invalid")

    assert result.returncode != 0
    assert "audit command=arm result=accepted auth=api-key" in result.stderr


def test_integrated_flight_blocks_direct_actuation(monkeypatch) -> None:
    monkeypatch.setenv("NOMAD_API_KEY", "qualification-key")
    monkeypatch.setenv("NOMAD_INTEGRATED_FLIGHT", "1")
    monkeypatch.setenv("NOMAD_RUNTIME_IPC_PORT", str(free_tcp_port()))

    result = invoke("arm", "--endpoint", "invalid")

    assert result.returncode != 0
    assert "runtime_owner_required" in result.stderr
    assert "heartbeat" not in result.stderr


@pytest.mark.parametrize("value", ["true", "TRUE", "yes", "1"])
def test_integrated_flight_boolean_spellings_block_direct_actuation(monkeypatch, value: str) -> None:
    monkeypatch.setenv("NOMAD_API_KEY", "qualification-key")
    monkeypatch.setenv("NOMAD_INTEGRATED_FLIGHT", value)
    monkeypatch.setenv("NOMAD_RUNTIME_IPC_PORT", str(free_tcp_port()))

    result = invoke("arm", "--endpoint", "invalid")

    assert result.returncode != 0
    assert "runtime_owner_required" in result.stderr
    assert "heartbeat" not in result.stderr


def test_invalid_integrated_flight_value_fails_closed(monkeypatch) -> None:
    monkeypatch.setenv("NOMAD_API_KEY", "qualification-key")
    monkeypatch.setenv("NOMAD_INTEGRATED_FLIGHT", "sometimes")
    monkeypatch.setenv("NOMAD_RUNTIME_IPC_PORT", str(free_tcp_port()))

    result = invoke("arm", "--endpoint", "invalid")

    assert result.returncode != 0
    assert "invalid_integrated_flight_setting" in result.stderr
    assert "NOMAD_INTEGRATED_FLIGHT must be a boolean value" in result.stderr
    assert "heartbeat" not in result.stderr


def test_runtime_listener_blocks_direct_actuation(monkeypatch) -> None:
    monkeypatch.setenv("NOMAD_API_KEY", "qualification-key")
    monkeypatch.delenv("NOMAD_INTEGRATED_FLIGHT", raising=False)
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as listener:
        listener.bind(("127.0.0.1", 0))
        listener.listen()
        monkeypatch.setenv("NOMAD_RUNTIME_IPC_PORT", str(listener.getsockname()[1]))

        result = invoke("arm", "--endpoint", "invalid")

    assert result.returncode != 0
    assert "runtime_owner_active" in result.stderr
    assert "heartbeat" not in result.stderr
