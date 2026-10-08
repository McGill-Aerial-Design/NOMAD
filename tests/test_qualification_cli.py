# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Safety boundary tests for the non-installed direct qualification driver."""

from __future__ import annotations

import os
import socket
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]


def find_binary() -> Path | None:
    names = ("nomad-qualification.exe", "nomad-qualification")
    build_dirs = (ROOT / "build" / "core", ROOT / "build" / "mavsdk-qualification")
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


@pytest.mark.parametrize(
    "arguments",
    [
        ("takeoff", "banana"),
        ("takeoff", "nan"),
        ("takeoff", "inf"),
        ("mode", "1x"),
        ("goto", "nan", "9.0", "5"),
        ("goto", "45.0", "inf", "5"),
        ("goto", "45.0", "9.0", "nan"),
        ("velocity", "--vx", "nan", "--duration", "1"),
        ("velocity", "--vx", "1", "--duration", "inf"),
        ("takeoff", "5", "9"),
        ("vtol-takeoff", "banana"),
        ("vtol-takeoff", "nan"),
        ("vtol-takeoff", "5", "9"),
        ("transition-to-vtol", "45.0", "-73.0"),
        ("transition-to-vtol", "nan", "-73.0", "20"),
        ("transition-to-vtol", "45.0", "-73.0", "inf"),
        ("transition-to-vtol", "45.0", "-73.0", "20", "30"),
        ("mode", "4", "extra"),
        ("goto", "45.0", "-73.0"),
        ("goto", "45.0", "banana", "5"),
        ("goto", "45.0", "9.0", "5", "7"),
        ("payload-demo", "16", "1.5"),
        ("payload-demo",),
        ("payload-demo", "3"),
        ("payload-demo", "3", "1.5", "9"),
        ("fixed-wing-route", "45", "-73", "10", "45.1", "-73.1"),
        ("fixed-wing-route", "nan", "-73", "10", "45.1", "-73.1", "10"),
        ("fixed-wing-recovery", "45", "-73", "inf"),
        ("quadplane-vtol-land", "45", "-73", "extra"),
    ],
)
def test_malformed_direct_arguments_fail_before_admission(monkeypatch, arguments: tuple[str, ...]) -> None:
    monkeypatch.setenv("NOMAD_RUNTIME_IPC_PORT", "invalid")
    result = invoke(*arguments)
    assert result.returncode != 0
    assert "Usage: nomad-qualification" in result.stdout
    assert "audit command=" not in result.stderr
    assert "heartbeat" not in result.stderr


@pytest.mark.parametrize(
    "port",
    [
        pytest.param(
            "", marks=pytest.mark.skipif(os.name == "nt", reason="Windows CRT treats empty environment values as unset")
        ),
        "0",
        "65536",
        "invalid",
        "99999999999999999999",
    ],
)
def test_invalid_owner_probe_port_inhibits_direct_commands(monkeypatch, port: str) -> None:
    monkeypatch.setenv("NOMAD_RUNTIME_IPC_PORT", port)
    monkeypatch.setenv("NOMAD_API_KEY", "qualification-key")
    result = invoke("arm", "--endpoint", "invalid")
    assert result.returncode != 0
    assert "runtime_owner_active" in result.stderr
    assert "heartbeat" not in result.stderr


def test_bound_owner_port_inhibits_before_listening(monkeypatch) -> None:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as owner:
        owner.bind(("127.0.0.1", 0))
        monkeypatch.setenv("NOMAD_RUNTIME_IPC_PORT", str(owner.getsockname()[1]))
        result = invoke("arm", "--endpoint", "invalid")
    assert result.returncode != 0
    assert "runtime_owner_active" in result.stderr
    assert "heartbeat" not in result.stderr
