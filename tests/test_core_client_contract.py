# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Contract tests for the installed runtime-only C++ CLI.

These tests exercise argument parsing and the protocol-v1 boundary without a
vehicle. Direct vehicle commands remain in the non-installed qualification tool.
"""

from __future__ import annotations

import socket
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]

EXPECTED_VERBS = (
    "connect",
    "status",
    "admit",
    "revoke",
    "handback",
    "arm",
    "disarm",
    "mode",
    "takeoff",
    "vtol-takeoff",
    "transition-to-fixed-wing",
    "fixed-wing-route",
    "fixed-wing-recovery",
    "transition-to-vtol",
    "quadplane-vtol-land",
    "goto",
    "land",
    "rtl",
    "servo",
    "relay",
    "motor-test",
    "gimbal-config",
    "mission-demo",
    "velocity",
    "velocity-demo",
    "fence-demo",
    "payload-demo",
)

UNSUPPORTED_REQUESTS = (
    ("connect",),
    ("arm",),
    ("disarm",),
    ("mode", "4"),
    ("takeoff", "5"),
    ("vtol-takeoff", "5"),
    ("transition-to-fixed-wing",),
    ("fixed-wing-route", "45", "-73", "10", "45.1", "-73.1", "10"),
    ("fixed-wing-recovery", "45", "-73", "10"),
    ("transition-to-vtol", "45", "-73", "10"),
    ("quadplane-vtol-land", "45", "-73"),
    ("goto", "45", "-73", "10"),
    ("land",),
    ("rtl",),
    ("mission-demo",),
    ("velocity", "--vx", "0.1", "--duration", "1"),
    ("velocity-demo",),
    ("fence-demo",),
    ("payload-demo", "3", "1.5"),
)

TYPED_REQUESTS = (
    ("status",),
    ("admit",),
    ("revoke",),
    ("handback",),
    ("servo", "1", "1500"),
    ("relay", "3", "1"),
    ("motor-test", "1", "1000", "1.0"),
    ("gimbal-config", "2"),
)


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
    reason="production C++ CLI not built; run `pixi run build-core` first",
)


def invoke(*arguments: str) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [str(BINARY), *arguments],
        capture_output=True,
        text=True,
        timeout=10,
        check=False,
    )


def free_tcp_port() -> int:
    """Return a loopback TCP port that is released before the client runs."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as probe:
        probe.bind(("127.0.0.1", 0))
        return int(probe.getsockname()[1])


def test_no_arguments_prints_usage_and_fails() -> None:
    result = invoke()

    assert result.returncode != 0
    assert "Usage: nomad <" in result.stdout
    assert "--direct" not in result.stdout
    assert "--endpoint" not in result.stdout
    assert "--system-id" not in result.stdout


def test_usage_lists_every_recognized_verb() -> None:
    result = invoke("not-a-command")

    assert result.returncode != 0
    missing = [verb for verb in EXPECTED_VERBS if verb not in result.stdout]
    assert not missing, f"usage omits recognized verbs: {missing}"


def test_unknown_command_prints_usage_and_fails() -> None:
    result = invoke("not-a-command")

    assert result.returncode != 0
    assert "Usage: nomad" in result.stdout


def test_removed_user_command_cannot_reach_runtime() -> None:
    result = invoke("user-command", "1", "2", "3", "4", "5", "6", "7")

    assert result.returncode != 0
    assert "Usage: nomad" in result.stdout
    assert "user-command" not in result.stdout


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
        ("payload-demo", "3", "1.5", "9"),
        ("servo", "1"),
        ("servo", "1", "1500", "9"),
        ("relay", "3"),
        ("relay", "16", "1"),
        ("relay", "3", "2"),
        ("motor-test", "1", "1000"),
        ("motor-test", "1", "banana", "1.0"),
        ("motor-test", "1", "1000", "1.0", "9"),
        ("gimbal-config", "x"),
        ("gimbal-config", "7"),
        ("--direct", "arm"),
        ("--runtime", "status"),
        ("--endpoint", "udpin:127.0.0.1:14550"),
        ("status", "--endpoint", "udpin:127.0.0.1:14550"),
        ("status", "--system-id", "1"),
    ],
)
def test_malformed_arguments_fail_fast_with_usage(arguments: tuple[str, ...]) -> None:
    result = invoke(*arguments)

    assert result.returncode != 0
    assert "Usage: nomad" in result.stdout


@pytest.mark.parametrize("arguments", TYPED_REQUESTS)
def test_typed_commands_use_runtime_ipc(monkeypatch, arguments: tuple[str, ...]) -> None:
    monkeypatch.setenv("NOMAD_RUNTIME_IPC_PORT", str(free_tcp_port()))

    result = invoke(*arguments)

    assert result.returncode != 0
    assert "error[runtime_unavailable]" in result.stderr
    assert "MAVSDK" not in result.stderr
    assert "heartbeat" not in result.stderr


@pytest.mark.parametrize("arguments", UNSUPPORTED_REQUESTS)
def test_unsupported_commands_report_unavailable_without_transport_fallback(
    monkeypatch, arguments: tuple[str, ...]
) -> None:
    monkeypatch.setenv("NOMAD_RUNTIME_IPC_PORT", "invalid")

    result = invoke(*arguments)

    assert result.returncode != 0
    assert "error[unsupported_request]" in result.stderr
    assert f"{arguments[0]} is not available through runtime protocol v1" in result.stderr
    assert "invalid_configuration" not in result.stderr
    assert "heartbeat" not in result.stderr
