# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Client contract tests for the C++ core CLI boundary.

The C++ ``nomad`` CLI is the replacement client boundary the transitional
Python vehicle path will hand over to. These tests pin the parts of that
boundary that are observable without a vehicle or SITL:

- the exact verb surface named by ``usage`` output;
- argument parsing: malformed values fail fast, before any socket work;
- SR-SEC-02/03 authentication: actuation verbs are refused without
  ``NOMAD_API_KEY`` before any socket work, accepted attempts carry an audit
  line, and telemetry verbs keep the no-key local fallback;
- no-vehicle failure behavior: an occupied endpoint and a silent endpoint
  each fail with a documented, deterministic diagnostic and a nonzero exit.

Command flows against a real vehicle stay in the ``core-sitl-*`` scenarios;
this file deliberately opens no MAVLink link to a live peer.

Skipped when the C++ binary has not been built (``pixi run build-core``).
"""

from __future__ import annotations

import socket
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]

# The usage line is the client-facing verb surface. Every verb below must
# appear there so a wrapper can trust the help text.
EXPECTED_VERBS = (
    "connect",
    "status",
    "arm",
    "disarm",
    "mode",
    "takeoff",
    "goto",
    "land",
    "rtl",
    "servo",
    "relay",
    "motor-test",
    "gimbal-config",
    "user-command",
    "mission-demo",
    "velocity-demo",
    "fence-demo",
    "payload-demo",
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
    reason="C++ core binary not built; run `pixi run build-core` first",
)


def invoke(*arguments: str) -> subprocess.CompletedProcess:
    return subprocess.run(
        [str(BINARY), *arguments],
        capture_output=True,
        text=True,
        timeout=30,
        check=False,
    )


def free_udp_port() -> int:
    """Return a currently unused UDP port (released before the caller binds)."""
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
        probe.bind(("0.0.0.0", 0))
        return int(probe.getsockname()[1])


def test_no_arguments_prints_usage_and_fails() -> None:
    result = invoke()

    assert result.returncode != 0
    assert "Usage: nomad" in result.stdout


def test_usage_lists_every_supported_verb() -> None:
    result = invoke("not-a-command")

    assert result.returncode != 0
    missing = [verb for verb in EXPECTED_VERBS if verb not in result.stdout]
    assert not missing, f"usage omits supported verbs: {missing}"


def test_unknown_command_prints_usage_and_fails() -> None:
    result = invoke("not-a-command")

    assert result.returncode != 0
    assert "Usage: nomad" in result.stdout


@pytest.mark.parametrize(
    "arguments",
    [
        ("takeoff", "banana"),  # altitude must parse as a float
        ("mode", "1x"),  # mode must parse as a decimal
        ("takeoff", "5", "9"),  # extra positional argument
        ("mode", "4", "extra"),  # extra positional argument
        ("goto", "45.0", "9.0"),  # goto requires latitude, longitude, and altitude
        ("goto", "45.0", "banana", "5"),  # longitude must parse as a float
        ("goto", "45.0", "9.0", "5", "7"),  # extra positional argument
        ("payload-demo", "16", "1.5"),  # relay number above the 0..15 bound
        ("payload-demo", "3", "1.5", "9"),  # extra positional argument
        ("servo", "1"),  # servo requires channel and pwm
        ("servo", "1", "1500", "9"),  # extra positional argument
        ("relay", "3"),  # relay requires number and on/off
        ("relay", "16", "1"),  # relay number above the 0..15 bound
        ("relay", "3", "2"),  # relay on/off must be 0 or 1
        ("motor-test", "1", "1000"),  # motor-test requires instance, pwm, timeout
        ("motor-test", "1", "banana", "1.0"),  # pwm must parse as a decimal
        ("motor-test", "1", "1000", "1.0", "9"),  # extra positional argument
        ("gimbal-config", "x"),  # mount mode must parse as a decimal
        ("gimbal-config", "7"),  # mount mode above the 0..4 bound
        ("user-command", "1"),  # user-command requires exactly seven values
        ("user-command", "1", "x", "3", "4", "5", "6", "7"),  # non-numeric parameter
        ("user-command", "1", "2", "3", "4", "5", "6", "7", "8"),  # more than seven values
        ("--endpoint", "udpin:0.0.0.0:14550"),  # flag without a command
    ],
)
def test_malformed_arguments_fail_fast_with_usage(arguments: tuple[str, ...]) -> None:
    result = invoke(*arguments)

    assert result.returncode != 0
    assert "Usage: nomad" in result.stdout


def test_occupied_endpoint_reports_connect_failure() -> None:
    # Occupy the same bind shape the CLI uses (wildcard address) so its
    # bind() fails deterministically on both Windows and POSIX.
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as holder:
        holder.bind(("0.0.0.0", 0))
        port = int(holder.getsockname()[1])

        result = invoke("connect", "--endpoint", f"udpin:0.0.0.0:{port}")

    assert result.returncode != 0
    assert "could not connect" in result.stderr


def test_silent_endpoint_times_out_cleanly() -> None:
    port = free_udp_port()

    result = invoke("connect", "--endpoint", f"udpin:0.0.0.0:{port}")

    assert result.returncode != 0
    assert "timed out waiting for ArduPilot heartbeat" in result.stderr


@pytest.mark.parametrize(
    "arguments",
    [
        ("arm",),
        ("disarm",),
        ("mode", "4"),
        ("takeoff", "5"),
        ("goto", "45.0", "9.0", "5"),
        ("land",),
        ("rtl",),
        ("servo", "1", "1500"),
        ("relay", "3", "1"),
        ("motor-test", "1", "1000", "1.0"),
        ("gimbal-config", "2"),
        ("user-command", "1", "2", "3", "4", "5", "6", "7"),
        ("mission-demo",),
        ("velocity-demo",),
        ("fence-demo",),
        ("payload-demo", "3", "1.5"),
    ],
)
def test_every_actuation_verb_refused_without_key_before_any_socket_work(
    monkeypatch, arguments: tuple[str, ...]
) -> None:
    monkeypatch.delenv("NOMAD_API_KEY", raising=False)

    result = invoke(*arguments)

    assert result.returncode != 0
    assert f"audit command={arguments[0]} result=refused auth=none reason=missing_api_key" in result.stderr
    assert "timed out waiting" not in result.stderr


def test_empty_api_key_is_treated_as_unset(monkeypatch) -> None:
    monkeypatch.setenv("NOMAD_API_KEY", "")

    result = invoke("arm")

    assert result.returncode != 0
    assert "result=refused" in result.stderr


def test_actuation_with_key_reaches_transport_and_audits(monkeypatch) -> None:
    monkeypatch.setenv("NOMAD_API_KEY", "nomad-dev-sitl-key")
    port = free_udp_port()

    result = invoke("arm", "--endpoint", f"udpin:0.0.0.0:{port}")

    assert result.returncode != 0
    assert "audit command=arm result=accepted auth=api-key" in result.stderr
    assert "timed out waiting for ArduPilot heartbeat" in result.stderr


def test_goto_is_an_actuation_verb_and_requires_the_key(monkeypatch) -> None:
    monkeypatch.delenv("NOMAD_API_KEY", raising=False)

    result = invoke("goto", "45.0", "9.0", "5")

    assert result.returncode != 0
    assert "audit command=goto result=refused auth=none reason=missing_api_key" in result.stderr
    assert "timed out waiting" not in result.stderr


def test_goto_with_key_reaches_transport_and_audits(monkeypatch) -> None:
    monkeypatch.setenv("NOMAD_API_KEY", "nomad-dev-sitl-key")
    port = free_udp_port()

    result = invoke("goto", "45.0", "9.0", "5", "--endpoint", f"udpin:0.0.0.0:{port}")

    assert result.returncode != 0
    assert "audit command=goto result=accepted auth=api-key" in result.stderr
    assert "timed out waiting for ArduPilot heartbeat" in result.stderr


def test_telemetry_verbs_keep_the_no_key_local_fallback(monkeypatch) -> None:
    monkeypatch.delenv("NOMAD_API_KEY", raising=False)
    port = free_udp_port()

    result = invoke("status", "--endpoint", f"udpin:0.0.0.0:{port}")

    assert result.returncode != 0
    assert "timed out waiting for ArduPilot heartbeat" in result.stderr
    assert "result=refused" not in result.stderr
