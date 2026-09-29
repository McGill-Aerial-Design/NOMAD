# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Primitives shared by the MAVSDK qualification fixtures.

Fixtures drive the non-installed `nomad-qualification` tool (through `run_cli`) and the
connection probe (`run_probe`, `run_staleness_probe`, `run_velocity_probe`) —
against `mavsdk_peer.VehiclePeer`, and both report the same way. Keeping that in
one module means a case only has to describe the behavior it proves.

The probe modes are the C++ contract binary's argv surface; see
tests/mavsdk_connection_test.cpp for what each one observes.
"""

from __future__ import annotations

import os
import socket
import subprocess
from pathlib import Path

from mavsdk_peer import ACCEPTED, CommandRecord, VehiclePeer
from mavsdk_qland_peer import with_qland_peer

ROOT = Path(__file__).resolve().parents[2]
BUILD_DIR = Path(os.environ.get("NOMAD_MAVSDK_FIXTURE_BUILD_DIR", ROOT / "build" / "mavsdk-phase-a"))


def find_binary(name: str) -> Path | None:
    names = (f"{name}.exe", name)
    for directory in (BUILD_DIR / "Release", BUILD_DIR / "Debug", BUILD_DIR):
        for candidate_name in names:
            candidate = directory / candidate_name
            if candidate.is_file():
                return candidate
    return None


def find_free_udp_port() -> int:
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    probe.bind(("127.0.0.1", 0))
    port = probe.getsockname()[1]
    probe.close()
    return port


def run_cli(binary: Path, port: int, *arguments: str, timeout: int = 25) -> subprocess.CompletedProcess:
    env = os.environ.copy()
    env.setdefault("NOMAD_API_KEY", "fixture-key")
    command = [str(binary), *arguments, "--endpoint", f"udpin:127.0.0.1:{port}"]
    return subprocess.run(command, capture_output=True, text=True, timeout=timeout, check=False, env=env)


def run_probe(
    binary: Path,
    port: int,
    system_id: int,
    command_id: int,
    wire_form: str,
    scheme: str = "udpin",
    timeout_ms: int = 2000,
) -> subprocess.CompletedProcess:
    command = [
        str(binary),
        "--probe",
        f"{scheme}:127.0.0.1:{port}",
        str(system_id),
        str(command_id),
        wire_form,
        str(timeout_ms),
    ]
    return subprocess.run(command, capture_output=True, text=True, timeout=25, check=False)


def run_staleness_probe(binary: Path, port: int, system_id: int, peer_lifetime: int) -> subprocess.CompletedProcess:
    command = [str(binary), "--staleness", f"udpin:127.0.0.1:{port}", str(system_id), str(peer_lifetime)]
    return subprocess.run(command, capture_output=True, text=True, timeout=peer_lifetime + 20, check=False)


def run_velocity_probe(binary: Path, port: int, system_id: int, setpoint: tuple[float, float, float, float]):
    command = [
        str(binary),
        "--velocity",
        f"udpin:127.0.0.1:{port}",
        str(system_id),
        *(str(value) for value in setpoint),
    ]
    return subprocess.run(command, capture_output=True, text=True, timeout=25, check=False)


def run_zero_delivery_probe(binary: Path, port: int, system_id: int, scenario: str) -> subprocess.CompletedProcess:
    command = [str(binary), "--zero-delivery", f"udpin:127.0.0.1:{port}", str(system_id), scenario]
    return subprocess.run(command, capture_output=True, text=True, timeout=45, check=False)


def run_param_probe(binary: Path, port: int, system_id: int, param_id: str, timeout_ms: int):
    command = [
        str(binary),
        "--param",
        f"udpin:127.0.0.1:{port}",
        str(system_id),
        param_id,
        str(timeout_ms),
    ]
    return subprocess.run(command, capture_output=True, text=True, timeout=25, check=False)


def run_fence_probe(binary: Path, port: int, system_id: int, enabled: bool) -> subprocess.CompletedProcess:
    command = [
        str(binary),
        "--fence",
        f"udpin:127.0.0.1:{port}",
        str(system_id),
        "enabled" if enabled else "disabled",
    ]
    return subprocess.run(command, capture_output=True, text=True, timeout=60, check=False)


def with_peer(port: int, system_id: int, ack_result: int | None, action, **peer_options):
    peer = VehiclePeer(port, system_id, ack_result, **peer_options)
    peer.start()
    try:
        return action(peer)
    finally:
        peer.stop()


def require(condition: bool, name: str, detail: str) -> None:
    if condition:
        print(f"[OK] {name}")
        return
    raise RuntimeError(f"{name} failed: {detail}")


def describe(result: subprocess.CompletedProcess, observed: list[CommandRecord]) -> str:
    return f"rc={result.returncode} stdout={result.stdout!r} stderr={result.stderr!r} commands={observed}"


def run_cli_case(
    cli: Path, *arguments: str, ack_result: int | None = ACCEPTED, timeout: int = 25, **peer_options
) -> tuple[subprocess.CompletedProcess, list[CommandRecord]]:
    """Run one CLI command against a fresh peer and return its observed commands."""
    port = find_free_udp_port()
    observed: list[CommandRecord] = []

    def action(peer: VehiclePeer) -> subprocess.CompletedProcess:
        result = run_cli(cli, port, *arguments, timeout=timeout)
        observed.extend(peer.commands())
        return result

    return with_peer(port, 1, ack_result, action, **peer_options), observed


def run_probe_case(
    probe: Path,
    command_id: int,
    wire_form: str,
    expected_system_id: int = 1,
    peer_system_id: int = 1,
    ack_result: int | None = ACCEPTED,
    timeout_ms: int = 2000,
) -> tuple[subprocess.CompletedProcess, list[CommandRecord]]:
    """Run one connection probe against a fresh peer and return its observed commands."""
    port = find_free_udp_port()
    observed: list[CommandRecord] = []

    def action(peer: VehiclePeer) -> subprocess.CompletedProcess:
        result = run_probe(probe, port, expected_system_id, command_id, wire_form, timeout_ms=timeout_ms)
        observed.extend(peer.commands())
        return result

    return with_peer(port, peer_system_id, ack_result, action), observed


def run_qland_probe(
    probe: Path, qland_ack_result: int = ACCEPTED
) -> tuple[subprocess.CompletedProcess, list[CommandRecord], list[tuple[int, int, int]]]:
    """Verify pinned version decoding and the fixed QLAND COMMAND_LONG wire request."""
    port = find_free_udp_port()
    observed: list[CommandRecord] = []
    targets: list[tuple[int, int, int]] = []

    def action(peer: VehiclePeer) -> subprocess.CompletedProcess:
        result = subprocess.run(
            [str(probe), "--qland", f"udpin:127.0.0.1:{port}", "1"],
            capture_output=True,
            text=True,
            timeout=15,
            check=False,
        )
        observed.extend(peer.commands())
        targets.extend(peer.command_targets)
        return result

    result = with_qland_peer(port, qland_ack_result, action)
    return result, observed, targets
