# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Contract tests for the installed runtime-only C++ CLI.

These tests exercise argument parsing and the protocol-v1 boundary without a
vehicle. Other direct flight commands remain in the non-installed qualification tool.
"""

from __future__ import annotations

import hashlib
import hmac
import json
import socket
import subprocess
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]

EXPECTED_VERBS = ("status", "admit", "revoke", "handback", "land", "servo", "relay", "motor-test", "gimbal-config")

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
    ("land",),
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


@pytest.mark.parametrize("advertise_auth", [False, True])
def test_rogue_or_legacy_runtime_cannot_receive_cli_mutation(monkeypatch, advertise_auth: bool) -> None:
    secret = "a" * 64
    monkeypatch.setenv("NOMAD_CLIENT_CREDENTIAL", secret)
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as listener:
        listener.bind(("127.0.0.1", 0))
        listener.listen()
        listener.settimeout(5)
        monkeypatch.setenv("NOMAD_RUNTIME_IPC_PORT", str(listener.getsockname()[1]))

        def serve() -> bytes:
            with listener.accept()[0] as connection:
                connection.settimeout(5)
                hello = json.loads(connection.makefile("rb").readline())
                response = {
                    "protocol": "nomad-core",
                    "version": 1,
                    "id": hello["id"],
                    "ok": True,
                    "type": "hello_response",
                    "runtime_incarnation": "rogue",
                    "authority": {"vehicle_session": 1, "generation": 0, "next_sequence": 1},
                }
                if advertise_auth:
                    response.update(client_authentication="hmac-sha256-v1", server_proof="0" * 64)
                connection.sendall(json.dumps(response).encode() + b"\n")
                return connection.recv(65536)

        with ThreadPoolExecutor(max_workers=1) as executor:
            pending = executor.submit(serve)
            result = invoke("servo", "8", "1500")
            assert pending.result() == b"", "CLI sent a mutation to an unauthenticated runtime"
    assert result.returncode != 0
    assert "authentication_required" in result.stderr
    assert secret not in result.stdout + result.stderr


def authenticated_hello(hello: dict, secret: str, advertise_land: bool) -> dict:
    """Mirror only the authenticated server handshake used by the installed client."""
    incarnation = "test-land-runtime"
    payload = f"nomad-core:server:v1:{hello['client_id']}:{hello['auth_nonce']}:{incarnation}"
    return {
        "protocol": "nomad-core",
        "version": 1,
        "id": hello["id"],
        "ok": True,
        "type": "hello_response",
        "runtime_incarnation": incarnation,
        "authority": {"vehicle_session": 1, "generation": 1, "next_sequence": 1},
        "client_authentication": "hmac-sha256-v1",
        "server_proof": hmac.new(secret.encode(), payload.encode(), hashlib.sha256).hexdigest(),
        "capabilities": ["land"] if advertise_land else ["status"],
    }


def run_land_client(monkeypatch, capabilities, reply_fields: dict | None = None) -> tuple:
    """Exchange one CLI request with an authenticated test server and observe replay."""
    secret = "a" * 64
    monkeypatch.setenv("NOMAD_CLIENT_CREDENTIAL", secret)
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as listener:
        listener.bind(("127.0.0.1", 0))
        listener.listen()
        listener.settimeout(5)
        monkeypatch.setenv("NOMAD_RUNTIME_IPC_PORT", str(listener.getsockname()[1]))

        def serve() -> tuple[dict | None, bytes]:
            with listener.accept()[0] as connection:
                connection.settimeout(5)
                stream = connection.makefile("rb")
                hello = json.loads(stream.readline())
                response = authenticated_hello(hello, secret, advertise_land=True)
                response["capabilities"] = capabilities
                connection.sendall(json.dumps(response).encode() + b"\n")
                line = stream.readline()
                if not line:
                    return None, b""
                mutation = json.loads(line)
                if reply_fields is None:
                    return mutation, b""
                response = {"protocol": "nomad-core", "version": 1, "id": mutation["id"], **reply_fields}
                connection.sendall(json.dumps(response).encode() + b"\n")
                return mutation, stream.read(65536)

        with ThreadPoolExecutor(max_workers=1) as executor:
            pending = executor.submit(serve)
            result = invoke("land")
            mutation, replay = pending.result()
    return result, mutation, replay


@pytest.mark.parametrize("capabilities", [None, ["status"], "land", [1, "land"], [None, "land"]])
def test_cli_land_requires_advertised_capability(monkeypatch, capabilities) -> None:
    result, mutation, _replay = run_land_client(monkeypatch, capabilities)
    assert mutation is None, "CLI sent LAND without an advertised capability"
    assert result.returncode != 0
    assert "unsupported_request" in result.stderr


@pytest.mark.parametrize(
    "changes",
    [
        {"outcome": None},
        {"outcome": 1},
        {"outcome": "unknown"},
        {
            "ok": False,
            "outcome": "rejected",
            "error": {"code": "rejected", "message": "contradictory"},
            "command_result": {"success": False, "acknowledged": True},
        },
        {"command_result": {"success": True, "acknowledged": False, "message": "LAND observed"}},
        {"command_result": {"success": False, "acknowledged": True, "message": "LAND observed"}},
        {"command_result": {"success": True, "message": "LAND observed"}},
        {"command_result": {"success": True, "acknowledged": True, "message": 1}},
        {"type": "status_response"},
        {"ok": "true"},
        {"version": "1"},
        {"id": "wrong-request"},
    ],
)
def test_cli_land_rejects_incomplete_or_contradictory_result_without_replay(monkeypatch, changes: dict) -> None:
    fields = {
        "ok": True,
        "type": "command_response",
        "outcome": "success",
        "command_result": {"success": True, "acknowledged": True, "message": "LAND observed"},
        **changes,
    }
    result, request, replay = run_land_client(monkeypatch, ["land"], fields)
    assert request["type"] == "land"
    assert replay == b"", "CLI replayed LAND after a contradictory reply"
    assert result.returncode != 0
    assert "unknown_outcome" in result.stderr


@pytest.mark.parametrize("outcome, code", [("unknown", "audit_failure"), ("interrupted", "authority_interrupted")])
def test_cli_land_error_preserves_uncertainty_without_replay(monkeypatch, outcome: str, code: str) -> None:
    fields = {"ok": False, "outcome": outcome, "error": {"code": code, "message": "operation could not be verified"}}
    result, request, replay = run_land_client(monkeypatch, ["land"], fields)
    assert request["type"] == "land"
    assert replay == b"", "CLI replayed an uncertain LAND error"
    assert result.returncode != 0
    assert code in result.stderr
    assert "outcome is unknown" in result.stderr
    assert "must not be replayed" in result.stderr


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
        ("land", "9"),
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


def test_installed_client_ignores_aircraft_transport_environment(monkeypatch) -> None:
    monkeypatch.setenv("NOMAD_MAVLINK_ENDPOINT", "invalid-aircraft-endpoint")
    monkeypatch.setenv("NOMAD_RUNTIME_IPC_PORT", str(free_tcp_port()))

    result = invoke("status")

    assert result.returncode != 0
    assert "error[runtime_unavailable]" in result.stderr
    assert "invalid-aircraft-endpoint" not in result.stderr
    assert "MAVSDK" not in result.stderr
    assert "heartbeat" not in result.stderr


@pytest.mark.parametrize("arguments", UNSUPPORTED_REQUESTS)
def test_unsupported_commands_report_unavailable_without_transport_fallback(
    monkeypatch, arguments: tuple[str, ...]
) -> None:
    monkeypatch.setenv("NOMAD_RUNTIME_IPC_PORT", "invalid")

    result = invoke(*arguments)

    assert result.returncode != 0
    assert "Usage: nomad" in result.stdout
    assert "takeoff" not in result.stdout
    assert result.stderr == ""
    assert "invalid_configuration" not in result.stderr
    assert "heartbeat" not in result.stderr
