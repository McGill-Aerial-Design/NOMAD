# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Exercise the persistent runtime against the local deterministic MAVLink peer."""

from __future__ import annotations

import hashlib
import hmac
import json
import os
import secrets
import signal
import socket
import subprocess
import sys
import tempfile
import time
from pathlib import Path
from typing import Any

from mavsdk_peer import COMMAND_DO_MOUNT_CONTROL, COMMAND_DO_SET_SERVO, VehiclePeer

ROOT = Path(__file__).resolve().parents[2]
FIXTURE_CREDENTIALS = {
    client: secrets.token_hex(32)
    for client in (
        "runtime-smoke",
        "nomad-cli",
        "runtime-gimbal-smoke",
        "operator",
    )
}


def find_runtime() -> Path:
    """Find the CMake-built runtime on the current platform."""
    if os.environ.get("NOMAD_RESOURCE_BUILD_DIR"):
        from resource_footprint import release_binary

        return release_binary(Path(os.environ["NOMAD_RESOURCE_BUILD_DIR"]), "nomad-runtime")
    names = ("nomad-runtime.exe", "nomad-runtime")
    for directory in (ROOT / "build" / "core" / "Debug", ROOT / "build" / "core" / "Release", ROOT / "build" / "core"):
        for name in names:
            candidate = directory / name
            if candidate.is_file():
                return candidate
    raise FileNotFoundError("build nomad-runtime first with `pixi run build-core`")


def find_cli() -> Path:
    """Find the installed-behavior C++ CLI that sends requests to the runtime."""
    if os.environ.get("NOMAD_RESOURCE_BUILD_DIR"):
        from resource_footprint import release_binary

        return release_binary(Path(os.environ["NOMAD_RESOURCE_BUILD_DIR"]), "nomad")
    names = ("nomad.exe", "nomad")
    for directory in (ROOT / "build" / "core" / "Debug", ROOT / "build" / "core" / "Release", ROOT / "build" / "core"):
        for name in names:
            candidate = directory / name
            if candidate.is_file():
                return candidate
    raise FileNotFoundError("build nomad first with `pixi run build-core`")


def free_port(protocol: int) -> int:
    """Reserve and return an unused loopback port number."""
    with socket.socket(socket.AF_INET, protocol) as probe:
        probe.bind(("127.0.0.1", 0))
        return int(probe.getsockname()[1])


def wait_for_listener(port: int, deadline_seconds: float = 10.0) -> None:
    """Wait until the local IPC listener accepts a connection."""
    deadline = time.monotonic() + deadline_seconds
    while time.monotonic() < deadline:
        try:
            with socket.create_connection(("127.0.0.1", port), timeout=0.2):
                return
        except OSError:
            time.sleep(0.05)
    raise TimeoutError(f"runtime did not listen on 127.0.0.1:{port}")


def send_request(port: int, message: dict[str, Any]) -> dict[str, Any]:
    """Send one bounded JSON Lines request and read its response."""
    message = dict(message)
    secret = message.pop("credential", "")
    if secret:
        unsigned = json.dumps(message, separators=(",", ":"))
        message["auth_payload"] = unsigned
        message["auth_proof"] = hmac.new(
            secret.encode(), ("nomad-core:request:v1:" + unsigned).encode(), hashlib.sha256
        ).hexdigest()
    payload = json.dumps(message, separators=(",", ":")).encode("utf-8") + b"\n"
    if len(payload) > 65537:
        raise ValueError("smoke request exceeds protocol limit")
    with socket.create_connection(("127.0.0.1", port), timeout=2.0) as client:
        client.settimeout(8.0)
        client.sendall(payload)
        received = bytearray()
        while len(received) <= 65536:
            byte = client.recv(1)
            if not byte:
                raise ConnectionError("runtime closed before completing its response")
            if byte == b"\n":
                response = json.loads(received)
                if not isinstance(response, dict):
                    raise ValueError("runtime response must be a JSON object")
                return response
            received.extend(byte)
    raise ValueError("runtime response exceeded the protocol limit")


def request(
    port: int, request_id: str, request_type: str, client_id: str = "runtime-smoke", **fields: object
) -> dict[str, Any]:
    """Send a protocol-v1 request with a stable client identity."""
    return send_request(
        port,
        {
            "protocol": "nomad-core",
            "version": 1,
            "client_id": client_id,
            "id": request_id,
            "type": request_type,
            "credential": FIXTURE_CREDENTIALS.get(client_id, ""),
            "command_source": client_id,
            **fields,
        },
    )


def write_fixture_credentials(directory: str) -> Path:
    """Create a private credential file accepted by the production loader."""
    credential_file = Path(directory) / "credentials.json"
    descriptor = os.open(credential_file, os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o600)
    with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
        json.dump(FIXTURE_CREDENTIALS, stream)
    if os.name == "nt":
        subprocess.run(["icacls", str(credential_file), "/setowner", os.getlogin()], capture_output=True, check=True)
        subprocess.run(
            [
                "icacls",
                str(credential_file),
                "/inheritance:r",
                "/grant:r",
                f"{os.getlogin()}:(F)",
                "*S-1-5-18:(F)",
                "*S-1-5-32-544:(F)",
            ],
            capture_output=True,
            check=True,
        )
    return credential_file


def start_runtime(binary: Path, udp_port: int, ipc_port: int) -> tuple[subprocess.Popen[bytes], Any, Any]:
    """Start a runtime attached to the local fake vehicle."""
    environment = os.environ.copy()
    environment["NOMAD_API_KEY"] = "runtime-smoke-key"
    storage = tempfile.TemporaryDirectory(prefix="nomad-runtime-test-")
    credential_file = write_fixture_credentials(storage.name)
    environment["NOMAD_CLIENT_CREDENTIALS_FILE"] = str(credential_file)
    environment["NOMAD_AUDIT_DIRECTORY"] = str(Path(storage.name) / "audit")
    environment["NOMAD_MAVLINK_ENDPOINT"] = f"udpin:127.0.0.1:{udp_port}"
    environment["NOMAD_RUNTIME_IPC_PORT"] = str(ipc_port)
    environment["NOMAD_CLIENT_CREDENTIAL"] = FIXTURE_CREDENTIALS["nomad-cli"]
    stdout = tempfile.TemporaryFile()
    stderr = tempfile.TemporaryFile()
    options: dict[str, object] = {}
    if os.name == "nt":
        options["creationflags"] = subprocess.CREATE_NEW_PROCESS_GROUP
    process = subprocess.Popen(
        [str(binary), "--endpoint", environment["NOMAD_MAVLINK_ENDPOINT"], "--ipc-port", str(ipc_port)],
        env=environment,
        stdout=stdout,
        stderr=stderr,
        **options,
    )
    process._nomad_test_storage = storage
    return process, stdout, stderr


def stop_runtime(process: subprocess.Popen[bytes]) -> None:
    """Request orderly shutdown and wait for owned sockets to close."""
    if process.poll() is not None:
        return
    if os.name == "nt":
        process.send_signal(signal.CTRL_BREAK_EVENT)
    else:
        process.send_signal(signal.SIGINT)
    process.wait(timeout=12)


def read_logs(stdout: Any, stderr: Any) -> str:
    """Read captured process output for an actionable failure report."""
    stdout.seek(0)
    stderr.seek(0)
    return (stdout.read() + stderr.read()).decode("utf-8", errors="replace")


def require(condition: bool, description: str) -> None:
    """Fail with the condition name so CI output identifies the broken step."""
    if not condition:
        raise AssertionError(description)
    print(f"[OK] {description}")


def verify_invalid_runtime_endpoint(binary: Path) -> None:
    """Reject unsupported aircraft transport before starting the runtime."""
    environment = os.environ.copy()
    environment["NOMAD_MAVLINK_ENDPOINT"] = "tcp:127.0.0.1:5760"
    environment["NOMAD_RUNTIME_IPC_PORT"] = str(free_port(socket.SOCK_STREAM))
    result = subprocess.run([str(binary)], capture_output=True, text=True, timeout=5, check=False, env=environment)
    require(
        result.returncode != 0 and "runtime configuration error: NOMAD_MAVLINK_ENDPOINT" in result.stderr,
        "runtime rejects unsupported aircraft endpoint configuration before listening",
    )


def wait_for_commands(peer: VehiclePeer, count: int) -> list[tuple[str, int, int | None, tuple[float, ...]]]:
    """Wait until the MAVLink peer has observed the expected command count."""
    deadline = time.monotonic() + 8.0
    while time.monotonic() < deadline:
        commands = [command for command in peer.commands() if command[1] == COMMAND_DO_SET_SERVO]
        if len(commands) >= count:
            return commands
        time.sleep(0.05)
    raise TimeoutError(f"fake vehicle saw {len(peer.commands())} commands, expected {count}")


def wait_for_mount_control(peer: VehiclePeer) -> tuple[str, int, int | None, tuple[float, ...]]:
    """Wait for the MAVSDK transport to send one mount angle target."""
    deadline = time.monotonic() + 8.0
    while time.monotonic() < deadline:
        for command in peer.commands():
            if command[1] == COMMAND_DO_MOUNT_CONTROL:
                return command
        time.sleep(0.05)
    raise TimeoutError("fake vehicle did not receive DO_MOUNT_CONTROL")


def authority_fields(hello: dict[str, Any], source: str) -> dict[str, object]:
    """Bind one request to the current runtime and vehicle generation."""
    authority = hello["authority"]
    return {
        "runtime_incarnation": hello["runtime_incarnation"],
        "vehicle_session": authority["vehicle_session"],
        "authority_generation": authority["generation"],
        "command_source": source,
        "sequence": authority["next_sequence"],
        "expires_at_ms": int(time.time() * 1000) + 3000,
    }


def run_cli(binary: Path, environment: dict[str, str], *arguments: str) -> subprocess.CompletedProcess[str]:
    """Run one C++ CLI request against the already-started runtime."""
    return subprocess.run(
        [str(binary), *arguments],
        capture_output=True,
        text=True,
        timeout=15,
        check=False,
        env=environment,
    )


def verify_hello_and_status(ipc_port: int) -> None:
    """Verify protocol negotiation and status readiness through fresh clients."""
    hello = request(ipc_port, "hello-1", "hello")
    require(hello["ok"] and hello["version"] == 1, "runtime HELLO negotiates protocol v1")
    require("set_servo" in hello["capabilities"], "HELLO lists typed output capability")

    status = request(ipc_port, "status-1", "status")["status"]
    require(status["runtime_ready"] is True, "STATUS reports runtime IPC readiness")
    deadline = time.monotonic() + 10.0
    while time.monotonic() < deadline and not status["identity_resolved"]:
        time.sleep(0.1)
        status = request(ipc_port, "status-wait", "status")["status"]
    require(status["vehicle_connected"] is True, "STATUS reports the fake vehicle session")
    require(status["identity_resolved"] is True, "STATUS reports resolved Copter identity")


def verify_cli_servo(binary: Path, peer: VehiclePeer, ipc_port: int) -> dict[str, str]:
    """Prove the CLI sends one typed command and closes its client connection."""
    environment = os.environ.copy()
    environment["NOMAD_RUNTIME_IPC_PORT"] = str(ipc_port)
    environment["NOMAD_CLIENT_CREDENTIAL"] = FIXTURE_CREDENTIALS["nomad-cli"]
    admitted = run_cli(binary, environment, "admit")
    require(admitted.returncode == 0, "CLI explicitly admits its software source")
    result = run_cli(binary, environment, "servo", "8", "1500")
    require(result.returncode == 0, "C++ CLI dispatches the typed servo request through runtime IPC")
    commands = wait_for_commands(peer, 1)
    require(len(commands) == 1, "fake MAVLink peer observes one SET_SERVO action")
    return environment


def verify_navigation_rejected(binary: Path, peer: VehiclePeer, environment: dict[str, str]) -> None:
    """Ensure the installed CLI cannot issue navigation commands outside v1."""
    result = run_cli(binary, environment, "goto", "45.5", "-73.5", "10")
    require(
        result.returncode != 0 and "unsupported_request" in result.stderr,
        "C++ CLI rejects navigation outside protocol v1",
    )
    require(len(wait_for_commands(peer, 1)) == 1, "rejected navigation produces no MAVLink command")


def verify_runtime_reconnect(ipc_port: int, peer: VehiclePeer, binary: Path, environment: dict[str, str]) -> None:
    """Verify a disconnected command client leaves runtime service available."""
    reconnected = request(ipc_port, "status-after-disconnect", "status")["status"]
    require(reconnected["runtime_ready"] is True, "runtime remains ready after the client disconnects")
    denied = request(
        ipc_port,
        "servo-2",
        "set_servo",
        channel=8,
        pwm_microseconds=1600,
    )
    require(denied["error"]["code"] == "stale_authority", "unbound client cannot mutate after reconnect")
    second = run_cli(binary, environment, "servo", "8", "1600")
    require(second.returncode == 0, "admitted CLI source can issue another fresh request")
    commands = wait_for_commands(peer, 2)
    require(len(commands) == 2, "two client requests produce exactly two SET_SERVO actions")


def verify_gimbal_target(ipc_port: int, peer: VehiclePeer) -> None:
    """Prove a typed runtime angle request reaches MAVSDK with fixed semantics."""
    source = "runtime-gimbal-smoke"
    hello = request(ipc_port, "gimbal-source-hello", "hello", client_id=source)
    revoked = request(
        ipc_port,
        "gimbal-revoke-cli-source",
        "revoke_authority",
        client_id=source,
        **authority_fields(hello, source),
    )
    require(revoked["ok"] is True, "runtime revokes the prior smoke-test owner before handback")

    hello = request(ipc_port, "gimbal-handback-hello", "hello", client_id=source)
    admitted = request(
        ipc_port,
        "gimbal-handback",
        "handback_authority",
        client_id=source,
        **authority_fields(hello, source),
    )
    require(admitted["ok"] is True, "runtime explicitly hands authority to the gimbal smoke source")

    hello = request(ipc_port, "gimbal-target-hello", "hello", client_id=source)
    target = request(
        ipc_port,
        "gimbal-target",
        "set_gimbal_target",
        client_id=source,
        pitch_deg=12.5,
        roll_deg=-7.5,
        **authority_fields(hello, source),
    )
    require(target["command_result"]["success"] is True, "typed gimbal target is acknowledged by the runtime")
    command = wait_for_mount_control(peer)
    require(
        command[0] == "COMMAND_LONG" and command[3] == (12.5, -7.5, 0.0, 0.0, 0.0, 0.0, 2.0),
        "MAVSDK sends fixed DO_MOUNT_CONTROL pitch, roll and targeting mode",
    )


def verify_runtime(binary: Path, peer: VehiclePeer, udp_port: int, ipc_port: int) -> None:
    """Check handshake, status, typed dispatch, reconnect and client isolation."""
    process, stdout, stderr = start_runtime(binary, udp_port, ipc_port)
    try:
        wait_for_listener(ipc_port)
        verify_hello_and_status(ipc_port)
        cli_environment = verify_cli_servo(find_cli(), peer, ipc_port)
        verify_navigation_rejected(find_cli(), peer, cli_environment)
        verify_runtime_reconnect(ipc_port, peer, find_cli(), cli_environment)
        verify_gimbal_target(ipc_port, peer)
    except Exception:
        if process.poll() is None:
            stop_runtime(process)
        print(read_logs(stdout, stderr), file=sys.stderr)
        raise
    stop_runtime(process)
    require(process.returncode == 0, f"runtime shuts down cleanly; {read_logs(stdout, stderr)}")
    stdout.close()
    stderr.close()


def verify_restart(binary: Path, udp_port: int, ipc_port: int) -> None:
    """Restart on the same endpoints and accept a fresh client session."""
    process, stdout, stderr = start_runtime(binary, udp_port, ipc_port)
    try:
        wait_for_listener(ipc_port)
        hello = request(ipc_port, "restart-hello", "hello")
        require(hello["ok"] is True, "runtime restart accepts a fresh IPC client")
    finally:
        stop_runtime(process)
    require(process.returncode == 0, f"restarted runtime shuts down cleanly; {read_logs(stdout, stderr)}")
    stdout.close()
    stderr.close()


def main() -> int:
    """Run the fake peer/runtime/client smoke with clean socket shutdown."""
    binary = find_runtime()
    udp_port = free_port(socket.SOCK_DGRAM)
    ipc_port = free_port(socket.SOCK_STREAM)
    peer = VehiclePeer(udp_port, 1)
    verify_invalid_runtime_endpoint(binary)
    peer.start()
    try:
        verify_runtime(binary, peer, udp_port, ipc_port)
        verify_restart(binary, udp_port, ipc_port)
    finally:
        peer.stop()
    print("runtime IPC smoke passed")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as error:
        print(f"runtime IPC smoke failed: {error}", file=sys.stderr)
        raise SystemExit(1)
