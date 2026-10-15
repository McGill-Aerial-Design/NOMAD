# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Private runtime IPC, authentication and process primitives for qualification fixtures."""

from __future__ import annotations

import hashlib
import hmac
import json
import os
import secrets
import signal
import socket
import subprocess
import tempfile
import time
from pathlib import Path
from typing import Any

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
REQUEST_TIMING_FIELDS = (
    "prepare_begin_ns",
    "prepare_complete_ns",
    "connect_begin_ns",
    "connect_complete_ns",
    "send_begin_ns",
    "send_complete_ns",
    "read_begin_ns",
    "first_byte_ns",
    "read_complete_ns",
    "response_ready_ns",
    "error_ns",
    "finished_ns",
)


def find_runtime() -> Path:
    """Find the CMake-built runtime on the current platform."""
    if os.environ.get("NOMAD_QUALIFICATION_BUILD_DIR"):
        from mavsdk_build_metrics import release_binary

        return release_binary(Path(os.environ["NOMAD_QUALIFICATION_BUILD_DIR"]), "nomad-runtime")
    names = ("nomad-runtime.exe", "nomad-runtime")
    for directory in (ROOT / "build" / "core" / "Debug", ROOT / "build" / "core" / "Release", ROOT / "build" / "core"):
        for name in names:
            candidate = directory / name
            if candidate.is_file():
                return candidate
    raise FileNotFoundError("build nomad-runtime first with `pixi run build-core`")


def find_cli() -> Path:
    """Find the installed-behavior C++ CLI that sends requests to the runtime."""
    if os.environ.get("NOMAD_QUALIFICATION_BUILD_DIR"):
        from mavsdk_build_metrics import release_binary

        return release_binary(Path(os.environ["NOMAD_QUALIFICATION_BUILD_DIR"]), "nomad")
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


def send_request(port: int, message: dict[str, Any], *, timing: dict[str, int | None] | None = None) -> dict[str, Any]:
    """Send one bounded JSON Lines request and read its response."""
    if timing is not None:
        timing.clear()
        timing.update(dict.fromkeys(REQUEST_TIMING_FIELDS))

    def mark(name: str) -> None:
        if timing is not None:
            timing[name] = time.monotonic_ns()

    mark("prepare_begin_ns")
    try:
        message = dict(message)
        secret = message.pop("credential", "")
        if secret:
            unsigned = json.dumps(message, separators=(",", ":"))
            message["auth_payload"] = unsigned
            message["auth_proof"] = hmac.new(
                secret.encode(), ("nomad-core:request:v1:" + unsigned).encode(), hashlib.sha256
            ).hexdigest()
        payload = json.dumps(message, separators=(",", ":")).encode("utf-8") + b"\n"
        mark("prepare_complete_ns")
        if len(payload) > 65537:
            raise ValueError("smoke request exceeds protocol limit")
        mark("connect_begin_ns")
        with socket.create_connection(("127.0.0.1", port), timeout=2.0) as client:
            mark("connect_complete_ns")
            client.settimeout(8.0)
            mark("send_begin_ns")
            client.sendall(payload)
            mark("send_complete_ns")
            received = bytearray()
            mark("read_begin_ns")
            while len(received) <= 65536:
                byte = client.recv(1)
                if not byte:
                    raise ConnectionError("runtime closed before completing its response")
                if not received:
                    mark("first_byte_ns")
                if byte == b"\n":
                    mark("read_complete_ns")
                    response = json.loads(received)
                    if not isinstance(response, dict):
                        raise ValueError("runtime response must be a JSON object")
                    mark("response_ready_ns")
                    return response
                received.extend(byte)
        raise ValueError("runtime response exceeded the protocol limit")
    except Exception:
        mark("error_ns")
        raise
    finally:
        mark("finished_ns")


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
