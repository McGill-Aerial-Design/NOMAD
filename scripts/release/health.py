# SPDX-License-Identifier: Apache-2.0
"""Read-only health probes. No second vehicle transport or authority admission."""

from __future__ import annotations

import json
import socket
import time


def request(port: int, protocol: str, kind: str) -> dict:
    message = {
        "protocol": protocol,
        "version": 1,
        "id": "deployment-health" if protocol == "nomad-core" else 1,
        "type": kind,
        "client_id": "deployment-health",
    }
    with socket.create_connection(("127.0.0.1", port), timeout=1) as client:
        client.settimeout(1)
        client.sendall((json.dumps(message) + "\n").encode())
        response = b""
        while not response.endswith(b"\n"):
            block = client.recv(4096)
            if not block or len(response) + len(block) > 1024 * 1024:
                raise ValueError("invalid health response")
            response += block
    value = json.loads(response)
    if not value.get("ok") or value.get("protocol") != protocol or value.get("version") != 1:
        raise ValueError("incompatible health response")
    return value


def wait(check, timeout: float = 30) -> None:
    deadline = time.monotonic() + timeout
    last = None
    while time.monotonic() < deadline:
        try:
            check()
            return
        except (OSError, ValueError, KeyError, RuntimeError) as error:
            last = error
        time.sleep(0.1)
    raise TimeoutError(f"candidate health check failed: {last}")


def core(port: int, candidate: dict) -> None:
    def check():
        hello = request(port, "nomad-core", "hello")
        status = request(port, "nomad-core", "status")["status"]
        if not status["runtime_ready"] or status["authority_owner"] is not None:
            raise ValueError("runtime not ready or authority unexpectedly owned")
        if hello.get("runtime_version") != candidate["version"]:
            raise ValueError("runtime implementation version mismatch")
        if not status["runtime_incarnation"]:
            raise ValueError("runtime incarnation missing")

    wait(check)


def router(port: int, candidate: dict) -> None:
    def check():
        hello = request(port, "nomad-link-router", "hello")
        request(port, "nomad-link-router", "get_status")
        if hello.get("implementationVersion") != candidate["version"]:
            raise ValueError("router implementation version mismatch")

    wait(check)
