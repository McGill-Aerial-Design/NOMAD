# SPDX-License-Identifier: Apache-2.0
"""Independent standalone router supervision and loopback candidate verification."""

from __future__ import annotations

import json
import os
import socket
import subprocess
import tempfile
import time
from pathlib import Path

from scripts.release import storage


def request(port: int, operation: str) -> dict:
    message = {"protocol": "nomad-link-router", "version": 1, "id": 1, "type": operation}
    with socket.create_connection(("127.0.0.1", port), timeout=2) as client:
        client.sendall((json.dumps(message) + "\n").encode())
        with client.makefile("rb") as stream:
            line = stream.readline(65537)
    if not line.endswith(b"\n") or len(line) > 65536:
        raise ValueError("router health response is incomplete or oversized")
    return json.loads(line)


def wait(predicate, description: str, seconds: float = 30) -> None:
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        try:
            if predicate():
                return
        except (OSError, ValueError):
            pass
        time.sleep(0.1)
    raise TimeoutError(description)


def health(port: int, candidate: dict) -> bool:
    hello = request(port, "hello")
    status = request(port, "get_status")
    return (
        hello.get("ok")
        and hello.get("protocol") == "nomad-link-router"
        and hello.get("version") == 1
        and hello.get("implementationVersion") == candidate["version"]
        and status.get("ok")
        and isinstance(status.get("status"), dict)
    )


def free_port(protocol: int) -> int:
    with socket.socket(socket.AF_INET, protocol) as peer:
        peer.bind(("127.0.0.1", 0))
        return peer.getsockname()[1]


def executable(candidate: dict) -> Path:
    result = Path(candidate["install_path"]) / "nomad-link-router.exe"
    storage.reject_links(result)
    if not result.is_file():
        raise ValueError("verified standalone router executable is missing")
    return result


def qualify_candidate(candidate: dict) -> None:
    """A local test router uses no operator topology or physical endpoints."""
    management = free_port(socket.SOCK_STREAM)
    config = {
        "ManagementPort": management,
        "PreferredLink": "fixture",
        "Links": [{"Id": "fixture", "BindAddress": "127.0.0.1", "Port": free_port(socket.SOCK_DGRAM), "Priority": 100}],
        "Consumers": [
            {
                "Id": "fixture",
                "RouterPort": free_port(socket.SOCK_DGRAM),
                "ClientPort": free_port(socket.SOCK_DGRAM),
                "AllowOutbound": False,
            }
        ],
    }
    with tempfile.TemporaryDirectory(prefix="nomad-router-preflight-") as directory:
        path = Path(directory) / "router.json"
        path.write_text(json.dumps(config), encoding="utf-8")
        with tempfile.TemporaryFile() as log:
            child = subprocess.Popen(
                [str(executable(candidate)), str(path)],
                stdin=subprocess.PIPE,
                stdout=log,
                stderr=log,
                creationflags=subprocess.CREATE_NO_WINDOW if os.name == "nt" else 0,
            )
            try:
                wait(lambda: health(management, candidate), "router candidate loopback health failed", 10)
                child.stdin.write(b"stop\n")
                child.stdin.flush()
                if child.wait(timeout=10) != 0:
                    raise RuntimeError("router candidate did not shut down safely")
            finally:
                if child.poll() is None:
                    child.kill()
                    child.wait(timeout=10)


class RouterAdapter:
    def __init__(self, root: Path, config: Path, management_port: int, start_command: list[str]):
        self.root, self.config = root / "router", config.absolute()
        self.port, self.start_command = management_port, start_command
        self.current = self.root / "current.json"
        self.process = self.root / "process.json"
        self.stop_request = self.root / "stop.request"
        if not start_command or not all(isinstance(arg, str) and arg for arg in start_command):
            raise ValueError("router start command requires nonempty JSON argv")

    def has_unmanaged(self) -> bool:
        running = storage.record_exists(self.process) and not self.supervisor_stopped()
        return storage.record_exists(self.current) or running or self.port_open()

    def port_open(self) -> bool:
        try:
            with socket.create_connection(("127.0.0.1", self.port), timeout=0.2):
                return True
        except OSError:
            return False

    def supervisor_stopped(self) -> bool:
        if storage.record_exists(self.process) and storage.read_json(self.process).get("status") not in {
            "stopped",
            "failed",
        }:
            return False
        if self.port_open():
            return False
        with storage.lock(self.root / "supervisor"):
            return True

    def current_matches(self, candidate: dict) -> bool:
        return storage.record_exists(self.current) and storage.read_json(self.current) == candidate

    def preflight(self, candidate: dict) -> None:
        storage.reject_links(self.config)
        if not self.config.is_file() or self.config.is_relative_to(self.root):
            raise ValueError("router config must be external to versioned deployment state")
        settings = storage.read_json(self.config)
        if settings.get("ManagementPort") != self.port:
            raise ValueError("router health port disagrees with external configuration")
        qualify_candidate(candidate)

    def stop(self) -> None:
        if not storage.record_exists(self.process):
            if self.port_open():
                raise RuntimeError("an unmanaged router is listening; adopt or stop it explicitly")
            wait(self.supervisor_stopped, "router supervisor has not released its lifetime lock")
            return
        if storage.read_json(self.process).get("status") in {"stopped", "failed"}:
            wait(self.supervisor_stopped, "previous router supervisor is still stopping")
            return
        storage.write_json(self.stop_request, {"stop": True})
        wait(
            self.supervisor_stopped,
            "router supervisor did not verify child shutdown",
        )

    def switch(self, candidate: dict | None) -> None:
        if candidate is None:
            self.current.unlink(missing_ok=True)
            return
        storage.write_json(self.current, candidate)

    def start(self) -> None:
        self.stop()
        self.process.unlink(missing_ok=True)
        subprocess.run(
            self.start_command,
            check=True,
            timeout=30,
            capture_output=True,
            creationflags=subprocess.CREATE_NO_WINDOW if os.name == "nt" else 0,
        )
        expected = storage.read_json(self.current)["release_version"]
        wait(lambda: self.started(expected), "router supervisor did not acknowledge the candidate startup")

    def started(self, version: str) -> bool:
        if not storage.record_exists(self.process):
            return False
        state = storage.read_json(self.process)
        if state.get("release_version") != version:
            return False
        if state.get("status") == "failed":
            raise RuntimeError("router child exited during startup")
        return state.get("status") == "running"

    def health(self, candidate: dict) -> None:
        wait(lambda: health(self.port, candidate), "active router management health failed")
