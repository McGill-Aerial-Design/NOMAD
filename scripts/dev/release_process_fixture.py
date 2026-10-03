# SPDX-License-Identifier: Apache-2.0
"""Unprivileged real-child adapters for deployment transaction qualification."""

from __future__ import annotations

import os
import subprocess
from pathlib import Path

from ground_router_smoke import management_request
from runtime_ipc_smoke import request, require, stop_runtime
from runtime_lifecycle_fixture import wait_for


class RuntimeProcess:
    """Keep production IPC and final-send shutdown, substituting only the OS supervisor."""

    def __init__(self, config: Path, port: int, directory: Path):
        self.config, self.port = config, port
        self.candidate: dict | None = None
        self.child: subprocess.Popen | None = None
        self.log = (directory / "runtime-process.log").open("w+b")
        self.fail_version: str | None = None
        self.fail_start = False

    def executable(self, candidate: dict) -> Path:
        name = "nomad-runtime.exe" if os.name == "nt" else "nomad-runtime"
        paths = list(Path(candidate["install_path"]).rglob(name))
        require(len(paths) == 1, "staged core contains exactly one runtime executable")
        return paths[0]

    def has_unmanaged(self) -> bool:
        return self.child is not None and self.child.poll() is None

    def current_matches(self, candidate: dict) -> bool:
        return self.candidate == candidate and self.has_unmanaged()

    def preflight(self, candidate: dict) -> None:
        require(self.config.is_file(), "external protected runtime configuration remains available")
        result = subprocess.run([str(self.executable(candidate)), "--version"], capture_output=True, timeout=10)
        require(result.returncode == 0, "candidate runtime version probe starts without a vehicle connection")
        label = "A" if candidate["source_sha"] == "a" * 40 else "B"
        require(("0.0.0-fixture." + label).encode() in result.stdout, "candidate reports expected fixture version")

    def stop(self) -> None:
        if self.child is None or self.child.poll() is not None:
            return
        stop_runtime(self.child)
        require(self.child.returncode == 0, "previous runtime closes final-send admission and drains cleanly")

    def switch(self, candidate: dict | None) -> None:
        require(self.child is None or self.child.poll() is not None, "switch occurs only after previous runtime exited")
        self.candidate = candidate

    def start(self) -> None:
        if self.candidate is None:
            return
        options = {"creationflags": subprocess.CREATE_NEW_PROCESS_GROUP} if os.name == "nt" else {}
        command = [str(self.executable(self.candidate)), "--config", str(self.config)]
        if self.fail_start:
            self.fail_start = False
            command.extend(["--ipc-port", "invalid"])
        self.child = subprocess.Popen(command, stdout=self.log, stderr=self.log, **options)

    def ready(self) -> bool:
        if self.child.poll() is not None:
            raise RuntimeError("candidate runtime exited during startup")
        return request(self.port, "release-health", "status")["status"]["runtime_ready"]

    def health(self, candidate: dict) -> None:
        wait_for(self.ready, "runtime not ready")
        hello = request(self.port, "release-hello", "hello")
        require(hello["ok"] and hello["version"] == 1, "candidate runtime IPC protocol is compatible")
        label = "A" if candidate["source_sha"] == "a" * 40 else "B"
        require(hello["runtime_version"] == "0.0.0-fixture." + label, "active runtime reports exact fixture version")
        status = request(self.port, "release-owner", "status")["status"]
        require(status["authority_owner"] is None, "candidate starts without restored software authority")
        if candidate["release_version"] == self.fail_version:
            raise RuntimeError("intentional candidate runtime health failure")

    def close(self) -> None:
        try:
            self.stop()
        finally:
            self.log.close()


class RouterProcess:
    """Run the standalone router against external loopback-only configuration."""

    def __init__(self, config: Path, port: int, directory: Path):
        self.config, self.port = config, port
        self.candidate: dict | None = None
        self.child: subprocess.Popen | None = None
        self.log = (directory / "router-process.log").open("w+b")
        self.fail_version: str | None = None

    def executable(self, candidate: dict) -> Path:
        paths = list(Path(candidate["install_path"]).rglob("nomad-link-router.exe"))
        require(len(paths) == 1, "staged router contains exactly one standalone executable")
        return paths[0]

    def has_unmanaged(self) -> bool:
        return self.child is not None and self.child.poll() is None

    def current_matches(self, candidate: dict) -> bool:
        return self.candidate == candidate and self.has_unmanaged()

    def preflight(self, candidate: dict) -> None:
        require(self.config.is_file(), "external router topology remains available")
        self.executable(candidate)

    def stop(self) -> None:
        if self.child is None or self.child.poll() is not None:
            return
        self.child.stdin.write(b"stop\n")
        self.child.stdin.flush()
        require(self.child.wait(timeout=10) == 0, "router safe shutdown completes before executable switch")

    def switch(self, candidate: dict | None) -> None:
        require(self.child is None or self.child.poll() is not None, "router switch follows verified process exit")
        self.candidate = candidate

    def start(self) -> None:
        if self.candidate is None:
            return
        command = [str(self.executable(self.candidate)), str(self.config)]
        self.child = subprocess.Popen(
            command,
            stdin=subprocess.PIPE,
            stdout=self.log,
            stderr=self.log,
            creationflags=subprocess.CREATE_NO_WINDOW if os.name == "nt" else 0,
        )

    def health(self, candidate: dict) -> None:
        def ready() -> bool:
            hello = management_request(self.port, {"id": 1, "type": "hello"})
            status = management_request(self.port, {"id": 2, "type": "get_status"})
            label = "A" if candidate["source_sha"] == "a" * 40 else "B"
            return (
                hello["ok"]
                and hello["version"] == 1
                and status["ok"]
                and bool(status["status"]["links"])
                and hello["implementationVersion"] == "0.0.0-fixture." + label
            )

        wait_for(ready, "router candidate management hello/status not ready")
        if candidate["release_version"] == self.fail_version:
            raise RuntimeError("intentional candidate router health failure")

    def close(self) -> None:
        try:
            self.stop()
        finally:
            self.log.close()
