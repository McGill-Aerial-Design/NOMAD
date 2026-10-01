# SPDX-License-Identifier: Apache-2.0
"""Qualification-only process supervision driver; never installed with the product."""

from __future__ import annotations

import json
import os
import subprocess
import threading
import time
from pathlib import Path

from runtime_ipc_smoke import stop_runtime, write_fixture_credentials


def write_private_json(path: Path, value: object) -> None:
    """Create protected test configuration using the current test account."""
    descriptor = os.open(path, os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o600)
    with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
        json.dump(value, stream)
    if os.name == "nt":
        subprocess.run(["icacls", str(path), "/setowner", os.getlogin()], capture_output=True, check=True)
        subprocess.run(
            [
                "icacls",
                str(path),
                "/inheritance:r",
                "/grant:r",
                f"{os.getlogin()}:(F)",
                "*S-1-5-18:(F)",
                "*S-1-5-32-544:(F)",
            ],
            capture_output=True,
            check=True,
        )


def deployment(directory: Path, udp: int, ipc: int) -> tuple[Path, dict[str, str]]:
    """Persist credentials and history across all child incarnations."""
    credentials = write_fixture_credentials(str(directory))
    settings = {
        "NOMAD_MAVLINK_ENDPOINT": f"udpin:127.0.0.1:{udp}",
        "NOMAD_RUNTIME_IPC_PORT": str(ipc),
        "NOMAD_CLIENT_CREDENTIALS_FILE": str(credentials),
        "NOMAD_AUDIT_DIRECTORY": str(directory / "audit"),
        "NOMAD_API_KEY": "qualification-only-enable-gate",
    }
    config = directory / "runtime.json"
    write_private_json(config, settings)
    return config, settings


class ProcessSupervisor:
    """Test OS-policy inputs with real children and short, bounded fixture delays.

    This is a fault-injection harness, not an alternative production supervisor.
    Platform tests verify the systemd/SCM adapters and actual configured delays.
    """

    def __init__(self, command: list[str], directory: Path, delays: tuple[float, ...] = (0.1, 0.2)):
        self.command, self.delays = command, delays
        self.child: subprocess.Popen[bytes] | None = None
        self.starts: list[float] = []
        self.exits: list[int] = []
        self.stop_event = threading.Event()
        self.log = (directory / "process.log").open("w+b")
        self.worker = threading.Thread(target=self._run, daemon=True)

    def start(self) -> None:
        self.worker.start()

    def _run(self) -> None:
        options = {"creationflags": subprocess.CREATE_NEW_PROCESS_GROUP} if os.name == "nt" else {}
        while not self.stop_event.is_set():
            self.child = subprocess.Popen(self.command, stdout=self.log, stderr=self.log, **options)
            self.starts.append(time.monotonic())
            code = self.child.wait()
            self.exits.append(code)
            if code in (0, 78) or len(self.exits) > len(self.delays):
                return
            if self.stop_event.wait(self.delays[len(self.exits) - 1]):
                return

    def stop(self) -> None:
        self.stop_event.set()
        if self.child is not None:
            stop_runtime(self.child)
        self.worker.join(timeout=15)
        if self.worker.is_alive():
            raise TimeoutError("qualification supervisor did not drain")
        if self.child is not None:
            stop_runtime(self.child)

    def close(self) -> None:
        self.stop()
        self.log.close()

    def logs(self) -> str:
        self.log.flush()
        self.log.seek(0)
        return self.log.read().decode("utf-8", errors="replace")


def wait_for(predicate, description: str, seconds: float = 15) -> None:
    """Use an explicit polling deadline for independent observed state."""
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        try:
            if predicate():
                return
        except (OSError, ConnectionError):
            pass
        time.sleep(0.05)
    raise TimeoutError(description)
