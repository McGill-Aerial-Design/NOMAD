# SPDX-License-Identifier: Apache-2.0
"""Activate a verified version through the existing PR56 Windows SCM service."""

from __future__ import annotations

import os
import re
import subprocess
import time
from pathlib import Path


def service_command(executable: Path, config: Path) -> str:
    for path in (executable, config):
        if not path.is_absolute() or any(value in str(path) for value in ('"', "\r", "\n")):
            raise ValueError("SCM executable and config require absolute paths without quotes or newlines")
    return f'"{executable}" --service --config "{config}"'


class ScmAdapter:
    """Preserve service account, recovery policy and protected external configuration."""

    requires_adoption = True

    def __init__(self, config: Path, health_check, runner=None, timeout=30.0):
        self.config = config.absolute()
        self.health_check = health_check
        self.runner = runner or subprocess.run
        self.timeout = timeout
        self.tool = str(Path(os.environ.get("SystemRoot", "C:/Windows")) / "System32" / "sc.exe")

    def command(self, *arguments: str):
        result = self.runner([self.tool, *arguments], capture_output=True, text=True, timeout=10)
        if result.returncode:
            raise RuntimeError(f"SCM {arguments[0]} failed with code {result.returncode}")
        return result

    def state(self) -> int:
        result = self.command("query", "nomad-runtime")
        match = re.search(r"STATE\s*:\s*(\d+)", result.stdout)
        if match is None:
            raise RuntimeError("SCM did not report a recognized runtime service state")
        return int(match.group(1))

    def wait(self, expected: int) -> None:
        deadline = time.monotonic() + self.timeout
        while time.monotonic() < deadline:
            if self.state() == expected:
                return
            time.sleep(0.1)
        raise TimeoutError("SCM runtime service did not reach the required state")

    def preflight(self, candidate: dict) -> None:
        executable = Path(candidate["install_path"]) / "bin" / "nomad-runtime.exe"
        if not executable.is_file() or not self.config.is_file():
            raise ValueError("SCM activation requires a complete staged runtime and external configuration")
        service_command(executable, self.config)
        self.state()

    def stop(self) -> None:
        if self.state() != 1:
            self.command("stop", "nomad-runtime")
        self.wait(1)

    def has_unmanaged(self) -> bool:
        self.state()
        return True

    def current_matches(self, candidate: dict) -> bool:
        executable = Path(candidate["install_path"]) / "bin" / "nomad-runtime.exe"
        expected = service_command(executable, self.config)
        result = self.command("qc", "nomad-runtime")
        match = re.search(r"BINARY_PATH_NAME\s*:\s*(.+)", result.stdout)
        return match is not None and match.group(1).strip() == expected

    def switch(self, candidate: dict | None) -> None:
        if self.state() != 1:
            raise RuntimeError("Stop the runtime service before changing its versioned executable")
        if candidate is None:
            return
        executable = Path(candidate["install_path"]) / "bin" / "nomad-runtime.exe"
        self.command("config", "nomad-runtime", "binPath=", service_command(executable, self.config))

    def start(self) -> None:
        self.command("start", "nomad-runtime")
        self.wait(4)

    def health(self, candidate: dict) -> None:
        self.wait(4)
        self.health_check(candidate)
