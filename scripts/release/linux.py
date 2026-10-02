# SPDX-License-Identifier: Apache-2.0
"""PR56 systemd lifecycle through a stopped, atomically switched core symlink."""

from __future__ import annotations

import os
import subprocess
from pathlib import Path

from scripts.release import health, storage


class SystemdAdapter:
    def __init__(self, root: Path, config: Path, port: int, runner=None):
        self.pointer = root.absolute() / "core" / "current"
        self.config = config.absolute()
        self.port = port
        self.runner = runner or subprocess.run

    def command(self, *arguments: str):
        return self.runner(
            ["systemctl", *arguments, "nomad-runtime.service"], check=True, capture_output=True, text=True, timeout=40
        )

    def preflight(self, candidate: dict) -> None:
        if not self.config.is_file() or not (Path(candidate["install_path"]) / "bin/nomad-runtime").is_file():
            raise ValueError("systemd activation requires staged core and external protected configuration")
        result = self.command("show", "--property=ExecStart", "--value")
        if str(self.pointer / "bin/nomad-runtime") not in result.stdout or str(self.config) not in result.stdout:
            raise ValueError("provision PR56 unit with current/bin/nomad-runtime and the external config first")

    def stop(self) -> None:
        self.command("stop")
        result = self.command("show", "--property=ActiveState", "--value")
        if result.stdout.strip() not in {"inactive", "failed"}:
            raise RuntimeError("systemd runtime did not stop; active pointer unchanged")

    def has_unmanaged(self) -> bool:
        return self.pointer.exists() or self.pointer.is_symlink()

    def current_matches(self, candidate: dict) -> bool:
        return self.pointer.is_symlink() and self.pointer.readlink() == Path(candidate["install_path"])

    def switch(self, candidate: dict | None) -> None:
        if self.command("show", "--property=ActiveState", "--value").stdout.strip() not in {"inactive", "failed"}:
            raise RuntimeError("runtime must be stopped before switching the active symlink")
        if candidate is None:
            self.pointer.unlink(missing_ok=True)
            storage.sync_directory(self.pointer.parent)
            return
        temporary = self.pointer.with_name(".current-new")
        temporary.unlink(missing_ok=True)
        os.symlink(candidate["install_path"], temporary, target_is_directory=True)
        os.replace(temporary, self.pointer)
        storage.sync_directory(self.pointer.parent)

    def start(self) -> None:
        self.command("reset-failed")
        self.command("start")

    def health(self, candidate: dict) -> None:
        health.core(self.port, candidate)
