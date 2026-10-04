# SPDX-License-Identifier: Apache-2.0
"""Replace only the verified NOMAD DLL while Mission Planner is closed."""

from __future__ import annotations

import hashlib
import os
import shutil
import struct
import subprocess
import tempfile
from pathlib import Path

try:
    from .mission_planner_target import VERSION as MP_VERSION
except ImportError:
    from mission_planner_target import VERSION as MP_VERSION


def run_powershell(script: str, environment=None) -> str:
    """Use positional arguments; never interpolate operator paths into code."""
    environment = dict(os.environ) if environment is None else dict(environment)
    environment = {key: value for key, value in environment.items() if key.casefold() != "psmodulepath"}
    result = subprocess.run(
        ["powershell.exe", "-NoProfile", "-NonInteractive", "-Command", script],
        check=True,
        capture_output=True,
        text=True,
        env=environment,
        timeout=10,
    )
    return result.stdout.strip()


def require_closed() -> None:
    if run_powershell("if (Get-Process -Name MissionPlanner -ErrorAction SilentlyContinue) { 'running' }"):
        raise RuntimeError("Close Mission Planner before activating or rolling back NOMAD; it was not stopped.")


def get_target_version(executable: Path) -> str:
    environment = dict(os.environ, NOMAD_RELEASE_MP_TARGET=str(executable))
    return run_powershell("(Get-Item -LiteralPath $env:NOMAD_RELEASE_MP_TARGET).VersionInfo.FileVersion", environment)


def get_plugin_version(path: Path) -> str:
    environment = dict(os.environ, NOMAD_RELEASE_PLUGIN_PATH=str(path))
    return run_powershell(
        "(Get-Item -LiteralPath $env:NOMAD_RELEASE_PLUGIN_PATH).VersionInfo.ProductVersion", environment
    )


def validate_dll(path: Path) -> None:
    """Reject a non-DLL PE before replacing the deployed plugin."""
    with path.open("rb") as stream:
        header = stream.read(64)
        if len(header) != 64 or header[:2] != b"MZ":
            raise ValueError("NOMADPlugin.dll has no DOS/PE header")
        stream.seek(struct.unpack_from("<I", header, 60)[0])
        pe = stream.read(24)
    if len(pe) != 24 or pe[:4] != b"PE\0\0" or not (struct.unpack_from("<H", pe, 22)[0] & 0x2000):
        raise ValueError("NOMADPlugin.dll is not a PE DLL")


def digest(path: Path) -> str:
    value = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            value.update(chunk)
    return value.hexdigest()


def require_plain_path(path: Path) -> None:
    for item in (path, *path.parents):
        if item.is_symlink() or (item.exists() and getattr(item.stat(), "st_file_attributes", 0) & 0x400):
            raise ValueError("Plugin installation must not use links or reparse points")


class PluginAdapter:
    """The deployment journal retains immutable payloads; this adapter owns one DLL."""

    def __init__(
        self,
        mission_planner: Path,
        target_version=MP_VERSION,
        process_check=None,
        version_reader=None,
        payload_version_reader=None,
    ):
        require_plain_path(mission_planner.absolute())
        self.directory = mission_planner.resolve()
        self.target_version = target_version
        self.process_check = process_check or require_closed
        self.version_reader = version_reader or get_target_version
        self.payload_version_reader = payload_version_reader or get_plugin_version
        self.target = self.directory / "plugins" / "NOMADPlugin.dll"

    def preflight(self, candidate: dict) -> None:
        self.process_check()
        require_plain_path(self.target)
        executable = self.directory / "MissionPlanner.exe"
        if not executable.is_file():
            raise ValueError("MissionPlanner.exe is absent from the requested installation")
        version = self.version_reader(executable).split(".")
        if ".".join(version[:3]) != self.target_version:
            raise ValueError(f"Unsupported Mission Planner target; require {self.target_version}")
        if candidate.get("mission_planner_target", self.target_version) != self.target_version:
            raise ValueError("Release expects a different Mission Planner target")
        payload = Path(candidate["install_path"]) / "NOMADPlugin.dll"
        validate_dll(payload)
        expected = candidate["version"] + "+" + candidate["source_sha"]
        if self.payload_version_reader(payload) != expected:
            raise ValueError("NOMAD DLL implementation version/source disagrees with the release metadata")

    def stop(self) -> None:
        self.process_check()

    def has_unmanaged(self) -> bool:
        return self.target.exists()

    def current_matches(self, candidate: dict) -> bool:
        require_plain_path(self.target)
        return self.target.is_file() and digest(self.target) == candidate["payload_sha256"]["NOMADPlugin.dll"]

    def switch(self, candidate: dict | None) -> None:
        self.process_check()
        require_plain_path(self.target)
        if candidate is None:
            self.target.unlink(missing_ok=True)
            return
        self.target.parent.mkdir(parents=True, exist_ok=True)
        source = Path(candidate["install_path"]) / "NOMADPlugin.dll"
        descriptor, temporary = tempfile.mkstemp(prefix=".nomad-", dir=self.target.parent)
        os.close(descriptor)
        try:
            shutil.copyfile(source, temporary)
            with open(temporary, "r+b") as stream:
                os.fsync(stream.fileno())
            os.replace(temporary, self.target)
        except PermissionError as error:
            raise RuntimeError("NOMAD DLL is locked or access denied; close Mission Planner and retry.") from error
        finally:
            Path(temporary).unlink(missing_ok=True)

    def start(self) -> None:
        self.process_check()

    def health(self, candidate: dict) -> None:
        self.process_check()
        expected = candidate["payload_sha256"]["NOMADPlugin.dll"]
        if not self.target.is_file() or digest(self.target) != expected:
            raise RuntimeError("Deployed NOMAD plugin does not match the verified release payload")
