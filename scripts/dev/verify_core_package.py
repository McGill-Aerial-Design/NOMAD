#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Verify the contents and offline CLI smoke of a staged NOMAD core package."""

from __future__ import annotations

import argparse
import os
import socket
import subprocess
import sys
import tarfile
import tempfile
import zipfile
from pathlib import Path, PurePosixPath

REQUIRED_FILES = (
    Path("include/nomad/vehicle/vehicle.hpp"),
    Path("share/nomad/LICENSE"),
    Path("share/nomad/NOTICE"),
    Path("share/nomad/config/README.md"),
    Path("share/nomad/config/nomad.env.example"),
    Path("share/nomad/operations.md"),
    Path("share/nomad/lifecycle/nomad-runtime.service.in"),
    Path("share/nomad/lifecycle/install_systemd.py"),
    Path("share/nomad/lifecycle/Manage-NomadRuntime.ps1"),
    Path("share/nomad/lifecycle/runtime.example.json"),
)
CLI_NAMES = ("nomad", "nomad.exe")
RUNTIME_NAMES = ("nomad-runtime", "nomad-runtime.exe")
QUALIFICATION_NAMES = ("nomad-qualification", "nomad-qualification.exe")
ALLOWED_BIN_NAMES = set(CLI_NAMES + RUNTIME_NAMES)


def find_binary(root: Path, names: tuple[str, ...]) -> Path | None:
    """Return the first packaged executable matching this host-neutral name set."""
    return next((root / "bin" / name for name in names if (root / "bin" / name).is_file()), None)


def find_install_root(base: Path) -> Path:
    """Find the install tree whether an archive adds a top-level directory."""
    if (base / "bin").is_dir():
        return base
    candidates = [item for item in base.iterdir() if item.is_dir() and (item / "bin").is_dir()]
    if len(candidates) != 1:
        raise ValueError(f"expected one package root under {base}, found {len(candidates)}")
    return candidates[0]


def validate_install_root(root: Path) -> list[str]:
    """Return content errors without executing anything."""
    errors = [f"missing {path}" for path in REQUIRED_FILES if not (root / path).is_file()]
    if find_binary(root, CLI_NAMES) is None:
        errors.append("missing bin/nomad or bin/nomad.exe")
    if find_binary(root, RUNTIME_NAMES) is None:
        errors.append("missing bin/nomad-runtime or bin/nomad-runtime.exe")
    if find_binary(root, QUALIFICATION_NAMES) is not None:
        errors.append("package contains non-installed qualification driver")
    bin_dir = root / "bin"
    if bin_dir.is_dir():
        unexpected = sorted(
            item.relative_to(bin_dir).as_posix()
            for item in bin_dir.rglob("*")
            if item.is_file() and item.relative_to(bin_dir).as_posix() not in ALLOWED_BIN_NAMES
        )
        if unexpected:
            errors.append(f"package contains unexpected bin files: {', '.join(unexpected)}")
    license_dir = root / "share/nomad/licenses/mavsdk-phase-a"
    if not license_dir.is_dir() or not any(license_dir.glob("*.txt")):
        errors.append("missing MAVSDK dependency license bundle")
    if (root / "include/mavsdk").exists():
        errors.append("package contains MAVSDK development headers")
    if (root / "lib").exists():
        errors.append("package contains development libraries")
    if (root / "share/nomad/config/nomad.env").exists():
        errors.append("package contains live config/nomad.env")
    return errors


def run_cli(binary: Path, root: Path, arguments: list[str], environment: dict[str, str] | None = None):
    """Run one installed executable command with a bounded wait."""
    return subprocess.run(
        [str(binary.resolve()), *arguments],
        cwd=root,
        env=environment,
        capture_output=True,
        text=True,
        timeout=10,
        check=False,
    )


def verify_usage(binary: Path, root: Path) -> list[str]:
    """Check the installed command list omits direct-mode options."""
    result = run_cli(binary, root, [])
    errors = []
    if result.returncode != 1:
        errors.append(f"usage invocation returned {result.returncode}, expected 1")
    if "Usage: nomad <" not in result.stdout:
        errors.append("usage invocation did not print the NOMAD command list")
    if any(flag in result.stdout for flag in ("--direct", "--endpoint", "--system-id")):
        errors.append("usage invocation exposes direct CLI configuration")
    return errors


def local_runtime_environment() -> dict[str, str]:
    """Select a currently unused loopback port for runtime-client checks."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as probe:
        probe.bind(("127.0.0.1", 0))
        ipc_port = int(probe.getsockname()[1])
    environment = os.environ.copy()
    environment["NOMAD_RUNTIME_IPC_PORT"] = str(ipc_port)
    return environment


def verify_bare_status(binary: Path, root: Path, environment: dict[str, str]) -> list[str]:
    """Check a bare status command contacts only the local runtime."""
    status_result = run_cli(binary, root, ["status"], environment)
    errors = []
    if status_result.returncode == 0 or "error[runtime_unavailable]" not in status_result.stderr:
        errors.append("bare status did not fail through runtime IPC")
    if "heartbeat" in status_result.stderr:
        errors.append("bare status fell back to a direct MAVLink connection")
    return errors


def verify_unsupported_navigation(binary: Path, root: Path, environment: dict[str, str]) -> list[str]:
    """Check an unavailable typed request is rejected before IPC or vehicle access."""
    unsupported_environment = environment.copy()
    unsupported_environment["NOMAD_RUNTIME_IPC_PORT"] = "invalid"
    unsupported = run_cli(binary, root, ["goto", "45", "-73", "10"], unsupported_environment)
    if "error[unsupported_request]" not in unsupported.stderr:
        return ["unsupported navigation verb did not report unavailable"]
    return []


def verify_direct_flag(binary: Path, root: Path) -> list[str]:
    """Check the removed direct selector fails before vehicle transport."""
    direct_flag = run_cli(binary, root, ["--direct", "status"])
    if "Usage: nomad" not in direct_flag.stdout or "heartbeat" in direct_flag.stderr:
        return ["--direct did not fail before vehicle transport"]
    return []


def verify_runtime_help(runtime: Path, root: Path) -> list[str]:
    """Check the installed runtime executable still exposes its help command."""
    result = run_cli(runtime, root, ["--help"])
    if result.returncode != 0 or "Usage: nomad-runtime" not in result.stdout:
        return ["runtime help invocation failed or omitted usage"]
    return []


def verify_cli(root: Path) -> list[str]:
    """Check that installed commands use IPC and never expose direct mode."""
    binary = find_binary(root, CLI_NAMES)
    runtime = find_binary(root, RUNTIME_NAMES)
    if binary is None or runtime is None:
        return ["package executable validation skipped because a required executable is missing"]

    environment = local_runtime_environment()
    errors = verify_usage(binary, root)
    errors.extend(verify_bare_status(binary, root, environment))
    errors.extend(verify_unsupported_navigation(binary, root, environment))
    errors.extend(verify_direct_flag(binary, root))
    errors.extend(verify_runtime_help(runtime, root))
    return errors


def safe_archive_member(name: str) -> bool:
    path = PurePosixPath(name)
    return not path.is_absolute() and ".." not in path.parts


def restore_zip_permissions(members: list[zipfile.ZipInfo], destination: Path) -> None:
    """Preserve archived Unix permissions; ZIP extraction otherwise drops executable bits."""
    if os.name == "nt":
        return
    for member in members:
        mode = (member.external_attr >> 16) & 0o777
        if mode:
            (destination / member.filename).chmod(mode)


def extract_archive(archive: Path, destination: Path) -> Path:
    """Extract a CPack ZIP/TGZ while rejecting path traversal and links."""
    destination.mkdir(parents=True, exist_ok=True)
    if archive.suffix.lower() == ".zip":
        with zipfile.ZipFile(archive) as package:
            members = package.infolist()
            if any(not safe_archive_member(member.filename) for member in members):
                raise ValueError("archive contains an unsafe ZIP path")
            package.extractall(destination)
            restore_zip_permissions(members, destination)
    elif archive.name.endswith((".tar.gz", ".tgz")):
        with tarfile.open(archive) as package:
            members = package.getmembers()
            if any(not safe_archive_member(member.name) or member.issym() or member.islnk() for member in members):
                raise ValueError("archive contains an unsafe TAR member")
            for member in members:
                package.extract(member, destination, filter="data")
    else:
        raise ValueError("package must be an install directory, ZIP, TGZ or tar.gz archive")
    return find_install_root(destination)


def package_inputs(path: Path) -> list[Path]:
    """Resolve an install tree, archive, or CPack output directory."""
    if path.is_file() or (path.is_dir() and (path / "bin").is_dir()):
        return [path]
    archives = sorted(
        item
        for item in path.iterdir()
        if item.is_file() and (item.suffix.lower() == ".zip" or item.name.endswith((".tar.gz", ".tgz")))
    )
    if not archives:
        raise ValueError(f"no install tree or CPack archive found at {path}")
    return archives


def verify_package(path: Path) -> list[str]:
    """Verify an install directory or extracted CPack archive."""
    with tempfile.TemporaryDirectory(prefix="nomad-core-package-") as temporary:
        errors = []
        for index, package in enumerate(package_inputs(path.resolve())):
            root = package if package.is_dir() else extract_archive(package, Path(temporary) / str(index))
            package_errors = validate_install_root(root)
            if not package_errors:
                package_errors.extend(verify_cli(root))
            errors.extend(f"{package.name}: {error}" for error in package_errors)
        return errors


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("package", type=Path, help="cmake --install prefix or CPack ZIP/TGZ archive")
    arguments = parser.parse_args()
    try:
        errors = verify_package(arguments.package)
    except (OSError, ValueError, subprocess.SubprocessError) as error:
        print(f"core package verification failed: {error}", file=sys.stderr)
        return 1
    if errors:
        for error in errors:
            print(f"core package verification failed: {error}", file=sys.stderr)
        return 1
    print(f"core package verified: {arguments.package}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
