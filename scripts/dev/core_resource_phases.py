# SPDX-License-Identifier: Apache-2.0
"""Time named existing build/qualification commands without changing their semantics."""

from __future__ import annotations

import os
import subprocess
import sys
import time
from pathlib import Path

from resource_metadata import ROOT, write_report


def run_phase(name: str, command: list[str], durations: dict, environment: dict | None = None) -> None:
    started = time.perf_counter()
    print(f"[resource phase] {name}", flush=True)
    result = subprocess.run(command, cwd=ROOT, env=environment, timeout=1800, check=False)
    durations[name] = {"seconds": round(time.perf_counter() - started, 6), "exit_code": result.returncode}
    if result.returncode:
        raise subprocess.CalledProcessError(result.returncode, command)


def build_release(build: Path, durations: dict) -> None:
    configure = ["cmake", "-S", str(ROOT), "-B", str(build), "-DCMAKE_BUILD_TYPE=Release", "-DBUILD_TESTING=ON"]
    run_phase("configure_including_dependency_superbuild", configure, durations)
    command = ["cmake", "--build", str(build), "--config", "Release", "--parallel", "2"]
    run_phase("mavsdk_build", [*command, "--target", "mavsdk"], durations)
    run_phase("core_build", [*command, "--target", "nomad", "nomad-runtime"], durations)
    run_phase(
        "qualification_build",
        [
            *command,
            "--target",
            "nomad-qualification",
            "nomad_mavsdk_authority_wire_probe",
            "nomad_mavsdk_connectivity_smoke",
        ],
        durations,
    )
    run_phase("test_targets_build", command, durations)


def qualify_release(build: Path, durations: dict) -> None:
    environment = os.environ.copy()
    environment["NOMAD_RESOURCE_BUILD_DIR"] = str(build)
    environment["NOMAD_MAVSDK_FIXTURE_BUILD_DIR"] = str(build)
    run_phase(
        "cpp_tests", ["ctest", "--test-dir", str(build), "-C", "Release", "--output-on-failure"], durations, environment
    )
    scripts = [
        ("provenance", "check_mavsdk_provenance.py"),
        ("connectivity", "mavsdk_connectivity_peer_fixture.py"),
        ("transport", "mavsdk_connection_fixture.py"),
        ("authority_wire", "mavsdk_authority_wire_fixture.py"),
        ("final_send", "mavsdk_authority_probe_fixture.py"),
        ("runtime_ipc", "runtime_ipc_smoke.py"),
        ("runtime_lifecycle", "runtime_lifecycle_qualification.py"),
    ]
    for name, script in scripts:
        run_phase(name, [sys.executable, str(ROOT / "scripts/dev" / script)], durations, environment)


def package_release(build: Path, durations: dict) -> None:
    run_phase("package", ["cpack", "--config", str(build / "CPackConfig.cmake"), "-C", "Release"], durations)
    run_phase("package_verification", [sys.executable, "scripts/dev/verify_core_package.py", str(build)], durations)
    if (build / "stage").exists():
        raise ValueError("stage already exists; use --observe for an existing verified stage or a fresh build tree")
    run_phase(
        "staged_install",
        [
            "cmake",
            "--install",
            str(build),
            "--prefix",
            str(build / "stage"),
            "--config",
            "Release",
            "--component",
            "nomad_runtime",
        ],
        durations,
    )
    run_phase(
        "staged_verification", [sys.executable, "scripts/dev/verify_core_package.py", str(build / "stage")], durations
    )


def save_phases(build: Path, durations: dict, cache_state: str) -> None:
    write_report(build / "resource-phases.json", {"cache_state": cache_state, "phases": durations})
