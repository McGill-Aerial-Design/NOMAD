# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Run the MAVSDK connect/status smoke against SITL."""

from __future__ import annotations

import os
import subprocess
import sys
import time
from pathlib import Path

import psutil

ROOT = Path(__file__).resolve().parents[2]


def get_sitl_port() -> str:
    port = os.environ.get("NOMAD_CORE_SITL_PORT", "14570")
    if not port.isdecimal() or not 1 <= int(port) <= 65535:
        raise ValueError("NOMAD_CORE_SITL_PORT must be a UDP port from 1 to 65535")
    return port


def get_system_id() -> str:
    system_id = os.environ.get("NOMAD_MAVSDK_SYSTEM_ID", "1")
    if not system_id.isdecimal() or not 1 <= int(system_id) <= 255:
        raise ValueError("NOMAD_MAVSDK_SYSTEM_ID must be from 1 to 255")
    return system_id


def find_binary() -> Path | None:
    if os.environ.get("NOMAD_QUALIFICATION_BUILD_DIR"):
        from mavsdk_build_metrics import release_binary

        return release_binary(Path(os.environ["NOMAD_QUALIFICATION_BUILD_DIR"]), "nomad_mavsdk_connectivity_smoke")
    names = ("nomad_mavsdk_connectivity_smoke.exe", "nomad_mavsdk_connectivity_smoke")
    build_dir = ROOT / "build" / "mavsdk-qualification"
    for directory in (build_dir, build_dir / "Debug", build_dir / "Release"):
        for name in names:
            candidate = directory / name
            if candidate.is_file():
                return candidate
    return None


def has_required_output(command: str, output: str, system_id: str) -> bool:
    required = ["connected=true", f"system={system_id}"]
    if command == "status":
        required.extend(("samples=", "position=", "battery_v=", "gps_fix=", "flight_mode="))
    return all(field in output for field in required)


def process_rss_bytes(process: psutil.Process) -> int | None:
    """Return current RSS for a process tree, or unavailable after exit."""
    try:
        children = process.children(recursive=True)
        return process.memory_info().rss + sum(child.memory_info().rss for child in children)
    except (psutil.AccessDenied, psutil.NoSuchProcess):
        return None


def run_measured_process(
    command: list[str], timeout_seconds: float
) -> tuple[subprocess.CompletedProcess[str], float, int | None]:
    """Run a small smoke command and sample its process-tree peak RSS."""
    started = time.perf_counter()
    process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    measured_process = psutil.Process(process.pid)
    peak_rss = process_rss_bytes(measured_process)
    deadline = started + timeout_seconds
    while process.poll() is None:
        current_rss = process_rss_bytes(measured_process)
        if current_rss is not None:
            peak_rss = max(peak_rss or 0, current_rss)
        if time.perf_counter() >= deadline:
            process.kill()
            process.communicate()
            raise subprocess.TimeoutExpired(command, timeout_seconds)
        time.sleep(0.01)
    stdout, stderr = process.communicate()
    result = subprocess.CompletedProcess(command, process.returncode, stdout, stderr)
    return result, time.perf_counter() - started, peak_rss


def report_runtime_metric(command: str, elapsed_seconds: float, peak_rss: int | None) -> None:
    fields = [
        f"command={command}",
        f"elapsed_seconds={elapsed_seconds:.3f}",
    ]
    if peak_rss is not None:
        fields.append(f"peak_process_tree_rss_bytes={peak_rss}")
    else:
        fields.append("peak_process_tree_rss_bytes=unavailable")
    print(f"mavsdk_connectivity_runtime_metric {' '.join(fields)}", flush=True)


def run_smoke(binary: Path, command: str, endpoint: str, system_id: str) -> int:
    print(f"Running MAVSDK {command} smoke against {endpoint}", flush=True)
    try:
        result, elapsed_seconds, peak_rss = run_measured_process([str(binary), command, endpoint, system_id], 20)
    except subprocess.TimeoutExpired:
        print(f"error: MAVSDK {command} smoke timed out", file=sys.stderr)
        return 2
    except OSError as error:
        print(f"error: could not run MAVSDK smoke: {error}", file=sys.stderr)
        return 2

    report_runtime_metric(command, elapsed_seconds, peak_rss)
    if result.stdout:
        print(result.stdout, end="")
    if result.stderr:
        print(result.stderr, end="", file=sys.stderr)
    if result.returncode == 0 and not has_required_output(command, result.stdout, system_id):
        print(f"error: MAVSDK {command} output did not satisfy the evidence contract", file=sys.stderr)
        return 2
    return result.returncode


def main() -> int:
    try:
        port = get_sitl_port()
        system_id = get_system_id()
    except ValueError as error:
        print(f"error: {error}", file=sys.stderr)
        return 2

    binary = find_binary()
    if binary is None:
        print(
            "error: MAVSDK connectivity smoke binary not found; run `pixi run build-mavsdk-connectivity-smoke` first",
            file=sys.stderr,
        )
        return 2

    endpoint = f"udpin:0.0.0.0:{port}"
    for command in ("connect", "status"):
        result = run_smoke(binary, command, endpoint, system_id)
        if result != 0:
            return result
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
