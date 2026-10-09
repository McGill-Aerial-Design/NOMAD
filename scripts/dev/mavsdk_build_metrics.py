# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Collect repeatable build-footprint metrics for the MAVSDK connectivity smoke."""

from __future__ import annotations

import json
import os
import subprocess
import sys
import time
from argparse import ArgumentParser
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_BUILD_DIR = ROOT / "build" / "mavsdk-qualification"


def release_binary(build: Path, name: str) -> Path:
    candidates = [build / "Release" / f"{name}.exe", build / f"{name}.exe", build / name]
    binary = next((item for item in candidates if item.is_file()), None)
    if binary is None:
        raise FileNotFoundError(f"Release {name} not found; build production targets first")
    return binary


def run_build(build_dir: Path = DEFAULT_BUILD_DIR) -> dict[str, float]:
    """Configure and build the connectivity smoke while measuring both steps."""
    configure = [
        "cmake",
        "-S",
        str(ROOT),
        "-B",
        str(build_dir),
        "-DCMAKE_BUILD_TYPE=Release",
        "-DBUILD_TESTING=OFF",
    ]
    build = [
        "cmake",
        "--build",
        str(build_dir),
        "--config",
        "Release",
        "--target",
        "nomad_mavsdk_connectivity_smoke",
    ]
    durations = {}
    for name, command in (("configure_seconds", configure), ("target_build_seconds", build)):
        started = time.perf_counter()
        subprocess.run(command, check=True)
        durations[name] = round(time.perf_counter() - started, 3)
    return durations


def directory_size(path: Path) -> int:
    """Return the total byte size of regular files below *path*."""
    return sum(item.stat().st_size for item in path.rglob("*") if item.is_file())


def find_smoke_binary(build_dir: Path = DEFAULT_BUILD_DIR) -> Path | None:
    """Find the single- or multi-config connectivity smoke executable."""
    names = ("nomad_mavsdk_connectivity_smoke.exe", "nomad_mavsdk_connectivity_smoke")
    for directory in (build_dir, build_dir / "Release", build_dir / "Debug"):
        for name in names:
            candidate = directory / name
            if candidate.is_file():
                return candidate
    return None


def collect_build_metrics(build_dir: Path = DEFAULT_BUILD_DIR) -> dict[str, int | str]:
    """Collect byte-count evidence from a completed connectivity-smoke build."""
    if not build_dir.is_dir():
        message = f"MAVSDK qualification build directory not found: {build_dir}"
        raise FileNotFoundError(message)

    binary = find_smoke_binary(build_dir)
    if binary is None:
        raise FileNotFoundError("MAVSDK connectivity smoke executable was not found")

    install_lib = build_dir / "mavsdk" / "third_party" / "install" / "lib"
    archives = []
    if install_lib.is_dir():
        archives = [item for item in install_lib.rglob("*") if item.is_file() and item.suffix.lower() in {".a", ".lib"}]

    return {
        "build_tree_bytes": directory_size(build_dir),
        "smoke_executable_bytes": binary.stat().st_size,
        "selected_static_archives": len(archives),
        "selected_static_archives_bytes": sum(item.stat().st_size for item in archives),
    }


def write_github_summary(metrics: dict[str, int | str], summary_path: Path) -> None:
    """Append the build metrics to the current GitHub Actions job summary."""
    rows = [
        "### MAVSDK connectivity smoke build metrics",
        "",
        "| Metric | Value |",
        "|---|---:|",
    ]
    rows.extend(f"| `{key}` | {value} |" for key, value in metrics.items())
    rows.append("")
    with summary_path.open("a", encoding="utf-8") as stream:
        stream.write("\n".join(rows))


def parse_arguments() -> tuple[bool, Path | None]:
    parser = ArgumentParser(description=__doc__)
    parser.add_argument("--build", action="store_true", help="configure and build before collecting metrics")
    parser.add_argument("--output", type=Path, help="write the JSON evidence record to this path")
    arguments = parser.parse_args()
    return arguments.build, arguments.output


def write_json(metrics: dict[str, float | int | str], output_path: Path) -> None:
    """Write a stable JSON evidence record outside the measured build tree."""
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(metrics, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def main() -> int:
    build_requested, output_path = parse_arguments()
    try:
        durations = run_build() if build_requested else {}
        metrics: dict[str, float | int | str] = collect_build_metrics()
        metrics.update(durations)
        if output_path:
            write_json(metrics, output_path)
    except (FileNotFoundError, OSError, subprocess.CalledProcessError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 2

    print(f"mavsdk_connectivity_build_metrics={json.dumps(metrics, sort_keys=True)}")
    summary = os.environ.get("GITHUB_STEP_SUMMARY")
    if summary:
        try:
            write_github_summary(metrics, Path(summary))
        except OSError as error:
            message = f"warning: could not write GitHub step summary: {error}"
            print(message, file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
