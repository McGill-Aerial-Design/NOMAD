# SPDX-License-Identifier: Apache-2.0
"""Build two real runtime identities, retaining the existing dependency build cache."""

from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def run(*arguments: str) -> None:
    subprocess.run(arguments, cwd=ROOT, check=True)


def build_fixture(build: Path, baseline: Path, label: str) -> None:
    prefix = ROOT / "build" / "release-fixtures" / ("core-" + label)
    run(
        "cmake",
        "-S",
        ".",
        "-B",
        str(build),
        "-DCMAKE_BUILD_TYPE=Debug",
        "-DBUILD_TESTING=ON",
        "-DNOMAD_FIXTURE_VERSION=0.0.0-fixture." + label,
    )
    run("cmake", "--build", str(build), "--config", "Debug", "--target", "nomad", "nomad-runtime")
    shutil.copytree(baseline, prefix, dirs_exist_ok=True)
    for installed in (prefix / "bin").iterdir():
        candidate = build / "Debug" / installed.name
        if not candidate.is_file():
            candidate = build / installed.name
        if not candidate.is_file():
            raise FileNotFoundError("fixture build did not produce " + installed.name)
        shutil.copy2(candidate, installed)


def main() -> int:
    build = ROOT / "build" / "core"
    baseline = ROOT / "build" / "release-fixtures" / "core-baseline"
    run(
        "cmake",
        "-S",
        ".",
        "-B",
        str(build),
        "-DCMAKE_BUILD_TYPE=Debug",
        "-DBUILD_TESTING=ON",
        "-DNOMAD_FIXTURE_VERSION=",
    )
    run("cmake", "--build", str(build), "--config", "Debug", "--target", "nomad", "nomad-runtime")
    run(
        "cmake", "--install", str(build), "--config", "Debug", "--component", "nomad_runtime", "--prefix", str(baseline)
    )
    try:
        for label in ("A", "B"):
            build_fixture(build, baseline, label)
    finally:
        run(
            "cmake",
            "-S",
            ".",
            "-B",
            str(build),
            "-DCMAKE_BUILD_TYPE=Debug",
            "-DBUILD_TESTING=ON",
            "-DNOMAD_FIXTURE_VERSION=",
        )
        run("cmake", "--build", str(build), "--config", "Debug", "--target", "nomad", "nomad-runtime")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
