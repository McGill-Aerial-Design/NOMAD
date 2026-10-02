# SPDX-License-Identifier: Apache-2.0
"""Comparable, path-free metadata for production-core resource evidence."""

from __future__ import annotations

import json
import platform
import re
import subprocess
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def git_sha(directory: Path) -> str:
    return subprocess.check_output(["git", "-C", str(directory), "rev-parse", "HEAD"], text=True).strip()


def read_cache(build: Path) -> dict[str, str]:
    entries = {}
    for line in (build / "CMakeCache.txt").read_text(encoding="utf-8").splitlines():
        if line and not line.startswith(("#", "//")) and "=" in line and ":" in line.split("=", 1)[0]:
            key, value = line.split("=", 1)
            entries[key.split(":", 1)[0]] = value
    return entries


def compiler_metadata(build: Path) -> dict[str, str]:
    candidates = sorted((build / "CMakeFiles").glob("*/CMakeCXXCompiler.cmake"))
    if len(candidates) != 1:
        raise ValueError("expected one configured C++ toolchain; use a fresh build directory")
    content = candidates[0].read_text(encoding="utf-8")
    values = {}
    for key in ("ID", "VERSION"):
        match = re.search(rf'set\(CMAKE_CXX_COMPILER_{key} "([^"]+)"\)', content)
        if match is None:
            raise ValueError(f"missing compiler {key}")
        values[key.lower()] = match[1]
    return values


def collect_metadata(build: Path, cache_state: str) -> dict:
    cache = read_cache(build)
    if cache.get("CMAKE_BUILD_TYPE") != "Release":
        raise ValueError("resource footprint requires a Release build")
    configuration = json.loads((build / "resource-build-config.json").read_text(encoding="utf-8"))
    configuration["plugins"] = sorted(configuration["plugins"])
    configuration.update({"generator": cache["CMAKE_GENERATOR"], "build_testing": cache["BUILD_TESTING"]})
    return {
        "nomad_sha": git_sha(ROOT),
        "mavsdk_sha": git_sha(ROOT / "third_party/MAVSDK"),
        "os": platform.system(),
        "os_release": platform.release(),
        "architecture": platform.machine().lower().replace("amd64", "x86_64"),
        "compiler": compiler_metadata(build),
        "cmake_version": subprocess.check_output(["cmake", "--version"], text=True).splitlines()[0],
        "build_type": "Release",
        "cmake": configuration,
        "binary_format": "PE-no-PDB" if platform.system() == "Windows" else "ELF-unstripped",
        "cache_state": cache_state,
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "protocol": "core-resources-v1",
    }


def write_report(path: Path, report: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
