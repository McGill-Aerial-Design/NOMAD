# SPDX-License-Identifier: Apache-2.0
"""Comparable, path-free metadata for production-core resource evidence."""

from __future__ import annotations

import hashlib
import json
import os
import platform
import re
import subprocess
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def git_sha(directory: Path) -> str:
    return subprocess.check_output(["git", "-C", str(directory), "rev-parse", "HEAD"], text=True).strip()


def source_identity(directory: Path) -> dict[str, str]:
    status = subprocess.check_output(
        ["git", "-C", str(directory), "status", "--porcelain", "--untracked-files=normal", "--ignore-submodules=none"],
        text=True,
    )
    nested = subprocess.check_output(
        [
            "git",
            "-C",
            str(directory),
            "submodule",
            "foreach",
            "--recursive",
            "--quiet",
            "git status --porcelain --untracked-files=normal --ignore-submodules=none",
        ],
        text=True,
    )
    if status.strip() or nested.strip():
        raise ValueError("resource evidence requires a clean NOMAD checkout and recursive submodules")
    return {"nomad_sha": git_sha(directory), "mavsdk_sha": git_sha(directory / "third_party/MAVSDK")}


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
    identity = source_identity(ROOT)
    if Path(cache["NOMAD_MAVSDK_SOURCE_DIR"]).resolve() != (ROOT / "third_party/MAVSDK").resolve():
        raise ValueError("resource evidence requires the pinned repository MAVSDK checkout")
    if cache.get("CMAKE_BUILD_TYPE") != "Release":
        raise ValueError("resource footprint requires a Release build")
    configuration = json.loads((build / "resource-build-config.json").read_text(encoding="utf-8"))
    configuration["plugins"] = sorted(configuration["plugins"])
    configuration.update({"generator": cache["CMAKE_GENERATOR"], "nomad_build_testing": cache["BUILD_TESTING"] == "ON"})
    flags = {
        key: cache.get(key, "")
        for key in (
            "CMAKE_CXX_FLAGS",
            "CMAKE_CXX_FLAGS_RELEASE",
            "CMAKE_EXE_LINKER_FLAGS",
            "CMAKE_EXE_LINKER_FLAGS_RELEASE",
            "CPACK_STRIP_FILES",
        )
    }
    configuration["flags_sha256"] = hashlib.sha256(json.dumps(flags, sort_keys=True).encode()).hexdigest()
    return {
        **identity,
        "source_clean": True,
        "os": platform.system(),
        "os_release": platform.release(),
        "architecture": platform.machine().lower().replace("amd64", "x86_64"),
        "compiler": compiler_metadata(build),
        "cmake_version": subprocess.check_output(["cmake", "--version"], text=True).splitlines()[0],
        "build_type": "Release",
        "cmake": configuration,
        "binary_format": "PE-no-PDB" if platform.system() == "Windows" else "ELF-unstripped",
        "cache_state": cache_state,
        "pixi_cache": "enabled; hit unknown" if os.environ.get("GITHUB_ACTIONS") else "unknown",
        "dependency_download_cache": "unknown",
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "protocol": "core-resources-v1",
    }


def failure_metadata(cache_state: str) -> dict:
    return {
        "nomad_sha": git_sha(ROOT),
        "mavsdk_sha": git_sha(ROOT / "third_party/MAVSDK"),
        "os": platform.system(),
        "os_release": platform.release(),
        "architecture": platform.machine(),
        "compiler": {"id": "unknown", "version": "unknown"},
        "cmake_version": "unknown",
        "build_type": "Release requested; configure failed",
        "cmake": {"configuration_status": "unavailable"},
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "cache_state": cache_state,
        "binary_format": "unmeasured",
        "protocol": "core-resources-v1",
    }


def write_report(path: Path, report: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
