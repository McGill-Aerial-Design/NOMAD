# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Exercise production package registration without compiling the aircraft runtime."""

from __future__ import annotations

import json
import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
pytestmark = pytest.mark.skipif(not shutil.which("cmake") or not shutil.which("cpack"), reason="CMake/CPack required")


@pytest.fixture
def project(tmp_path: Path) -> Path:
    source = tmp_path / "package policy with spaces"
    source.mkdir()
    module = (ROOT / "cmake/CorePackage.cmake").as_posix()
    (source / "CMakeLists.txt").write_text(
        "cmake_minimum_required(VERSION 3.22.1)\n"
        "project(package_policy VERSION 0.0.0 LANGUAGES NONE)\n"
        'set(CMAKE_SYSTEM_NAME "${TEST_SYSTEM}")\n'
        'set(CMAKE_SYSTEM_PROCESSOR "${TEST_PROCESSOR}")\n'
        'set(CMAKE_SIZEOF_VOID_P "${TEST_POINTER_BYTES}")\n'
        'if(TEST_SYSTEM STREQUAL "Windows")\n  set(WIN32 TRUE)\nelse()\n  set(WIN32 FALSE)\nendif()\n'
        'set(NOMAD_RELEASE_VERSION "dev-package-policy")\n'
        'set(NOMAD_COMPONENT_VERSION "0.0.0-policy")\n'
        f'set(NOMAD_SOURCE_SHA "{"a" * 40}")\n'
        f'set(NOMAD_MAVSDK_SHA "{"b" * 40}")\n'
        "set(NOMAD_OFFICIAL false)\nset(NOMAD_SOURCE_DIRTY false)\n"
        "add_library(nomad INTERFACE)\nadd_library(nomad-runtime INTERFACE)\n"
        f'include("{module}")\n'
        'file(WRITE "${CMAKE_BINARY_DIR}/development-ready.txt" "configured")\n',
        encoding="utf-8",
    )
    return source


def configure(project: Path, system: str, processor: str, pointer_bytes: int = 8, *options: str) -> Path:
    build = project / "build"
    result = subprocess.run(
        [
            "cmake",
            "-S",
            str(project),
            "-B",
            str(build),
            f"-DTEST_SYSTEM={system}",
            f"-DTEST_PROCESSOR={processor}",
            f"-DTEST_POINTER_BYTES={pointer_bytes}",
            *options,
        ],
        capture_output=True,
        text=True,
        timeout=30,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert (build / "development-ready.txt").read_text(encoding="utf-8") == "configured"
    return build


def assert_release_rejected(build: Path, reason: str) -> None:
    identity = json.loads((build / "package-identity.json").read_text(encoding="utf-8"))
    assert identity.get("platform") not in {"windows", "linux"}, "unsupported target retained a release identity"
    stage = build / "stage"
    archives = {path.name: path.read_bytes() for path in build.glob("nomad-core-*.zip")}
    for command in [
        ["cmake", "--install", str(build), "--prefix", str(stage), "--component", "nomad_runtime"],
        ["cpack", "--config", str(build / "CPackConfig.cmake"), "-G", "ZIP"],
    ]:
        result = subprocess.run(command, capture_output=True, text=True, timeout=30)
        assert result.returncode != 0, "unsupported release operation succeeded"
        assert reason in result.stdout + result.stderr, result.stdout + result.stderr
    assert not stage.exists() or not any(stage.rglob("*")), "rejected installation wrote staged payload"
    assert {path.name: path.read_bytes() for path in build.glob("nomad-core-*.zip")} == archives, (
        "rejected packaging created or changed a core archive"
    )


@pytest.mark.parametrize(
    "system,processor,label,suffix",
    [
        ("Windows", "AMD64", "windows", ".exe"),
        ("Linux", "x86_64", "linux", ""),
    ],
)
def test_reviewed_platforms_keep_their_identity(project, system, processor, label, suffix):
    build = configure(project, system, processor)
    identity = json.loads((build / "package-identity.json").read_text(encoding="utf-8"))
    assert identity["platform"] == label and identity["architecture"] == "x86_64"
    assert f"bin/nomad{suffix}" in identity["required_files"]
    assert f"bin/nomad-runtime{suffix}" in identity["required_files"]


@pytest.mark.parametrize(
    "system,processor,pointer_bytes",
    [
        ("Darwin", "x86_64", 8),
        ("Darwin", "arm64", 8),
        ("FreeBSD", "x86_64", 8),
        ("Windows", "ARM64", 8),
        ("Linux", "aarch64", 8),
        ("Windows", "x86", 4),
    ],
)
def test_other_targets_keep_development_and_reject_release(project, system, processor, pointer_bytes):
    build = configure(project, system, processor, pointer_bytes)
    assert_release_rejected(build, "Reviewed NOMAD release packages require Windows or Linux x86-64")


def test_reconfiguration_cannot_reuse_a_stale_release_identity(project):
    build = configure(project, "Windows", "AMD64")
    assert json.loads((build / "package-identity.json").read_text(encoding="utf-8"))["platform"] == "windows"
    (build / "nomad-core-earlier.zip").write_bytes(b"preserved earlier artifact")
    configure(project, "Darwin", "arm64")
    assert_release_rejected(build, "Reviewed NOMAD release packages require Windows or Linux x86-64")


@pytest.mark.parametrize("option", ["-DNOMAD_SOURCE_ARCHIVE=true", "-DNOMAD_FIXTURE_VERSION=0.0.0-fixture.A"])
def test_fixture_and_source_archive_install_prohibition_remains(project, option):
    build = configure(project, "Windows", "AMD64", 8, option)
    stage = build / "stage"
    result = subprocess.run(
        ["cmake", "--install", str(build), "--prefix", str(stage), "--component", "nomad_runtime"],
        capture_output=True,
        text=True,
        timeout=30,
    )
    assert result.returncode != 0
    assert "Fixture/source-archive runtimes cannot be installed or packaged" in result.stdout + result.stderr
    assert not stage.exists() or not any(stage.rglob("*"))
