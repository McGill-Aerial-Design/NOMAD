# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Unit tests for the non-deploying core package verifier."""

from __future__ import annotations

import os
import subprocess
import zipfile
from pathlib import Path

import pytest

from scripts.dev import verify_core_package


def make_install_tree(root: Path) -> None:
    (root / "bin").mkdir(parents=True)
    (root / "bin" / "nomad.exe").write_text("placeholder", encoding="utf-8")
    (root / "bin" / "nomad-runtime.exe").write_text("placeholder", encoding="utf-8")
    for relative in verify_core_package.REQUIRED_FILES:
        path = root / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("placeholder", encoding="utf-8")
    licenses = root / "share/nomad/licenses/mavsdk-phase-a"
    licenses.mkdir(parents=True, exist_ok=True)
    (licenses / "example.txt").write_text("license", encoding="utf-8")


def test_valid_install_tree_has_no_content_errors(tmp_path: Path) -> None:
    make_install_tree(tmp_path)

    assert verify_core_package.validate_install_root(tmp_path) == []


@pytest.mark.parametrize("name", ["config/actuators.example.json", "lifecycle/migrate_actuators.py"])
def test_package_requires_actuator_provisioning_assets(tmp_path: Path, name: str) -> None:
    make_install_tree(tmp_path)
    relative = Path("share/nomad") / name
    (tmp_path / relative).unlink()

    assert f"missing {relative}" in verify_core_package.validate_install_root(tmp_path)


def test_install_tree_rejects_live_configuration(tmp_path: Path) -> None:
    make_install_tree(tmp_path)
    live_config = tmp_path / "share/nomad/config/nomad.env"
    live_config.write_text("NOMAD_API_KEY=secret", encoding="utf-8")

    errors = verify_core_package.validate_install_root(tmp_path)

    assert "package contains live config/nomad.env" in errors


def test_install_tree_rejects_mavsdk_development_payload(tmp_path: Path) -> None:
    make_install_tree(tmp_path)
    (tmp_path / "include/mavsdk").mkdir(parents=True)
    (tmp_path / "lib/mavsdk.lib").parent.mkdir(parents=True)

    errors = verify_core_package.validate_install_root(tmp_path)

    assert "package contains MAVSDK development headers" in errors
    assert "package contains development libraries" in errors


def test_install_tree_rejects_qualification_driver(tmp_path: Path) -> None:
    make_install_tree(tmp_path)
    (tmp_path / "bin" / "nomad-qualification.exe").write_text("placeholder", encoding="utf-8")

    errors = verify_core_package.validate_install_root(tmp_path)

    assert "package contains non-installed qualification driver" in errors


def test_install_tree_rejects_unexpected_bin_files(tmp_path: Path) -> None:
    make_install_tree(tmp_path)
    (tmp_path / "bin" / "qualification-probe.exe").write_text("placeholder", encoding="utf-8")

    errors = verify_core_package.validate_install_root(tmp_path)

    assert "package contains unexpected bin files: qualification-probe.exe" in errors


def test_find_install_root_accepts_cpack_top_level_directory(tmp_path: Path) -> None:
    nested = tmp_path / "nomad-core-0.1.0"
    (nested / "bin").mkdir(parents=True)

    assert verify_core_package.find_install_root(tmp_path) == nested


def test_cpack_directory_selects_current_archives_and_requires_both(tmp_path: Path) -> None:
    (tmp_path / "CPackConfig.cmake").write_text(
        'set(CPACK_PACKAGE_DIRECTORY "/tmp/\u5047")\nset(CPACK_PACKAGE_FILE_NAME "nomad-core-current")\n',
        encoding="utf-8",
    )
    archives = [tmp_path / ("nomad-core-current" + suffix) for suffix in (".zip", ".tar.gz")]
    for archive in archives:
        archive.write_bytes(b"test archive")
    (tmp_path / "nomad-core-old.zip").write_bytes(b"stale archive")
    assert verify_core_package.package_inputs(tmp_path) == archives
    archives[1].unlink()
    with pytest.raises(ValueError, match="missing configured CPack archives"):
        verify_core_package.package_inputs(tmp_path)


@pytest.mark.parametrize("name", ["../outside", "bad/name", "..", ""])
def test_cpack_directory_rejects_unsafe_output_name(tmp_path: Path, name: str) -> None:
    (tmp_path / "CPackConfig.cmake").write_text(f'set(CPACK_PACKAGE_FILE_NAME "{name}")\n')
    with pytest.raises(ValueError, match="safe package filename"):
        verify_core_package.package_inputs(tmp_path)


@pytest.mark.skipif(os.name == "nt", reason="Unix executable permissions")
def test_zip_executable_runs_after_extraction(tmp_path: Path) -> None:
    archive = tmp_path / "package.zip"
    binary = zipfile.ZipInfo("bin/nomad")
    binary.create_system = 3
    binary.external_attr = 0o104755 << 16
    with zipfile.ZipFile(archive, "w") as package:
        package.writestr(binary, "#!/bin/sh\nprintf 'package executable works\\n'\n")
    root = verify_core_package.extract_archive(archive, tmp_path / "extracted")

    result = subprocess.run([root / "bin/nomad"], capture_output=True, text=True, check=True, timeout=5)

    assert result.stdout == "package executable works\n"
    assert (root / "bin/nomad").stat().st_mode & 0o7777 == 0o755


def test_cmake_keeps_direct_driver_outside_the_installed_cli() -> None:
    cmake = (Path(__file__).resolve().parents[1] / "CMakeLists.txt").read_text(encoding="utf-8")
    cli_target = cmake.split("add_executable(nomad\n", maxsplit=1)[1].split(
        "add_library(nomad_runtime_core", maxsplit=1
    )[0]
    qualification_target = cmake.split("add_executable(nomad-qualification EXCLUDE_FROM_ALL", maxsplit=1)[1].split(
        "endif()", maxsplit=1
    )[0]

    assert "src/qualification/" not in cli_target
    assert "nomad_mavsdk_connection" not in cli_target
    assert "src/qualification/main.cpp" in qualification_target
    assert "nomad_mavsdk_connection" in qualification_target
    assert "add_executable(nomad_mavsdk_connectivity_smoke EXCLUDE_FROM_ALL" in cmake
    assert "add_executable(nomad_mavsdk_authority_wire_probe EXCLUDE_FROM_ALL" in cmake
    assert "install(TARGETS nomad nomad-runtime " in cmake
