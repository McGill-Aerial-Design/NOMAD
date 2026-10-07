# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Structural checks for the post-migration setup entrypoints."""

from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SETUP_FILES = (
    ROOT / "scripts/setup/bootstrap.sh",
    ROOT / "scripts/setup/bootstrap.ps1",
    ROOT / "scripts/setup/setup_jetson.sh",
    ROOT / "scripts/setup/setup_service.sh",
)
OPTIONAL_AUTOSTART_FLAGS = (
    "NOMAD_AUTOSTART_MEDIAMTX",
    "NOMAD_AUTOSTART_ISAAC_ROS_CONTAINER",
    "NOMAD_AUTOSTART_ROS_VEHICLE",
    "NOMAD_AUTOSTART_VIDEO_BRIDGE",
)


def read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def test_setup_entrypoints_have_no_deleted_python_service_wiring() -> None:
    retired_markers = (
        "edge_core",
        "edge core",
        "nomad-edge-core",
        "fastapi",
        "uvicorn",
        "venv/bin",
        "pip install",
        ":8000",
        "/health",
    )

    for path in SETUP_FILES:
        content = read(path).lower()
        for marker in retired_markers:
            assert marker not in content, f"{path.relative_to(ROOT)} retains {marker!r}"


def test_bootstraps_build_cpp_core_and_use_profile_manager() -> None:
    for path in SETUP_FILES[:2]:
        content = read(path)
        assert "pre-commit install" in content
        assert "build-core" in content
        assert "profile-list" in content


def test_jetson_setup_builds_core_and_uses_retained_service_target() -> None:
    content = read(ROOT / "scripts/setup/setup_jetson.sh")

    assert "build-essential" in content
    assert "git submodule update --init --recursive" in content
    assert "git pull --ff-only" in content
    assert "cmake --build" in content
    assert 'scripts/profile.py" validate' in content
    assert "infra/systemd/install.sh" in content
    assert "systemctl start nomad.target" in content
    assert "systemctl disable --now mavlink-router.service" in content
    assert "systemctl enable mavlink-router" not in content
    assert "8554" in content
    assert "14560" in content
    assert "14550" in content


def test_service_setup_validates_profile_and_rejects_unqualified_optional_workloads() -> None:
    content = read(ROOT / "scripts/setup/setup_service.sh")

    assert 'scripts/profile.py" validate' in content
    assert "infra/systemd/install.sh" in content
    assert "Unavailable:" in content
    for flag in OPTIONAL_AUTOSTART_FLAGS:
        assert flag in content
    assert "systemctl start nomad.target" in content
    assert "nomad status" in content


def test_setup_does_not_mutate_runtime_config_directly() -> None:
    for path in SETUP_FILES[2:]:
        content = read(path)
        assert "sed -i" not in content
        assert ".bak." not in content
