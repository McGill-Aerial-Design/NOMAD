# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Focused checks for retained service and video deployment wiring."""

import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def test_nomad_cli_and_systemd_inventory_have_no_deleted_edge_service():
    cli = read("scripts/nomad")
    installer = read("infra/systemd/install.sh")
    ros_unit = read("infra/systemd/nomad-ros-vehicle.service")
    video_unit = read("infra/systemd/nomad-video-bridge.service")

    assert "edge_core" not in cli
    assert "nomad-edge-core" not in installer
    assert "nomad-edge-core" not in ros_unit
    assert "nomad-edge-core" not in video_unit

    listed_units = re.findall(r"^\s+(nomad[-.a-z]+(?:service|target))$", installer, re.MULTILINE)
    assert listed_units
    for unit in listed_units:
        assert (ROOT / "infra" / "systemd" / unit).is_file(), unit


def test_example_config_excludes_deleted_service_owners():
    config = read("config/nomad.env.example")

    assert "NOMAD_AUTOSTART_EDGE_CORE" not in config
    assert "NOMAD_AUTOSTART_HEALTH_MONITOR" not in config
    assert "NOMAD_AUTOSTART_TIME_SYNC" not in config
    assert "NOMAD_MAVLINK_ENDPOINT=udpin:127.0.0.1:14601" in config


def test_shell_profile_command_delegates_to_validated_profile_manager():
    wrapper = read("scripts/nomad-profile")

    assert 'exec python3 "$SCRIPT_DIR/profile.py" "$@"' in wrapper
    assert 'cp "$src"' not in wrapper


def test_video_service_owns_local_process_lifecycle():
    script = read("scripts/services/video_bridge.sh")

    assert "python3 -m python.tools.simple_video_bridge" in script
    assert "video_bridge.pid" in script
    assert "/api/video" not in script
    assert "require_container" in script
    assert "video capability unavailable" in script


def test_container_and_jetson_image_retain_video_tools_without_edge_mount():
    container = read("scripts/services/isaac_ros_container.sh")
    dockerfile = read("docker/Dockerfile.jetson")

    assert "edge_core" not in container
    assert "COPY python/tools/ /opt/nomad/python/tools/" in dockerfile
    assert 'PYTHONPATH="/opt/nomad"' in dockerfile
