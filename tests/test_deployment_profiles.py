# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Deterministic tests for NOMAD deployment profiles.

Validates the three supported profiles:
- onboard_companion
- groundstation_gpu
- groundstation_minimal

Verifies profile separation of concerns:
- compute placement
- companion presence
- perception/GPU capability
- video source
- VIO availability
- core placement and transport
- direct MAVLink connection
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest

_REPO_ROOT = Path(__file__).resolve().parent.parent
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

import scripts.profile as profile
from scripts.profile import PROFILES, PROFILES_DIR, _parse_env, normalize_mavlink_endpoint, sync_mission_planner


def test_supported_profiles_exist() -> None:
    expected_profiles = {"onboard_companion", "groundstation_gpu", "groundstation_minimal"}
    assert set(PROFILES) == expected_profiles
    assert {path.stem for path in PROFILES_DIR.glob("*.env")} == expected_profiles
    for name in expected_profiles:
        profile_file = PROFILES_DIR / f"{name}.env"
        assert profile_file.exists(), f"Missing profile file: {profile_file}"
        assert name in PROFILES, f"Profile {name} not registered in PROFILES dictionary"


def test_product_profiles_exclude_retired_service_flags() -> None:
    retired = {
        "NOMAD_AUTOSTART_EDGE_CORE",
        "NOMAD_AUTOSTART_HEALTH_MONITOR",
        "NOMAD_AUTOSTART_TIME_SYNC",
    }
    for name in PROFILES:
        env = _parse_env(PROFILES_DIR / f"{name}.env")
        assert retired.isdisjoint(env)


def test_onboard_companion_profile_separation() -> None:
    env = _parse_env(PROFILES_DIR / "onboard_companion.env")
    assert env.get("NOMAD_PROFILE") == "onboard_companion"
    assert env.get("NOMAD_COMPUTE_PLACEMENT") == "onboard"
    assert env.get("NOMAD_HAS_COMPANION") == "true"
    assert env.get("NOMAD_HAS_PERCEPTION") == "true"
    assert env.get("NOMAD_VIO_SOURCE_REQUIRED") == "true"
    assert "rtsp://" in env.get("NOMAD_VIDEO_RTSP_URL", "")
    assert env.get("NOMAD_MAVLINK_ENDPOINT") == "udpin:0.0.0.0:14550"
    assert env.get("NOMAD_API_KEY", "") == ""
    assert env.get("NOMAD_ROS_VIO_SOURCE", "") == ""
    assert env.get("NOMAD_AUTOSTART_ISAAC_ROS_CONTAINER") == "false"
    assert env.get("NOMAD_AUTOSTART_ROS_VEHICLE") == "false"
    assert env.get("NOMAD_AUTOSTART_VIDEO_BRIDGE") == "false"


def test_groundstation_gpu_profile_separation() -> None:
    env = _parse_env(PROFILES_DIR / "groundstation_gpu.env")
    assert env.get("NOMAD_PROFILE") == "groundstation_gpu"
    assert env.get("NOMAD_COMPUTE_PLACEMENT") == "groundstation"
    assert env.get("NOMAD_HAS_COMPANION") == "false"
    assert env.get("NOMAD_HAS_PERCEPTION") == "true"
    assert env.get("NOMAD_VIO_SOURCE_REQUIRED") == "true"
    assert "127.0.0.1" in env.get("NOMAD_VIDEO_RTSP_URL", "")
    assert env.get("NOMAD_CORE_SITL_PORT")
    assert env.get("NOMAD_MAVLINK_ENDPOINT") == "udpin:127.0.0.1:14601"
    assert env.get("NOMAD_API_KEY", "") == ""
    assert env.get("NOMAD_ROS_VIO_SOURCE", "") == ""
    assert env.get("NOMAD_AUTOSTART_ISAAC_ROS_CONTAINER") == "false"
    assert env.get("NOMAD_AUTOSTART_ROS_VEHICLE") == "false"
    assert env.get("NOMAD_AUTOSTART_VIDEO_BRIDGE") == "false"


def test_groundstation_minimal_profile_separation() -> None:
    env = _parse_env(PROFILES_DIR / "groundstation_minimal.env")
    assert env.get("NOMAD_PROFILE") == "groundstation_minimal"
    assert env.get("NOMAD_COMPUTE_PLACEMENT") == "groundstation"
    assert env.get("NOMAD_HAS_COMPANION") == "false"
    assert env.get("NOMAD_HAS_PERCEPTION") == "false"
    assert env.get("NOMAD_VIO_SOURCE_REQUIRED") == "false"
    assert env.get("NOMAD_VIDEO_RTSP_URL", "") == ""
    # Direct MAVLink / C++ core transport remains active
    assert env.get("NOMAD_CORE_SITL_PORT")
    assert env.get("NOMAD_MAVLINK_ENDPOINT") == "udpin:127.0.0.1:14601"
    assert env.get("NOMAD_API_KEY", "") == ""
    assert env.get("NOMAD_ROS_VIO_SOURCE", "") == ""
    assert env.get("NOMAD_AUTOSTART_ISAAC_ROS_CONTAINER") == "false"
    assert env.get("NOMAD_AUTOSTART_ROS_VEHICLE") == "false"
    assert env.get("NOMAD_AUTOSTART_VIDEO_BRIDGE") == "false"


@pytest.mark.parametrize(
    ("endpoint", "expected"),
    [
        ("127.0.0.1:14550", "udpin:127.0.0.1:14550"),
        ("udp:127.0.0.1:14550", "udpin:127.0.0.1:14550"),
        ("udpin:0.0.0.0:14550", "udpin:0.0.0.0:14550"),
        ("udpout:router.example:65535", "udpout:router.example:65535"),
    ],
)
def test_normalize_mavlink_endpoint(endpoint: str, expected: str) -> None:
    assert normalize_mavlink_endpoint(endpoint) == expected


@pytest.mark.parametrize(
    "endpoint",
    [
        "",
        "host",
        ":14550",
        "host:",
        "host:0",
        "host:65536",
        "host:not-a-port",
        "tcp:host:14550",
        "udp://host:14550",
        "udpin:host:14550:extra",
        "udp: host:14550",
        "udpin:[::1]:14550",
        "udpin:999.999.999.999:14550",
        "udpin:-router.example:14550",
        "udpin:router..example:14550",
    ],
)
def test_normalize_mavlink_endpoint_rejects_malformed_values(endpoint: str) -> None:
    with pytest.raises(ValueError):
        normalize_mavlink_endpoint(endpoint)


def test_sync_mission_planner_rejects_profile_identity_mismatch(tmp_path: Path, monkeypatch) -> None:
    cfg_file = tmp_path / "nomad_config.json"
    monkeypatch.setenv("NOMAD_MP_CONFIG", str(cfg_file))
    env = _parse_env(PROFILES_DIR / "groundstation_minimal.env")

    with pytest.raises(ValueError, match="NOMAD_PROFILE"):
        sync_mission_planner("onboard_companion", env)

    assert not cfg_file.exists()


def test_load_rejects_invalid_endpoint_before_replacing_active_config(tmp_path: Path, monkeypatch) -> None:
    profiles = tmp_path / "profiles"
    profiles.mkdir()
    source = profiles / "groundstation_minimal.env"
    source.write_text(
        "NOMAD_PROFILE=groundstation_minimal\nNOMAD_MAVLINK_ENDPOINT=tcp:vehicle:5760\n",
        encoding="utf-8",
    )
    active = tmp_path / "nomad.env"
    active.write_text("preserve=true\n", encoding="utf-8")
    monkeypatch.setattr(profile, "PROFILES_DIR", profiles)
    monkeypatch.setattr(profile, "ENV_FILE", active)

    with pytest.raises(SystemExit):
        profile.cmd_load("groundstation_minimal")

    assert active.read_text(encoding="utf-8") == "preserve=true\n"
    assert not list(tmp_path.glob("nomad.env.bak.*"))


def test_load_writes_canonical_endpoint(tmp_path: Path, monkeypatch) -> None:
    profiles = tmp_path / "profiles"
    profiles.mkdir()
    source = profiles / "groundstation_minimal.env"
    source.write_text(
        "NOMAD_PROFILE=groundstation_minimal\nNOMAD_MAVLINK_ENDPOINT=127.0.0.1:14550\n",
        encoding="utf-8",
    )
    active = tmp_path / "nomad.env"
    monkeypatch.setattr(profile, "PROFILES_DIR", profiles)
    monkeypatch.setattr(profile, "ENV_FILE", active)
    monkeypatch.setenv("NOMAD_MP_CONFIG", str(tmp_path / "nomad_config.json"))

    profile.cmd_load("groundstation_minimal")

    assert "NOMAD_MAVLINK_ENDPOINT=udpin:127.0.0.1:14550" in active.read_text(encoding="utf-8")


def test_validate_rejects_invalid_active_profile(tmp_path: Path, monkeypatch) -> None:
    active = tmp_path / "nomad.env"
    active.write_text(
        "NOMAD_PROFILE=groundstation_minimal\nNOMAD_MAVLINK_ENDPOINT=999.999.999.999:14550\n",
        encoding="utf-8",
    )
    monkeypatch.setattr(profile, "ENV_FILE", active)

    with pytest.raises(SystemExit):
        profile.cmd_validate()


def test_save_uses_template_schema_and_preserves_secret_placeholder(tmp_path: Path, monkeypatch) -> None:
    profiles = tmp_path / "profiles"
    profiles.mkdir()
    template = profiles / "groundstation_minimal.env"
    template.write_text(
        "NOMAD_PROFILE=groundstation_minimal\n"
        "NOMAD_MAVLINK_ENDPOINT=udpin:0.0.0.0:14550\n"
        "NOMAD_API_KEY=\n"
        "NOMAD_SIM_MODE=false\n",
        encoding="utf-8",
    )
    active = tmp_path / "nomad.env"
    active.write_text(
        "NOMAD_PROFILE=groundstation_minimal\n"
        "NOMAD_MAVLINK_ENDPOINT=127.0.0.1:14550\n"
        "NOMAD_API_KEY=deployment-secret\n"
        "NOMAD_SIM_MODE=true\n"
        "NOMAD_AUTOSTART_EDGE_CORE=true\n"
        "NOMAD_DEV_ONLY=unexpected\n",
        encoding="utf-8",
    )
    monkeypatch.setattr(profile, "PROFILES_DIR", profiles)
    monkeypatch.setattr(profile, "ENV_FILE", active)
    monkeypatch.setattr("builtins.input", lambda _: "y")

    profile.cmd_save("groundstation_minimal")

    saved = template.read_text(encoding="utf-8")
    assert "NOMAD_MAVLINK_ENDPOINT=udpin:127.0.0.1:14550" in saved
    assert "NOMAD_SIM_MODE=true" in saved
    assert "NOMAD_API_KEY=\n" in saved
    assert "deployment-secret" not in saved
    assert "NOMAD_AUTOSTART_EDGE_CORE" not in saved
    assert "NOMAD_DEV_ONLY" not in saved


RETIRED_MP_CONFIG_FIELDS = (
    "JetsonApiKey",
    "JetsonIP",
    "JetsonPort",
    "CoreExePath",
    "CoreClientMode",
    "CoreMavlinkEndpoint",
)


def assert_retired_mp_fields_removed(config: dict[str, object]) -> None:
    for field in RETIRED_MP_CONFIG_FIELDS:
        assert field not in config


def test_sync_mission_planner_removes_stale_profile_fields(tmp_path: Path, monkeypatch) -> None:
    cfg_file = tmp_path / "nomad_config.json"
    initial_config = {
        "CustomUserSetting": "preserved_value",
        "ActiveProfile": "initial",
        "JetsonApiKey": "retired",
        "JetsonIP": "retired",
        "JetsonPort": 8000,
        "CoreExePath": "retired-nomad-path",
        "CoreClientMode": "LegacyOneShot",
        "CoreMavlinkEndpoint": "udpin:0.0.0.0:14550",
        "CoreApiKey": "old-key",
        "VideoUrl": "old-video",
    }
    cfg_file.write_text(json.dumps(initial_config), encoding="utf-8")
    monkeypatch.setenv("NOMAD_MP_CONFIG", str(cfg_file))

    # Sync onboard_companion
    onboard_env = _parse_env(PROFILES_DIR / "onboard_companion.env")
    onboard_env["NOMAD_API_KEY"] = "operator-key"
    sync_mission_planner("onboard_companion", onboard_env)
    synced = json.loads(cfg_file.read_text(encoding="utf-8"))
    assert synced["CustomUserSetting"] == "preserved_value"
    assert synced["ActiveProfile"] == "onboard_companion"
    assert synced["IntegratedFlightMode"] is True
    assert synced["CoreApiKey"] == onboard_env["NOMAD_API_KEY"]
    assert synced["VideoUrl"] == onboard_env["NOMAD_VIDEO_RTSP_URL"]
    assert_retired_mp_fields_removed(synced)

    # Sync groundstation_minimal
    minimal_env = _parse_env(PROFILES_DIR / "groundstation_minimal.env")
    sync_mission_planner("groundstation_minimal", minimal_env)
    synced_min = json.loads(cfg_file.read_text(encoding="utf-8"))
    assert synced_min["ActiveProfile"] == "groundstation_minimal"
    assert synced_min["IntegratedFlightMode"] is True
    assert "VideoUrl" not in synced_min
    assert "CoreApiKey" not in synced_min
    assert_retired_mp_fields_removed(synced_min)
    assert synced_min["CustomUserSetting"] == "preserved_value"


def test_sync_mission_planner_clears_integrated_mode_for_nonintegrated_profile(tmp_path: Path, monkeypatch) -> None:
    cfg_file = tmp_path / "nomad_config.json"
    cfg_file.write_text(json.dumps({"IntegratedFlightMode": True}), encoding="utf-8")
    monkeypatch.setenv("NOMAD_MP_CONFIG", str(cfg_file))
    minimal_env = _parse_env(PROFILES_DIR / "groundstation_minimal.env")
    minimal_env["NOMAD_INTEGRATED_FLIGHT"] = "0"

    sync_mission_planner("groundstation_minimal", minimal_env)

    synced = json.loads(cfg_file.read_text(encoding="utf-8"))
    assert synced["IntegratedFlightMode"] is False


@pytest.mark.parametrize("content", ["not json", "[]"])
def test_sync_mission_planner_preserves_invalid_existing_config(content: str, tmp_path: Path, monkeypatch) -> None:
    cfg_file = tmp_path / "nomad_config.json"
    cfg_file.write_text(content, encoding="utf-8")
    monkeypatch.setenv("NOMAD_MP_CONFIG", str(cfg_file))

    env = _parse_env(PROFILES_DIR / "groundstation_minimal.env")
    sync_mission_planner("groundstation_minimal", env)

    assert cfg_file.read_text(encoding="utf-8") == content
