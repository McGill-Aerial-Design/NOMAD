# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Migrate profile-owned settings in Mission Planner's local config."""

from __future__ import annotations

import json
import os
from pathlib import Path

if __package__:
    from .profile_application import remove_temporary, stage_file
else:
    from profile_application import remove_temporary, stage_file

_RETIRED_MP_FIELDS = (
    "IntegratedFlightMode",
    "SprayTargetCameraRangeM",
    "SprayRangeToleranceM",
    "SprayTriggerMaxDistanceM",
    "SprayAimPixelX",
    "SprayAimPixelY",
    "SprayAimTolerancePx",
    "SprayServoFireAngleDeg",
    "SprayForwardGain",
    "SprayLateralGain",
    "SprayAltitudeGain",
    "SprayYawGain",
    "SprayUseYawAlignment",
    "SprayMaxForwardSpeedMps",
    "SprayMaxLateralSpeedMps",
    "SprayMaxAltitudeSpeedMps",
    "SprayMaxYawRateRadps",
    "SprayLockHoldMs",
    "SprayAlignTimeoutS",
    "RouterLinks",
    "RouterConsumers",
    "RouterEnabled",
    "RouterMode",
    "RadioMasterConnectionType",
    "RadioMasterPort",
    "RadioMasterComPort",
    "RadioMasterTcpHost",
    "RadioMasterBaudRate",
    "LteMavlinkPort",
    "LteRemoteHost",
    "LteRemotePort",
    "AutoFailoverEnabled",
    "PreferredMavlinkLink",
    "AutoReconnectToPreferred",
    "PreferredLinkReconnectDelay",
    "MavlinkHeartbeatTimeout",
    "RouterBindAddress",
    "RouterDedupEnabled",
    "ManagementBindAddress",
    "JetsonApiKey",
    "JetsonIP",
    "JetsonPort",
    "CoreExePath",
    "CoreClientMode",
    "CoreMavlinkEndpoint",
)


def _config_path() -> Path | None:
    override = os.environ.get("NOMAD_MP_CONFIG")
    if override:
        return Path(override)
    local = os.environ.get("LOCALAPPDATA")
    if not local:
        return None
    return Path(local) / "Mission Planner" / "plugins" / "nomad_config.json"


def _apply_profile_settings(config: dict[str, object], name: str, env: dict[str, str]) -> None:
    for key in ("Payloads", "Actuators"):
        if key in config:
            if config[key] != []:
                raise ValueError(f"{key} must be migrated to runtime configuration before loading a profile")
            config.pop(key)
    if config.get("SerialJoystickEnabled") is True:
        raise ValueError("Enabled virtual-gamepad input requires explicit USB HID review before loading a profile")
    for key in list(config):
        if key.startswith("SerialJoystick"):
            config.pop(key)
    config.pop("CoreApiKey", None)
    legacy_mode = config.get("RouterMode")
    if legacy_mode is not None and (not isinstance(legacy_mode, str) or legacy_mode.strip().lower() != "standalone"):
        raise ValueError("RouterMode is unsupported; run only the standalone ground router")
    for key in ("RouterBindAddress", "ManagementBindAddress"):
        address = config.get(key)
        if address not in (None, "", "127.0.0.1"):
            raise ValueError(f"{key} must be 127.0.0.1; the standalone router is loopback-only")

    if "DualLinkEnabled" not in config and "RouterEnabled" in config:
        config["DualLinkEnabled"] = config["RouterEnabled"]

    for field in _RETIRED_MP_FIELDS:
        config.pop(field, None)

    value = env.get("NOMAD_VIDEO_RTSP_URL", "").strip()
    if value:
        config["VideoUrl"] = value
    else:
        config.pop("VideoUrl", None)
    config["ActiveProfile"] = name


def prepare_config(name: str, env: dict[str, str]) -> tuple[Path, bytes] | None:
    """Read and validate MP settings without changing the saved config."""
    path = _config_path()
    if path is None:
        return

    config: dict[str, object] = {}
    if path.exists():
        try:
            loaded = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, UnicodeError, json.JSONDecodeError) as exc:
            raise ValueError("Mission Planner config is unreadable or malformed") from exc
        if not isinstance(loaded, dict):
            raise ValueError("Mission Planner config is not a JSON object")
        config = loaded

    _apply_profile_settings(config, name, env)
    return path, json.dumps(config, indent=2).encode("utf-8")


def sync_config(name: str, env: dict[str, str]) -> str:
    """Apply only MP settings; propagate failures to the caller."""
    prepared = prepare_config(name, env)
    if prepared is None:
        print("[SKIPPED] mission_planner: config path unavailable; set NOMAD_MP_CONFIG to sync")
        return "skipped"
    path, content = prepared
    temporary = stage_file(path, content)
    try:
        temporary.replace(path)
    finally:
        remove_temporary(temporary)
    print("[APPLIED] mission_planner: profile settings synced")
    return "applied"
