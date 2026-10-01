# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Migrate profile-owned settings in Mission Planner's local config."""

from __future__ import annotations

import json
import os
from pathlib import Path

_RETIRED_MP_FIELDS = (
    "IntegratedFlightMode",
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
    config.pop("CoreApiKey", None)
    legacy_mode = config.get("RouterMode")
    if legacy_mode is not None and (not isinstance(legacy_mode, str) or legacy_mode.strip().lower() != "standalone"):
        raise ValueError(f"RouterMode {legacy_mode!r} is unsupported; run only the standalone ground router")
    for key in ("RouterBindAddress", "ManagementBindAddress"):
        address = config.get(key)
        if address not in (None, "", "127.0.0.1"):
            raise ValueError(f"{key} must be 127.0.0.1; the standalone router is loopback-only")

    if "DualLinkEnabled" not in config and "RouterEnabled" in config:
        config["DualLinkEnabled"] = config["RouterEnabled"]

    removed = [field for field in _RETIRED_MP_FIELDS if field in config]
    for field in _RETIRED_MP_FIELDS:
        config.pop(field, None)
    if removed:
        print("[INFO] Removed retired Mission Planner settings: " + ", ".join(removed))

    value = env.get("NOMAD_VIDEO_RTSP_URL", "").strip()
    if value:
        config["VideoUrl"] = value
    else:
        config.pop("VideoUrl", None)
    config["ActiveProfile"] = name


def sync_config(name: str, env: dict[str, str]) -> None:
    """Merge validated profile settings into Mission Planner's saved config."""
    path = _config_path()
    if path is None:
        print("[INFO] Mission Planner config path unknown (set NOMAD_MP_CONFIG to sync); skipped MP sync")
        return

    config: dict[str, object] = {}
    if path.exists():
        try:
            loaded = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            print(f"[WARN] Mission Planner config is unreadable; left unchanged: {exc}")
            return
        if not isinstance(loaded, dict):
            print("[WARN] Mission Planner config is not a JSON object; left unchanged")
            return
        config = loaded

    _apply_profile_settings(config, name, env)
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_suffix(".json.tmp")
        temporary.write_text(json.dumps(config, indent=2), encoding="utf-8")
        temporary.replace(path)
        print(f"[OK] Synced Mission Planner config (profile: {name}) -> {path}")
    except Exception as exc:  # noqa: BLE001
        print(f"[WARN] Could not write Mission Planner config: {exc}")
