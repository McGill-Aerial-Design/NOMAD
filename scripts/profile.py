# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""
NOMAD Configuration Profile Manager (cross-platform).

Provides load / save / list / show / diff / edit for configuration profiles.
Each profile is a complete .env file in config/profiles/ that can be loaded
into config/nomad.env (the gitignored runtime config).

On `load`, the profile's API key / endpoint are also synced into the Mission
Planner plugin config (nomad_config.json) along with an ActiveProfile marker,
so switching profiles also switches the GCS settings and the in-app profile
indicator. Set NOMAD_MP_CONFIG to override the plugin config path.

Usage:
  python scripts/profile.py load <name>
  python scripts/profile.py save <name>
  python scripts/profile.py list
  python scripts/profile.py show
  python scripts/profile.py diff <name>
  python scripts/profile.py edit
"""

from __future__ import annotations

import os
import shutil
import subprocess
import sys
from datetime import datetime
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
PROFILES_DIR = REPO_ROOT / "config" / "profiles"
ENV_FILE = REPO_ROOT / "config" / "nomad.env"

PROFILES = {
    "drone": "Real Jetson + ZED2i + ArduPilot flight controller (production)",
    "dev": "Minimal Edge Core only (API dev / CI)",
}


def _key_settings(path: Path) -> dict[str, str]:
    keys = [
        "NOMAD_PROFILE",
        "NOMAD_PROFILE_DESCRIPTION",
        "NOMAD_SIM_MODE",
        "NOMAD_ENABLE_SERVOS",
        "NOMAD_ALLOW_INSECURE_REMOTE",
    ]
    result: dict[str, str] = {}
    if not path.exists():
        return result
    for line in path.read_text(encoding="utf-8").splitlines():
        stripped = line.strip()
        if not stripped or stripped.startswith("#"):
            continue
        if "=" not in stripped:
            continue
        k, _, v = stripped.partition("=")
        k = k.strip()
        if k in keys:
            result[k] = v.strip().strip('"')
    return result


def _parse_env(path: Path) -> dict[str, str]:
    """Return all KEY=VALUE pairs from an env file."""
    env: dict[str, str] = {}
    if not path.exists():
        return env
    for line in path.read_text(encoding="utf-8").splitlines():
        s = line.strip()
        if not s or s.startswith("#") or "=" not in s:
            continue
        k, _, v = s.partition("=")
        env[k.strip()] = v.strip().strip('"')
    return env


def _mp_config_path() -> Path | None:
    """Resolve the Mission Planner plugin config path (nomad_config.json).

    Honors NOMAD_MP_CONFIG override; otherwise uses the Windows LOCALAPPDATA
    location the plugin reads. Returns None when it cannot be determined.
    """
    override = os.environ.get("NOMAD_MP_CONFIG")
    if override:
        return Path(override)
    local = os.environ.get("LOCALAPPDATA")
    if not local:
        return None
    return Path(local) / "Mission Planner" / "plugins" / "nomad_config.json"


def _host_from_url(url: str | None) -> str | None:
    if not url:
        return None
    host = url.split("://")[-1].split("/")[0].split(":")[0]
    return host or None


def sync_mission_planner(name: str, env: dict[str, str]) -> None:
    """Merge profile-controlled settings into the Mission Planner plugin config
    so switching a profile also switches the GCS API key / endpoint and the
    active-profile indicator. Other plugin settings are preserved."""
    import json

    path = _mp_config_path()
    if path is None:
        print("[INFO] Mission Planner config path unknown (set NOMAD_MP_CONFIG to sync); skipped MP sync")
        return

    cfg: dict[str, object] = {}
    if path.exists():
        try:
            cfg = json.loads(path.read_text(encoding="utf-8")) or {}
        except Exception:
            cfg = {}

    if "NOMAD_API_KEY" in env:
        cfg["JetsonApiKey"] = env["NOMAD_API_KEY"]
    port = env.get("NOMAD_API_PORT", "")
    if port.isdigit():
        cfg["JetsonPort"] = int(port)
    host = _host_from_url(env.get("NOMAD_API_URL"))
    if host and host != "0.0.0.0":
        cfg["JetsonIP"] = "127.0.0.1" if host == "localhost" else host
    cfg["ActiveProfile"] = name

    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        tmp = path.with_suffix(".json.tmp")
        tmp.write_text(json.dumps(cfg, indent=2), encoding="utf-8")
        tmp.replace(path)
        print(f"[OK] Synced Mission Planner config (profile: {name}) -> {path}")
    except Exception as e:  # noqa: BLE001
        print(f"[WARN] Could not write Mission Planner config: {e}")


def cmd_list() -> None:
    print("Available profiles:")
    print(f"{'PROFILE':<20} {'SIM MODE':<12} {'DESCRIPTION'}")
    print(f"{'-------':<20} {'--------':<12} {'-----------'}")
    for f in sorted(PROFILES_DIR.glob("*.env")):
        name = f.stem
        settings = _key_settings(f)
        desc = settings.get("NOMAD_PROFILE_DESCRIPTION", PROFILES.get(name, ""))
        sim = settings.get("NOMAD_SIM_MODE", "false")
        sim_label = "sim" if sim.lower() in ("true", "1", "yes") else "hw"
        print(f"{name:<20} {sim_label:<12} {desc}")


def _print_load_summary(settings: dict) -> None:
    desc = settings.get("NOMAD_PROFILE_DESCRIPTION", "")
    if desc:
        print(f"      {desc}")

    sim = settings.get("NOMAD_SIM_MODE", "false")
    servos = settings.get("NOMAD_ENABLE_SERVOS", "false")
    print()
    print("Key settings:")
    print(f"  NOMAD_SIM_MODE      = {sim}")
    print(f"  NOMAD_ENABLE_SERVOS = {servos}")


def _warn_user_placeholder() -> None:
    if "/home/USER/" not in ENV_FILE.read_text(encoding="utf-8"):
        return
    import getpass

    user = getpass.getuser()
    print()
    print("[WARN] Paths contain USER placeholder. Fix with:")
    print(f"  Replace /home/USER/ with /home/{user}/ in {ENV_FILE}")


def _print_next_steps(settings: dict) -> None:
    sim = settings.get("NOMAD_SIM_MODE", "false")
    print()
    print("Next steps:")
    if sim.lower() in ("true", "1", "yes"):
        print("  1. Edit paths in config/nomad.env if needed")
        print("  2. Run the hardware-free dev stack:  pixi run dev   (or pixi run dev-up)")
    else:
        print("  1. Edit paths and auth tokens in config/nomad.env")
        print("  2. Deploy to Jetson:                 nomad start all")


def cmd_load(name: str) -> None:
    src = PROFILES_DIR / f"{name}.env"
    if not src.exists():
        print(f"[FAIL] Profile not found: {src}")
        print("Available profiles:")
        for f in sorted(PROFILES_DIR.glob("*.env")):
            print(f"  {f.stem}")
        sys.exit(1)

    if ENV_FILE.exists():
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        backup = ENV_FILE.parent / f"nomad.env.bak.{ts}"
        shutil.copy2(ENV_FILE, backup)
        print(f"[INFO] Backed up current config to {backup.name}")

    shutil.copy2(src, ENV_FILE)
    print(f"[OK] Loaded profile: {name}")

    # Keep the Mission Planner plugin in sync (API key, endpoint, indicator).
    sync_mission_planner(name, _parse_env(ENV_FILE))

    settings = _key_settings(src)
    _print_load_summary(settings)
    _warn_user_placeholder()
    _print_next_steps(settings)


def cmd_save(name: str) -> None:
    if not ENV_FILE.exists():
        print(f"[FAIL] No current config found at {ENV_FILE}")
        print("Load a profile first: python scripts/profile.py load <name>")
        sys.exit(1)

    dest = PROFILES_DIR / f"{name}.env"
    if dest.exists():
        answer = input(f"Profile '{name}' already exists. Overwrite? [y/N] ").strip().lower()
        if answer != "y":
            print("[INFO] Aborted")
            return

    lines = ENV_FILE.read_text(encoding="utf-8").splitlines()
    filtered = [
        line
        for line in lines
        if not line.startswith("NOMAD_PROFILE=") and not line.startswith("NOMAD_PROFILE_DESCRIPTION=")
    ]

    header = [
        f"# Saved by nomad-profile on {datetime.now().isoformat()}",
        f"NOMAD_PROFILE={name}",
        f'NOMAD_PROFILE_DESCRIPTION="Saved from current config on {datetime.now():%Y-%m-%d}"',
        "",
    ]
    dest.write_text("\n".join(header + filtered) + "\n", encoding="utf-8")
    print(f"[OK] Saved current config as profile: {name}")
    print(f"     -> {dest}")


def cmd_show() -> None:
    if not ENV_FILE.exists():
        print("[WARN] No active config (config/nomad.env does not exist)")
        print("Load a profile: python scripts/profile.py load <name>")
        sys.exit(1)

    settings = _key_settings(ENV_FILE)
    profile = settings.get("NOMAD_PROFILE", "unknown")
    desc = settings.get("NOMAD_PROFILE_DESCRIPTION", "No description")
    sim = settings.get("NOMAD_SIM_MODE", "false")

    print(f"Active profile:   {profile}")
    print(f"Description:      {desc}")
    print(f"Sim mode:         {sim}")
    print(f"Config file:      {ENV_FILE}")


def cmd_diff(name: str) -> None:
    src = PROFILES_DIR / f"{name}.env"
    if not src.exists():
        print(f"[FAIL] Profile not found: {src}")
        sys.exit(1)
    if not ENV_FILE.exists():
        print("[FAIL] No current config to diff against")
        sys.exit(1)

    try:
        result = subprocess.run(
            ["diff", "-u", str(ENV_FILE), str(src)],
            capture_output=True,
            text=True,
            timeout=5,
        )
        print(result.stdout)
    except FileNotFoundError:
        lines_a = ENV_FILE.read_text(encoding="utf-8").splitlines()
        lines_b = src.read_text(encoding="utf-8").splitlines()
        import difflib

        for line in difflib.unified_diff(lines_a, lines_b, fromfile="current", tofile=name, lineterm=""):
            print(line)


def cmd_edit() -> None:
    if not ENV_FILE.exists():
        print("[WARN] No active config. Loading 'dev' profile first.")
        cmd_load("dev")

    editor = os.environ.get("EDITOR", "notepad" if sys.platform == "win32" else "nano")
    print(f"[INFO] Opening {ENV_FILE} with {editor}")
    subprocess.run([editor, str(ENV_FILE)])


def main() -> None:
    if len(sys.argv) < 2:
        print("Usage: python scripts/profile.py <load|save|list|show|diff|edit> [name]")
        print()
        print("Commands:")
        print("  load <name>  Load a profile (sim, drone, dev, or custom)")
        print("  save <name>  Save current config as a new profile")
        print("  list         List available profiles")
        print("  show         Show the active profile")
        print("  diff <name>  Diff a profile against current config")
        print("  edit         Open the current config in $EDITOR")
        sys.exit(0)

    action = sys.argv[1]

    if action == "list":
        cmd_list()
    elif action == "load":
        if len(sys.argv) < 3:
            print("[FAIL] Usage: python scripts/profile.py load <name>")
            sys.exit(1)
        cmd_load(sys.argv[2])
    elif action == "save":
        if len(sys.argv) < 3:
            print("[FAIL] Usage: python scripts/profile.py save <name>")
            sys.exit(1)
        cmd_save(sys.argv[2])
    elif action == "show":
        cmd_show()
    elif action == "diff":
        if len(sys.argv) < 3:
            print("[FAIL] Usage: python scripts/profile.py diff <name>")
            sys.exit(1)
        cmd_diff(sys.argv[2])
    elif action == "edit":
        cmd_edit()
    else:
        print(f"[FAIL] Unknown command: {action}")
        sys.exit(1)


if __name__ == "__main__":
    main()
