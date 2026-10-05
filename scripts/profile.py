# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""
NOMAD Configuration Profile Manager (cross-platform).

Provides load / save / list / show / diff / edit for configuration profiles.
Each profile owns product settings in config/profiles/. Loading merges these
with reviewed deployment-local settings in the gitignored config/nomad.env.

On `load`, the client profile is synced into the Mission Planner plugin
config (nomad_config.json) along with an ActiveProfile marker. The qualification
inhibition flag remains in the environment and is not a Mission Planner setting.
The MAVLink endpoint stays in the runtime environment; Mission Planner connects
to the runtime over loopback IPC. Set NOMAD_MP_CONFIG to override the plugin
config path.

Usage:
  python scripts/profile.py load <name>
  python scripts/profile.py save <name>
  python scripts/profile.py list
  python scripts/profile.py show
  python scripts/profile.py diff <name>
  python scripts/profile.py edit
"""

from __future__ import annotations

import difflib
import ipaddress
import os
import re
import subprocess
import sys
from pathlib import Path

if __package__:
    from .profile_application import apply_targets
    from .profile_mission_planner import prepare_config
    from .profile_mission_planner import sync_config as _sync_mission_planner_config
    from .profile_settings import PROFILE_KEYS, get_assignment_key, prepare_environment
else:
    from profile_application import apply_targets
    from profile_mission_planner import prepare_config
    from profile_mission_planner import sync_config as _sync_mission_planner_config
    from profile_settings import PROFILE_KEYS, get_assignment_key, prepare_environment

REPO_ROOT = Path(__file__).resolve().parent.parent
PROFILES_DIR = REPO_ROOT / "config" / "profiles"
ENV_FILE = REPO_ROOT / "config" / "nomad.env"

PROFILES = {
    "onboard_companion": "Onboard companion: Jetson/SBC runs ROS 2, VIO, and video workloads",
    "groundstation_gpu": "Ground station GPU: Workstation runs ROS 2, VIO, camera, and perception locally",
    "groundstation_minimal": "Ground station minimal: Runtime IPC and standalone router, no companion or perception",
}

_ENDPOINT_PATTERN = re.compile(
    r"^(?:(?P<scheme>udp|udpin|udpout):)?(?P<host>[^:/\s]+):(?P<port>[0-9]+)$",
    re.IGNORECASE,
)
_RETIRED_PROFILE_SETTINGS = {
    "NOMAD_ENABLE_SERVOS": "servo outputs are configured as payloads; this environment flag has no consumer",
    "NOMAD_BRIDGE_MAVLINK_ENDPOINT": "aircraft transport belongs to NOMAD_MAVLINK_ENDPOINT in nomad-runtime",
    "NOMAD_CORE_SITL_PORT": "this port belongs to test tooling and is not a product-profile setting",
    "NOMAD_LTE_UDP_PORT": "physical ground links belong in the standalone router Links array",
    "NOMAD_ELRS_SERIAL": "physical ground links belong in the standalone router Links array",
    "NOMAD_ELRS_BAUD": "physical ground links belong in the standalone router Links array",
    "NOMAD_VIDEO_RTSP_PORT": "this profile alias is not consumed; configure the MediaMTX RTSP_PORT",
    "NOMAD_VIO_SOURCE_REQUIRED": "this profile setting has no runtime consumer and must be removed",
    "NOMAD_VIO_MAX_AGE_S": "this profile setting has no runtime consumer and must be removed",
}
_UNSAVED_SECRET_KEYS = {"NOMAD_API_KEY", "NOMAD_CLIENT_CREDENTIAL"}
_HOST_LABEL_PATTERN = re.compile(r"^[A-Za-z0-9](?:[A-Za-z0-9-]{0,61}[A-Za-z0-9])?$")


def _validate_endpoint_host(host: str) -> None:
    try:
        ipaddress.ip_address(host)
        return
    except ValueError:
        pass

    if re.fullmatch(r"[0-9.]+", host):
        raise ValueError("MAVLink endpoint host must be a valid IPv4 address or hostname")
    if len(host) > 253 or any(not _HOST_LABEL_PATTERN.fullmatch(label) for label in host.split(".")):
        raise ValueError("MAVLink endpoint host must be a valid IPv4 address or hostname")


def _parse_mavlink_endpoint(endpoint: str) -> tuple[str, str, int]:
    if not isinstance(endpoint, str):
        raise ValueError("MAVLink endpoint must be a string")

    match = _ENDPOINT_PATTERN.fullmatch(endpoint.strip())
    if match is None:
        raise ValueError("MAVLink endpoint must be [scheme:]host:port")

    host = match.group("host")
    port = int(match.group("port"))
    if not host or port < 1 or port > 65535:
        raise ValueError("MAVLink endpoint requires a host and port 1..65535")
    _validate_endpoint_host(host)

    scheme = (match.group("scheme") or "udpin").lower()
    if scheme == "udp":
        scheme = "udpin"
    return scheme, host, port


def validate_mavlink_endpoint(endpoint: str) -> None:
    """Raise ValueError unless endpoint is a supported MAVLink UDP address."""
    _parse_mavlink_endpoint(endpoint)


def normalize_mavlink_endpoint(endpoint: str) -> str:
    """Return an endpoint in canonical ``scheme:host:port`` form."""
    scheme, host, port = _parse_mavlink_endpoint(endpoint)
    return f"{scheme}:{host}:{port}"


def _validated_profile_env(name: str, env: dict[str, str]) -> dict[str, str]:
    if name not in PROFILES:
        raise ValueError(f"Unsupported product profile: {name}")
    if env.get("NOMAD_PROFILE") != name:
        raise ValueError(f"NOMAD_PROFILE must equal {name}")
    retired = sorted(set(env).intersection(_RETIRED_PROFILE_SETTINGS))
    if retired:
        details = "; ".join(f"{key}: {_RETIRED_PROFILE_SETTINGS[key]}" for key in retired)
        raise ValueError(f"Unsupported product-profile settings: {details}")
    normalized = dict(env)
    normalized["NOMAD_MAVLINK_ENDPOINT"] = normalize_mavlink_endpoint(env.get("NOMAD_MAVLINK_ENDPOINT", ""))
    integrated_value = env.get("NOMAD_INTEGRATED_FLIGHT", "false").strip().lower()
    if integrated_value not in {"1", "true", "yes", "0", "false", "no"}:
        raise ValueError("NOMAD_INTEGRATED_FLIGHT must be a boolean value")
    return normalized


_KEY_SETTINGS = (
    "NOMAD_PROFILE",
    "NOMAD_PROFILE_DESCRIPTION",
    "NOMAD_SIM_MODE",
    "NOMAD_ENABLE_SERVOS",
)


def read_env_text(text: str) -> dict[str, str]:
    """Return KEY=VALUE pairs from env-file text, skipping blanks and comments."""
    env: dict[str, str] = {}
    for line in text.splitlines():
        stripped = line.strip()
        if not stripped or stripped.startswith("#") or "=" not in stripped:
            continue
        key, _, value = stripped.partition("=")
        env[key.strip()] = value.strip().strip('"')
    return env


def read_env_file(path: Path) -> dict[str, str]:
    """Return KEY=VALUE pairs from an env file, or an empty map when absent."""
    if not path.exists():
        return {}
    return read_env_text(path.read_text(encoding="utf-8"))


def _key_settings(path: Path) -> dict[str, str]:
    settings = read_env_file(path)
    return {key: settings[key] for key in _KEY_SETTINGS if key in settings}


def _parse_env(path: Path) -> dict[str, str]:
    """Return all KEY=VALUE pairs from an env file."""
    return read_env_file(path)


def sync_mission_planner(name: str, env: dict[str, str]) -> str:
    """Merge profile-controlled settings into the Mission Planner plugin config."""
    return _sync_mission_planner_config(name, _validated_profile_env(name, env))


def cmd_list() -> None:
    print("Available profiles:")
    print(f"{'PROFILE':<20} {'SIM MODE':<12} {'DESCRIPTION'}")
    print(f"{'-------':<20} {'--------':<12} {'-----------'}")
    for name in sorted(PROFILES):
        f = PROFILES_DIR / f"{name}.env"
        if not f.exists():
            continue
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
    print()
    print("Key settings:")
    print(f"  NOMAD_SIM_MODE      = {sim}")


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
        print("  2. Run the hardware-free SITL stack: pixi run dev-up")
    else:
        print("  1. Edit paths and auth tokens in config/nomad.env")
        print("  2. Deploy to Jetson:                 nomad start all")


def _prepare_env_content(content: str, profile_env: dict[str, str]) -> bytes:
    current = ENV_FILE.read_text(encoding="utf-8") if ENV_FILE.exists() else ""
    defaults = (REPO_ROOT / "config" / "nomad.env.example").read_text(encoding="utf-8")
    return prepare_environment(content, current, defaults, profile_env["NOMAD_MAVLINK_ENDPOINT"])


def _apply_profile(name: str, src: Path) -> bool:
    try:
        content = src.read_text(encoding="utf-8")
        profile_env = _validated_profile_env(name, read_env_text(content))
        env_content = _prepare_env_content(content, profile_env)
    except (OSError, UnicodeError, ValueError):
        print("[FAILED] env: profile or current env is unreadable or invalid; unchanged")
        print("[SKIPPED] mission_planner: application aborted; unchanged")
        return False
    try:
        mp_config = prepare_config(name, profile_env)
    except (OSError, UnicodeError, ValueError) as exc:
        print("[SKIPPED] env: Mission Planner preflight failed; unchanged")
        print(f"[FAILED] mission_planner: {exc}")
        return False
    return apply_targets(ENV_FILE, env_content, mp_config)


def cmd_load(name: str) -> None:
    src = PROFILES_DIR / f"{name}.env"
    if name not in PROFILES or not src.exists():
        print(f"[FAIL] Profile not found: {src}")
        print("[FAILED] env: profile not found; unchanged")
        print("[SKIPPED] mission_planner: application aborted; unchanged")
        print("Available profiles:")
        for profile_name in sorted(PROFILES):
            if (PROFILES_DIR / f"{profile_name}.env").exists():
                print(f"  {profile_name}")
        sys.exit(1)

    if not _apply_profile(name, src):
        print(f"[FAILED] Profile load: {name}")
        sys.exit(1)
    print(f"[OK] Profile load completed: {name} (see target results above)")

    settings = _key_settings(src)
    _print_load_summary(settings)
    _warn_user_placeholder()
    _print_next_steps(settings)


def _format_env_value(value: str) -> str:
    if not value or re.search(r"\s|#", value):
        return f'"{value}"'
    return value


def _format_saved_profile(name: str, profile_env: dict[str, str], template: Path) -> str:
    template_text = template.read_text(encoding="utf-8")
    allowed = set(read_env_text(template_text)).intersection(PROFILE_KEYS)
    values = {key: value for key, value in profile_env.items() if key in allowed and key not in _UNSAVED_SECRET_KEYS}
    values["NOMAD_PROFILE"] = name

    result: list[str] = []
    for line in template_text.splitlines():
        key = get_assignment_key(line)
        if key and key not in PROFILE_KEYS:
            continue
        if key in values and not line.lstrip().startswith("#"):
            result.append(f"{key}={_format_env_value(values[key])}")
        else:
            result.append(line)
    return "\n".join(result) + "\n"


def cmd_save(name: str) -> None:
    if not ENV_FILE.exists():
        print(f"[FAIL] No current config found at {ENV_FILE}")
        print("Load a profile first: python scripts/profile.py load <name>")
        sys.exit(1)

    if name not in PROFILES:
        print(f"[FAIL] Unsupported product profile: {name}")
        sys.exit(1)

    try:
        profile_env = _validated_profile_env(name, _parse_env(ENV_FILE))
    except ValueError as exc:
        print(f"[FAIL] Current config is invalid: {exc}")
        sys.exit(1)

    dest = PROFILES_DIR / f"{name}.env"
    if not dest.exists():
        print(f"[FAIL] Product profile template is missing: {dest}")
        sys.exit(1)
    answer = input(f"Profile '{name}' already exists. Overwrite? [y/N] ").strip().lower()
    if answer != "y":
        print("[INFO] Aborted")
        return

    dest.write_text(_format_saved_profile(name, profile_env, dest), encoding="utf-8")
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


def cmd_validate() -> None:
    if not ENV_FILE.exists():
        print(f"[FAIL] No current config found at {ENV_FILE}")
        sys.exit(1)

    env = _parse_env(ENV_FILE)
    name = env.get("NOMAD_PROFILE", "")
    try:
        _validated_profile_env(name, env)
    except ValueError as exc:
        print(f"[FAIL] Current config is invalid: {exc}")
        sys.exit(1)
    print(f"[OK] Active product profile is valid: {name}")


def cmd_diff(name: str) -> None:
    src = PROFILES_DIR / f"{name}.env"
    if name not in PROFILES or not src.exists():
        print(f"[FAIL] Profile not found: {src}")
        sys.exit(1)
    if not ENV_FILE.exists():
        print("[FAIL] No current config to diff against")
        sys.exit(1)

    lines_a = _profile_assignment_lines(ENV_FILE)
    lines_b = _profile_assignment_lines(src)
    for line in difflib.unified_diff(lines_a, lines_b, fromfile="current profile settings", tofile=name, lineterm=""):
        print(line)


def _profile_assignment_lines(path: Path) -> list[str]:
    settings = read_env_file(path)
    return [f"{key}={_format_env_value(settings[key])}" for key in sorted(PROFILE_KEYS.intersection(settings))]


def cmd_edit() -> None:
    if not ENV_FILE.exists():
        print("[FAIL] No active config. Load one of the supported product profiles first.")
        sys.exit(1)

    editor = os.environ.get("EDITOR", "notepad" if sys.platform == "win32" else "nano")
    print(f"[INFO] Opening {ENV_FILE} with {editor}")
    subprocess.run([editor, str(ENV_FILE)])


def cmd_which() -> None:
    if not ENV_FILE.exists():
        print("[WARN] No active config file found")
        sys.exit(1)
    print(ENV_FILE)


def _print_usage() -> None:
    print("Usage: python scripts/profile.py <load|save|list|show|validate|diff|edit|which> [name]")
    print()
    print("Commands:")
    print("  load <name>  Load a supported product profile")
    print("  save <name>  Update a supported product profile from current config")
    print("  list         List available profiles")
    print("  show         Show the active profile")
    print("  validate     Validate the active product profile")
    print("  diff <name>  Diff a profile against current config")
    print("  edit         Open the current config in $EDITOR")
    print("  which        Print the active config path")


def _require_profile_name(action: str) -> str:
    if len(sys.argv) < 3:
        print(f"[FAIL] Usage: python scripts/profile.py {action} <name>")
        sys.exit(1)
    return sys.argv[2]


def main() -> None:
    if len(sys.argv) < 2:
        _print_usage()
        return

    action = sys.argv[1]

    if action == "list":
        cmd_list()
    elif action == "load":
        cmd_load(_require_profile_name(action))
    elif action == "save":
        cmd_save(_require_profile_name(action))
    elif action == "show":
        cmd_show()
    elif action == "validate":
        cmd_validate()
    elif action == "diff":
        cmd_diff(_require_profile_name(action))
    elif action == "edit":
        cmd_edit()
    elif action == "which":
        cmd_which()
    else:
        print(f"[FAIL] Unknown command: {action}")
        sys.exit(1)


if __name__ == "__main__":
    main()
