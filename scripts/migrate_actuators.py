# SPDX-License-Identifier: Apache-2.0
"""Export reviewed legacy MP outputs to backend settings; never overwrite the input."""

from __future__ import annotations

import argparse
import json
import os
import re
import subprocess
import tempfile
from pathlib import Path

KINDS = ("Drop", "Slider", "Relay", "Reel", "CamTilt")
SWITCH_KEYS = tuple(f"JoystickSw{number}{direction}Action" for number in (1, 2, 3) for direction in ("Up", "Down"))


def read_value(old: dict, key: str, default: object, expected: type):
    value = old.get(key, default)
    if type(value) is not expected:
        raise ValueError(f"Legacy {key} has an invalid type; review the original configuration")
    return value


def read_kind(old: dict) -> int:
    value = old.get("Kind", 0)
    if type(value) is int and 0 <= value < len(KINDS):
        return value
    if isinstance(value, str) and value in KINDS:
        return KINDS.index(value)
    raise ValueError("Unknown legacy output kind; no channel/action was remapped")


def validate_legacy_output(old: dict) -> None:
    if not read_value(old, "Enabled", True, bool):
        raise ValueError("Disabled legacy outputs require explicit review; they are not silently discarded")
    if read_value(old, "RcChannel", 0, int) != 0:
        raise ValueError("Legacy RC pass-through requires explicit review outside NOMAD software authorization")


def primary_label(kind: int, behavior: str) -> str:
    if kind == 0:
        return "Release"
    if behavior == "RelayToggle":
        return "On"
    return "Activate" if kind == 2 else "Apply"


def convert_output(old: dict, index: int, kind: int) -> dict:
    validate_legacy_output(old)
    low = read_value(old, "PwmMin", 1000, int)
    high = read_value(old, "PwmMax", 2000, int)
    neutral = read_value(old, "PwmNeutral", 1500, int)
    reversed_output = read_value(old, "Reversed", False, bool)
    pulse = read_value(old, "PulseMs", 500, int)
    if pulse < 0:
        raise ValueError("Negative legacy pulse duration requires review")
    behavior = (
        "ServoToggle",
        "ServoPosition",
        "RelayPulse" if pulse > 0 else "RelayToggle",
        "ServoBidirectional",
        "ServoPosition",
    )[kind]
    return {
        "id": f"legacy-{index + 1}",
        "name": read_value(old, "Name", "Actuator", str),
        "behavior": behavior,
        "channel": read_value(old, "Channel", 9, int),
        "pwm_min": low,
        "pwm_max": high,
        "pwm_neutral": neutral,
        "safe_pwm": (high if reversed_output else low) if kind == 0 else neutral,
        "reversed": reversed_output,
        "pulse_ms": pulse if kind == 2 and pulse > 0 else 500,
        "hold_ms": read_value(old, "HoldSafetyS", 10, int) * 1000 if kind == 3 else 1000,
        "primary_label": primary_label(kind, behavior),
        "secondary_label": "Retract" if kind == 0 else "Off" if kind == 2 else "Safe",
        "negative_label": "Out",
        "neutral_label": "Stop",
        "positive_label": "In",
        "hazardous": kind in (0, 2),
        "confirmation_count": 3 if kind == 0 else 2 if kind == 2 else 0,
        "confirmation_window_ms": 3000,
        "require_neutral": True,
    }


def convert_binding(action: str, kinds: list[int], definitions: list[dict]) -> str:
    if action in ("", "None"):
        return "None"
    match = re.fullmatch(r"(DropToggleP|ReelInP|ReelOutP)([1-3])", action, re.IGNORECASE)
    if match:
        prefix, number = match.group(1).lower(), int(match.group(2))
        kind = 0 if prefix == "droptogglep" else 3
        operation = "toggle" if kind == 0 else "positive" if prefix == "reelinp" else "negative"
    elif action.lower() == "firewaterpump":
        kind, number, operation = 2, 1, "activate"
    else:
        raise ValueError("Unknown legacy switch action; no action was remapped")
    matches = [definition for definition, old_kind in zip(definitions, kinds, strict=True) if old_kind == kind]
    if number > len(matches):
        raise ValueError("Legacy switch action has no matching configured output")
    if action.lower() == "firewaterpump" and matches[number - 1]["behavior"] == "RelayToggle":
        raise ValueError("Legacy relay UI toggle conflicts with its HID pulse action; review both actions explicitly")
    return f"{matches[number - 1]['id']}:{operation}"


def convert_config(config: dict) -> tuple[dict, dict]:
    if any(key not in config for key in SWITCH_KEYS):
        raise ValueError("Missing legacy switch settings require review; old compiled defaults are not guessed")
    termination_enabled = read_value(config, "JoystickKillSwitchEnabled", True, bool)
    termination_index = read_value(config, "JoystickTerminationButtonIndex", 6, int)
    if termination_index != 6:
        raise ValueError(
            "Legacy termination input was compiled as button 6; a different index requires explicit review"
        )
    if config.get("SerialJoystickEnabled") is True:
        raise ValueError("Virtual-gamepad input cannot be mapped to USB HID automatically; review physical inputs")
    if config.get("JoystickCameraTiltEnabled") is True or config.get("JoystickZedEnabled") is True:
        raise ValueError("Enabled relative-rate axis requires explicit review before absolute position input")
    payloads = config.get("Payloads")
    if not isinstance(payloads, list) or config.get("Actuators"):
        raise ValueError("Expected one legacy Payloads list; mixed or missing ownership requires review")
    if not all(isinstance(old, dict) for old in payloads):
        raise ValueError("Invalid legacy output entry")
    kinds = [read_kind(old) for old in payloads]
    definitions = [
        convert_output(old, index, kind) for index, (old, kind) in enumerate(zip(payloads, kinds, strict=True))
    ]
    frontend = dict(config)
    for key in SWITCH_KEYS:
        frontend[key] = convert_binding(read_value(config, key, "None", str), kinds, definitions)
    for key in list(frontend):
        if key in ("Payloads", "Actuators") or key.startswith("SerialJoystick"):
            frontend.pop(key)
    # These are the six exact button indices used by the retired direct-input switch mapper.
    frontend["JoystickButtonIndices"] = [0, 1, 2, 3, 4, 5]
    frontend["JoystickTerminationButtonIndex"] = termination_index
    frontend["JoystickKillSwitchEnabled"] = termination_enabled
    return {"actuator_configs": definitions}, frontend


def write_private(path: Path, data: dict) -> None:
    descriptor = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    with os.fdopen(descriptor, "w", encoding="utf-8", newline="\n") as stream:
        json.dump(data, stream, indent=2)
        stream.write("\n")
    if os.name == "nt":
        account = os.getlogin()
        subprocess.run(["icacls", str(path), "/setowner", account], capture_output=True, check=True)
        subprocess.run(
            ["icacls", str(path), "/inheritance:r", "/grant:r", f"{account}:(F)", "*S-1-5-18:(F)", "*S-1-5-32-544:(F)"],
            capture_output=True,
            check=True,
        )


def migrate(legacy: Path, backend: Path, frontend: Path, runtime: Path) -> None:
    if len({path.resolve() for path in (legacy, backend, frontend)}) != 3:
        raise ValueError("Original and both output paths must be different")
    if backend.exists() or frontend.exists():
        raise ValueError("Outputs must be new files; the original and existing settings are never overwritten")
    config = json.loads(legacy.read_text(encoding="utf-8"))
    if not isinstance(config, dict):
        raise ValueError("Legacy configuration must be a JSON object")
    backend_data, frontend_data = convert_config(config)
    # The production backend validates all bounds; this converter owns no operation policy.
    with tempfile.TemporaryDirectory(prefix="nomad-actuator-migration-") as directory:
        staged = Path(directory) / "actuators.json"
        write_private(staged, backend_data)
        result = subprocess.run(
            [str(runtime.resolve()), "--validate-actuators", str(staged.resolve())],
            capture_output=True,
            text=True,
            timeout=15,
            check=False,
        )
        if result.returncode:
            raise ValueError(
                "Backend rejected migrated settings; review exact values without clamping: " + result.stderr.strip()
            )
    write_private(backend, backend_data)
    write_private(frontend, frontend_data)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("legacy", type=Path)
    parser.add_argument("backend", type=Path)
    parser.add_argument("frontend", type=Path)
    parser.add_argument("--runtime", type=Path, required=True)
    args = parser.parse_args()
    try:
        migrate(args.legacy, args.backend, args.frontend, args.runtime)
    except (ValueError, OSError, subprocess.SubprocessError) as error:
        print(f"Migration rejected; preserve the original settings: {error}")
        return 1
    print("Migration exported reviewed backend settings and private frontend settings; original unchanged.")
    print("Review the output files, provision NOMAD_ACTUATORS_FILE, and explicitly select the real USB HID device.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
