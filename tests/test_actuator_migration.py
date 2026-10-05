# SPDX-License-Identifier: Apache-2.0
"""Migration preserves exact output identity or fails without changing saved settings."""

import json
import subprocess
from pathlib import Path

import pytest

from scripts import migrate_actuators as migration

ROOT = Path(__file__).resolve().parents[1]


def legacy_config() -> dict:
    config = dict.fromkeys(migration.SWITCH_KEYS, "None")
    config.update(
        {
            "CoreClientCredential": "private-test-credential",
            "Payloads": [
                {
                    "Name": "Operator name",
                    "Kind": "Drop",
                    "Channel": 8,
                    "PwmMin": 1100,
                    "PwmMax": 1900,
                    "PwmNeutral": 1500,
                    "Reversed": True,
                },
                {"Name": "Other name", "Kind": 2, "Channel": 2, "PulseMs": 500},
            ],
            "JoystickSw1UpAction": "DropToggleP1",
            "JoystickSw3DownAction": "FireWaterPump",
            "SerialJoystickEnabled": False,
        }
    )
    return config


def test_migration_preserves_channel_endpoints_labels_and_stable_actions():
    backend, frontend = migration.convert_config(legacy_config())
    first, second = backend["actuator_configs"]
    assert (first["id"], first["channel"], first["pwm_min"], first["pwm_max"], first["safe_pwm"]) == (
        "legacy-1",
        8,
        1100,
        1900,
        1900,
    )
    assert first["reversed"] is True
    assert first["behavior"] == "ServoToggle" and first["confirmation_count"] == 3
    assert second["behavior"] == "RelayPulse" and second["channel"] == 2
    assert (second["pulse_ms"], second["confirmation_count"]) == (500, 2)
    assert frontend["JoystickSw1UpAction"] == "legacy-1:toggle"
    assert frontend["JoystickSw3DownAction"] == "legacy-2:activate"
    # Reviewed pre-migration DrivePayloadButtons read pressed[i] from buttons[i], 0..5.
    assert frontend["JoystickButtonIndices"] == [0, 1, 2, 3, 4, 5]
    assert frontend["CoreClientCredential"] == "private-test-credential"
    assert "private-test-credential" not in json.dumps(backend)
    assert "Payloads" not in frontend and "SerialJoystickEnabled" not in frontend


@pytest.mark.parametrize(
    "key,value", [("Kind", "Unknown"), ("Channel", True), ("RcChannel", 9), ("Enabled", False), ("PulseMs", -1)]
)
def test_ambiguous_output_migration_is_rejected(key, value):
    config = legacy_config()
    config["Payloads"][0][key] = value
    with pytest.raises(ValueError):
        migration.convert_config(config)


@pytest.mark.parametrize("key", ["SerialJoystickEnabled", "JoystickCameraTiltEnabled", "JoystickZedEnabled"])
def test_incompatible_physical_input_requires_visible_review(key):
    config = legacy_config()
    config[key] = True
    with pytest.raises(ValueError, match="input|axis"):
        migration.convert_config(config)


def test_missing_mapped_output_is_not_silently_remapped():
    config = legacy_config()
    config["JoystickSw1UpAction"] = "DropToggleP3"
    with pytest.raises(ValueError, match="no matching"):
        migration.convert_config(config)


def test_missing_legacy_switch_defaults_are_not_silently_disabled():
    config = legacy_config()
    config.pop("JoystickSw2UpAction")
    with pytest.raises(ValueError, match="old compiled defaults are not guessed"):
        migration.convert_config(config)


def test_conflicting_legacy_toggle_ui_and_pulse_hid_is_rejected():
    config = legacy_config()
    config["Payloads"][1]["PulseMs"] = 0
    with pytest.raises(ValueError, match="UI toggle conflicts with its HID pulse"):
        migration.convert_config(config)


def test_legacy_relay_toggle_without_pulse_shortcut_retains_behavior():
    config = legacy_config()
    config["Payloads"][1]["PulseMs"] = 0
    config["JoystickSw3DownAction"] = "None"
    backend, _ = migration.convert_config(config)
    assert backend["actuator_configs"][1]["behavior"] == "RelayToggle"


def runtime_binary() -> Path:
    for folder in (ROOT / "build/core/Debug", ROOT / "build/core", ROOT / "build/core/Release"):
        for name in ("nomad-runtime.exe", "nomad-runtime"):
            path = folder / name
            if path.is_file():
                return path
    pytest.skip("build-core supplies the native backend validator")


def test_native_validated_export_is_private_deterministic_and_keeps_original(tmp_path):
    source, backend, frontend = (tmp_path / name for name in ("legacy.json", "backend.json", "frontend.json"))
    source.write_text(json.dumps(legacy_config()), encoding="utf-8")
    original = source.read_bytes()
    migration.migrate(source, backend, frontend, runtime_binary())
    assert source.read_bytes() == original
    assert json.loads(backend.read_text()) == migration.convert_config(legacy_config())[0]
    assert "private-test-credential" not in backend.read_text()
    result = subprocess.run(
        [str(runtime_binary()), "--validate-actuators", str(backend.resolve())],
        capture_output=True,
        text=True,
        timeout=15,
        check=False,
    )
    assert result.returncode == 0, result.stderr
    assert "no vehicle connection" in result.stdout


def test_native_bound_failure_changes_neither_original_nor_outputs(tmp_path):
    config = legacy_config()
    config["Payloads"][1]["PulseMs"] = 5000
    source, backend, frontend = (tmp_path / name for name in ("legacy.json", "backend.json", "frontend.json"))
    source.write_text(json.dumps(config), encoding="utf-8")
    original = source.read_bytes()
    with pytest.raises(ValueError, match="Backend rejected"):
        migration.migrate(source, backend, frontend, runtime_binary())
    assert source.read_bytes() == original
    assert not backend.exists() and not frontend.exists()


def test_existing_destination_is_never_overwritten(tmp_path):
    source, backend, frontend = (tmp_path / name for name in ("legacy.json", "backend.json", "frontend.json"))
    source.write_text(json.dumps(legacy_config()), encoding="utf-8")
    backend.write_text("reviewed existing file", encoding="utf-8")
    with pytest.raises(ValueError, match="new files"):
        migration.migrate(source, backend, frontend, tmp_path / "unused-runtime")
    assert backend.read_text() == "reviewed existing file"
    assert not frontend.exists()
