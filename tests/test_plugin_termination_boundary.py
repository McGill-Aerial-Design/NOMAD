# SPDX-License-Identifier: Apache-2.0
"""Prevent the removed ground-side termination recipe from being reintroduced.

These are source-boundary checks, not aircraft termination qualification.
"""

from pathlib import Path

ROOT = Path(__file__).resolve().parents[1] / "mission_planner" / "src"


def test_plugin_has_no_land_as_termination_recipe():
    for source in ROOT.rglob("*.cs"):
        text = source.read_text(encoding="utf-8")
        for removed in ("EmergencyLand", '"LAND_SPEED"', '"WPNAV_SPEED_DN"', "FORCED DESCENT IN"):
            assert removed not in text, f"Ground-side termination recipe returned in {source}: {removed}"
        assert "forced descent required" not in text.lower(), f"Misleading termination warning in {source}"


def test_both_activation_callers_report_unavailable():
    for name in ("Input/NomadJoystickService.Switches.cs", "Geofence/BoundaryManager.cs"):
        text = (ROOT / name).read_text(encoding="utf-8")
        assert "if (!FlightModeController.RequestTermination())" in text, name
        assert "Termination unavailable. Take manual control." in text, name
        assert "Forced descent engaged" not in text, name


def test_plugin_fence_export_cannot_write_or_disable_aircraft_fence():
    text = (ROOT / "Views/NOMADBoundaryView.MPFence.cs").read_text(encoding="utf-8")
    assert not (ROOT / "Geofence/MPFenceUploader.cs").exists(), "Direct fence writer must remain removed"
    assert "MPFenceUploader" not in text
    assert "BtnClearVehicleFence_Click" not in text
    assert "No aircraft fence was changed." in text
    assert "MapOverlayManager.ExportToMPGeoFence(" in text
    assert "MapFenceActionToParam" not in text


def test_removed_speed_settings_have_no_compatibility_path():
    for source in ROOT.rglob("*.cs"):
        text = source.read_text(encoding="utf-8")
        for removed in ("JoystickKillLandSpeedCmS", "TerminationDescentRateMps", "landSpeedCmS", "CommLossAction"):
            assert removed not in text, f"Dead termination tuning survived in {source}: {removed}"
