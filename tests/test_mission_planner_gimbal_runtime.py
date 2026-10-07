# SPDX-License-Identifier: Apache-2.0
"""Keep Mission Planner gimbal angles behind the typed runtime boundary."""

from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def test_gimbal_controller_has_no_direct_mavlink_angle_writer() -> None:
    """Reject a return of the old Mission Planner-owned MAVLink sender."""
    controller = (ROOT / "mission_planner/src/Control/GimbalController.cs").read_text(encoding="utf-8")
    plugin_root = ROOT / "mission_planner/src"
    source_files = [path for path in plugin_root.rglob("*.cs") if "bin" not in path.parts and "obj" not in path.parts]
    plugin_source = "\n".join(path.read_text(encoding="utf-8") for path in source_files)

    forbidden = (
        "MainV2.comPort",
        "doCommandAsync",
        "GimbalFrame",
        "BuildMountControl",
        "DO_MOUNT_CONTROL",
        "MavlinkSerialLock",
    )
    for symbol in forbidden:
        assert symbol not in controller, f"gimbal controller still contains direct-writer symbol {symbol}"
    for symbol in ("DO_MOUNT_CONTROL", "GimbalFrame", "BuildMountControl", "MavlinkSerialLock"):
        assert symbol not in plugin_source, f"Mission Planner source still contains direct-writer symbol {symbol}"

    assert "OutputController.SendGimbalTarget" in controller


def test_all_mission_planner_gimbal_inputs_share_the_controller() -> None:
    """Keep window, arrow-key, and physical stick inputs on the shared controller."""
    window = (ROOT / "mission_planner/src/Control/GimbalJoystickWindow.cs").read_text(encoding="utf-8")
    joystick = (ROOT / "mission_planner/src/Input/NomadJoystickService.cs").read_text(encoding="utf-8")
    arrow_keys = (ROOT / "mission_planner/src/Control/GimbalArrowKeyFilter.cs").read_text(encoding="utf-8")

    assert "GimbalController.ApplyStick" in window
    assert "GimbalController.RequestPitchRollTarget" in window
    assert "GimbalController.ApplyStick" in joystick
    assert "GimbalController.NudgeTarget" in arrow_keys
