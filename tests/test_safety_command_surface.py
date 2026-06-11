# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""SR-SEC-01: the MAVLink command surface contains no failsafe-disabling command.

NOMAD's safety argument for hazard H-07 is that the FC's own failsafes remain
the certified inner layer because NOMAD never writes parameters or issues a
command that could disable them. That is true by construction today; this test
makes it checkable: it scans the source of every module that owns a MAVLink
link and fails if a parameter write, force-arm magic number, or any deny-listed
MAV_CMD/parameter name appears.
"""

from __future__ import annotations

from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parent.parent

# Every Python module that holds a MAVLink connection and can transmit.
COMMAND_SURFACE_FILES = [
    "edge_core/services/mavlink/commands.py",
    "edge_core/services/mavlink/connection.py",
    "edge_core/services/mavlink/module.py",
    "edge_core/services/mavlink/__init__.py",
    "edge_core/ros_http_bridge/mavlink_velocity.py",
]

# Tokens that would let NOMAD weaken an FC failsafe. Parameter writes are
# forbidden wholesale: every FC failsafe (FS_*, FENCE_*, ARMING_*, BRD_SAFETY*)
# is a parameter, so "no param writes" covers them all.
FORBIDDEN_TOKENS = [
    "param_set_send",  # MAVLink parameter write
    "MAV_CMD_DO_SET_PARAMETER",
    "MAV_CMD_DO_FLIGHTTERMINATION",
    "MAV_CMD_DO_PARACHUTE",
    "MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN",
    "21196",  # ArduPilot force-arm/disarm magic (bypasses arming checks)
    "FS_THR_ENABLE",
    "FS_GCS_ENABLE",
    "FS_EKF_ACTION",
    "FENCE_ENABLE",
    "ARMING_CHECK",
    "BRD_SAFETY",
]


# The complete public command surface of MavlinkCommands. A method added here
# can transmit to the FC — adding one is an SC change (name the SR-* requirement
# and update this pin deliberately). Dead transmit paths were deleted on
# 2026-06-10 (gimbal, vision-estimate, obstacle-distance: zero callers).
MAVLINK_COMMANDS_SURFACE = {
    "arm_disarm",
    "send_velocity_command",
    "trigger_payload",
    "set_relay",
    "stop_velocity",
    "send_statustext",
    "set_mode",
    "takeoff",
    "land",
    "request_home_position",
    "send_global_position_target",
    "send_position_target",
}


def test_mavlink_commands_surface_is_pinned():
    from edge_core.services.mavlink.commands import MavlinkCommands

    public = {
        name for name in vars(MavlinkCommands) if not name.startswith("_") and callable(getattr(MavlinkCommands, name))
    }
    assert public == MAVLINK_COMMANDS_SURFACE, (
        f"MavlinkCommands public surface changed: added={public - MAVLINK_COMMANDS_SURFACE}, "
        f"removed={MAVLINK_COMMANDS_SURFACE - public}. Update the pin only with an SR-* justification."
    )


@pytest.mark.parametrize("rel_path", COMMAND_SURFACE_FILES)
def test_no_failsafe_disabling_commands(rel_path: str):
    source_file = REPO_ROOT / rel_path
    assert source_file.is_file(), f"command-surface file moved/renamed: {rel_path} (update this test)"
    source = source_file.read_text(encoding="utf-8")
    hits = [token for token in FORBIDDEN_TOKENS if token in source]
    assert not hits, (
        f"{rel_path} contains forbidden failsafe-affecting token(s) {hits}. "
        "NOMAD must never disable or weaken an FC failsafe (SR-SEC-01, H-07); "
        "if this is intentional it needs a requirement change reviewed by a human."
    )
