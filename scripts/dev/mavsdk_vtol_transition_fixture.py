# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""MAVSDK wire and ACK coverage for both qualified QuadPlane transitions."""

from __future__ import annotations

import subprocess
from pathlib import Path

from mavsdk_fixture_harness import describe, require
from mavsdk_peer import ACCEPTED, COMMAND_DO_VTOL_TRANSITION, DENIED, VehiclePeer
from pymavlink.dialects.v20 import ardupilotmega as mavlink

PINNED_QUADPLANE_PARAMETERS = {
    "Q_ENABLE": 2.0,
    "Q_FRAME_CLASS": 7.0,
    "Q_TILT_ENABLE": 1.0,
    "Q_TILT_MASK": 3.0,
    "Q_TILT_TYPE": 0.0,
    "Q_TILT_RATE_UP": 40.0,
    "Q_TILT_MAX": 45.0,
}


def case_quadplane_transition_to_vtol_is_verified(cli: Path) -> None:
    result, observed, targets = run_transition_case(cli, "transition-to-vtol", ACCEPTED)
    transition_rows = [row for row in observed if row[1] == COMMAND_DO_VTOL_TRANSITION]
    require(
        result.returncode == 0 and "transition to VTOL verified" in result.stdout,
        "QuadPlane transition to multicopter is verified from fresh state",
        describe(result, observed),
    )
    require(
        len(transition_rows) == 1
        and transition_rows[0][0] == "COMMAND_LONG"
        and transition_rows[0][3] == (3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        "back transition encodes COMMAND_LONG DO_VTOL_TRANSITION param1=MAV_VTOL_STATE_MC",
        describe(result, observed),
    )
    require(
        targets == [(COMMAND_DO_VTOL_TRANSITION, 1, 1)],
        "back transition targets the discovered autopilot system and component",
        f"commands={observed} targets={targets}",
    )

    denied, denied_commands, _denied_targets = run_transition_case(cli, "transition-to-vtol", DENIED)
    require(
        denied.returncode != 0 and "rejected by ArduPilot" in denied.stdout,
        "back transition denied ACK fails truthfully",
        describe(denied, denied_commands),
    )


def case_transition_directions_use_the_same_targeted_command_transport(cli: Path) -> None:
    for verb, initial_state, target_state in (
        ("transition-to-fixed-wing", mavlink.MAV_VTOL_STATE_MC, mavlink.MAV_VTOL_STATE_FW),
        ("transition-to-vtol", mavlink.MAV_VTOL_STATE_FW, mavlink.MAV_VTOL_STATE_MC),
    ):
        result, observed, targets = run_transition_case(
            cli,
            verb,
            ACCEPTED,
            initial_state=initial_state,
        )
        command_rows = [row for row in observed if row[1] == COMMAND_DO_VTOL_TRANSITION]
        require(
            result.returncode == 0
            and len(command_rows) == 1
            and command_rows[0][0] == "COMMAND_LONG"
            and command_rows[0][3] == (float(target_state), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
            and targets == [(COMMAND_DO_VTOL_TRANSITION, 1, 1)],
            f"{verb} uses a targeted COMMAND_LONG with the requested VTOL state",
            describe(result, observed) + f" targets={targets}",
        )


def run_transition_case(
    cli: Path,
    verb: str,
    ack_result: int,
    *,
    initial_state: int = mavlink.MAV_VTOL_STATE_FW,
):
    from mavsdk_fixture_harness import find_free_udp_port, run_cli, with_peer

    port = find_free_udp_port()
    commands = []
    targets = []
    peer_options = {
        "params": PINNED_QUADPLANE_PARAMETERS,
        "vehicle_type": mavlink.MAV_TYPE_FIXED_WING,
        "autopilot_type": mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
        "initial_mode": 10,
        "initial_armed": True,
        "vtol_state": initial_state,
        "initial_relative_altitude_m": 20.0,
        "transition_reports_intermediate": True,
        "transition_reaches_fixed_wing": True,
        "transition_reaches_multicopter": True,
    }

    def action(peer: VehiclePeer) -> subprocess.CompletedProcess:
        arguments = ("45.5017", "-73.5673", "20") if verb == "transition-to-vtol" else ()
        result = run_cli(cli, port, verb, *arguments, timeout=25)
        commands.extend(peer.commands())
        targets.extend(target for target in peer.command_targets if target[0] == COMMAND_DO_VTOL_TRANSITION)
        return result

    result = with_peer(port, 1, ack_result, action, **peer_options)
    return result, commands, targets
