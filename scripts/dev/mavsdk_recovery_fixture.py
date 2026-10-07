# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Check the bounded recovery command against a deterministic MAVSDK peer."""

from __future__ import annotations

from pathlib import Path

from mavsdk_fixture_harness import describe, find_free_udp_port, require, run_cli, run_cli_case, with_peer
from mavsdk_peer import ACCEPTED, COMMAND_DO_REPOSITION, DENIED, GLOBAL_RELATIVE_ALT_INT_FRAME, CommandRecord
from pymavlink.dialects.v20 import ardupilotmega as mavlink


def require_recovery_wire(result, observed: list[CommandRecord], targets: list[tuple[int, int, int]], point) -> None:
    diagnostic = describe(result, observed)
    commands = [item for item in observed if item[1] == COMMAND_DO_REPOSITION]
    require(
        result.returncode == 0 and "recovery region reached" in result.stdout,
        "recovery reports success after peer position progress",
        diagnostic,
    )
    require(
        len(commands) == 1 and commands[0][0] == "COMMAND_INT" and commands[0][2] == GLOBAL_RELATIVE_ALT_INT_FRAME,
        "recovery uses one relative-altitude COMMAND_INT reposition",
        diagnostic,
    )
    require(
        [target for target in targets if target[0] == COMMAND_DO_REPOSITION] == [(COMMAND_DO_REPOSITION, 1, 1)],
        "recovery targets the selected vehicle system and component",
        f"targets={targets}; {diagnostic}",
    )
    params = commands[0][3]
    require(
        params[0] == 0.0
        and params[1] == 0.0
        and params[2] == 30.0
        and params[3] != params[3]
        and params[4] == round(point[0] * 1.0e7)
        and params[5] == round(point[1] * 1.0e7)
        and params[6] == point[2],
        "recovery carries the reviewed fixed-wing target parameters",
        diagnostic,
    )


def case_quadplane_fixed_wing_recovery_uses_reposition_command_int(cli: Path) -> None:
    point = (45.5030, -73.5673, 20.0)
    port = find_free_udp_port()
    observed: list[CommandRecord] = []
    targets: list[tuple[int, int, int]] = []

    def action(peer):
        result = run_cli(cli, port, "fixed-wing-recovery", *(str(value) for value in point))
        observed.extend(peer.commands())
        targets.extend(peer.command_targets)
        return result

    result = with_peer(
        port,
        1,
        ACCEPTED,
        action,
        params={"Q_ENABLE": 2.0, "Q_GUIDED_MODE": 0.0},
        vehicle_type=mavlink.MAV_TYPE_FIXED_WING,
        autopilot_type=mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
        initial_mode=15,
        initial_armed=True,
        vtol_state=mavlink.MAV_VTOL_STATE_FW,
    )
    require_recovery_wire(result, observed, targets, point)


def case_quadplane_fixed_wing_recovery_rejects_denied_ack(cli: Path) -> None:
    result, observed = run_cli_case(
        cli,
        "fixed-wing-recovery",
        "45.5030",
        "-73.5673",
        "20",
        ack_result=DENIED,
        params={"Q_ENABLE": 2.0, "Q_GUIDED_MODE": 0.0},
        vehicle_type=mavlink.MAV_TYPE_FIXED_WING,
        autopilot_type=mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
        initial_mode=15,
        initial_armed=True,
        vtol_state=mavlink.MAV_VTOL_STATE_FW,
    )
    require(
        result.returncode != 0 and "recovery region reached" not in result.stdout,
        "denied recovery command does not report completion",
        describe(result, observed),
    )
