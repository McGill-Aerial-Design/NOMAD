# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Deterministic MAVSDK peer case for fixed-wing QuadPlane route packets."""

from __future__ import annotations

from pathlib import Path

from mavsdk_fixture_harness import describe, require, run_cli_case
from mavsdk_peer import (
    COMMAND_DO_REPOSITION,
    COMMAND_DO_SET_MODE,
    GLOBAL_RELATIVE_ALT_INT_FRAME,
    CommandRecord,
)
from pymavlink.dialects.v20 import ardupilotmega as mavlink


def find_parameters(observed: list[CommandRecord], command_id: int) -> tuple[float, ...] | None:
    for kind, command, _frame, parameters in observed:
        if kind == "COMMAND_LONG" and command == command_id:
            return parameters
    return None


def require_route_wire(
    result, observed: list[CommandRecord], waypoints: tuple[tuple[float, float, float], ...]
) -> None:
    reposition = [item for item in observed if item[0] == "COMMAND_INT" and item[1] == COMMAND_DO_REPOSITION]
    mode_parameters = find_parameters(observed, COMMAND_DO_SET_MODE)
    diagnostic = describe(result, observed)
    require(
        result.returncode == 0 and "fixed-wing route verified: waypoints=2" in result.stdout,
        "QuadPlane route success follows two reported position changes",
        diagnostic,
    )
    require(
        len(reposition) == 2
        and all(frame == GLOBAL_RELATIVE_ALT_INT_FRAME for _kind, _command, frame, _params in reposition),
        "fixed-wing route uses two relative-altitude COMMAND_INT reposition requests",
        diagnostic,
    )
    require(
        mode_parameters is not None and mode_parameters[:2] == (1.0, 15.0),
        "fixed-wing route explicitly establishes semantic GUIDED mode",
        diagnostic,
    )
    for index, (_kind, _command, _frame, parameters) in enumerate(reposition):
        latitude, longitude, altitude = waypoints[index]
        require(
            parameters[0] == 0.0
            and parameters[1] == 0.0
            and parameters[2] == 30.0
            and parameters[3] != parameters[3]
            and parameters[4] == round(latitude * 1.0e7)
            and parameters[5] == round(longitude * 1.0e7)
            and parameters[6] == altitude,
            f"waypoint {index + 1} carries the reviewed route parameters",
            diagnostic,
        )


def case_quadplane_fixed_wing_route_uses_reposition_command_int(cli: Path) -> None:
    waypoints = ((45.5030, -73.5673, 20.0), (45.5043, -73.5673, 20.0))
    arguments = tuple(str(value) for waypoint in waypoints for value in waypoint)
    result, observed = run_cli_case(
        cli,
        "fixed-wing-route",
        *arguments,
        params={"Q_ENABLE": 2.0},
        vehicle_type=mavlink.MAV_TYPE_FIXED_WING,
        autopilot_type=mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
        initial_mode=10,
        initial_armed=True,
        vtol_state=mavlink.MAV_VTOL_STATE_FW,
    )
    require_route_wire(result, observed, waypoints)
