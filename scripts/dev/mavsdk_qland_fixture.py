# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Deterministic QLAND command and pinned-version transport checks."""

from __future__ import annotations

from mavsdk_fixture_harness import describe, require, run_qland_probe
from mavsdk_peer import (
    COMMAND_DO_SET_MODE,
    COMMAND_NAV_LAND,
    COMMAND_NAV_RETURN_TO_LAUNCH,
    DENIED,
    CommandRecord,
)
from pymavlink.dialects.v20 import ardupilotmega as mavlink


def find_parameters(observed: list[CommandRecord], command_id: int):
    for kind, command, _frame, parameters in observed:
        if kind == "COMMAND_LONG" and command == command_id:
            return parameters
    return None


def case_qland_transport_wire_semantics(probe) -> None:
    result, observed, targets = run_qland_probe(probe)
    version_request = find_parameters(observed, mavlink.MAV_CMD_REQUEST_MESSAGE)
    qland = find_parameters(observed, COMMAND_DO_SET_MODE)
    require(
        result.returncode == 0 and "version=4.7.1 hash=dbe79216 landed_state=on_ground ack=0" in result.stdout,
        "EXTENDED_SYS_STATE landed-state mapping, pinned version readback, and QLAND acknowledgement",
        describe(result, observed),
    )
    require(
        version_request is not None
        and version_request[0] == float(mavlink.MAVLINK_MSG_ID_AUTOPILOT_VERSION)
        and qland == (1.0, 20.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        and targets
        and all(target_system == 1 and target_component == 1 for _command, target_system, target_component in targets)
        and {command for command, _system, _component in targets}
        <= {mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, mavlink.MAV_CMD_REQUEST_MESSAGE, COMMAND_DO_SET_MODE}
        and sum(command == COMMAND_DO_SET_MODE for command, _system, _component in targets) == 1
        and all(
            command not in (COMMAND_NAV_LAND, COMMAND_NAV_RETURN_TO_LAUNCH)
            for _kind, command, _frame, _parameters in observed
        ),
        "QLAND is one targeted COMMAND_LONG with fixed custom-mode parameters and no NAV_LAND/RTL command",
        describe(result, observed),
    )


def case_qland_transport_rejects_denied_ack(probe) -> None:
    result, observed, targets = run_qland_probe(probe, DENIED)
    qland = find_parameters(observed, COMMAND_DO_SET_MODE)
    require(
        result.returncode != 0
        and "landed_state=on_ground" in result.stdout
        and "ack=2" in result.stdout
        and qland == (1.0, 20.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        and sum(command == COMMAND_DO_SET_MODE for command, _system, _component in targets) == 1,
        "a denied QLAND COMMAND_ACK is returned as failure",
        describe(result, observed),
    )
