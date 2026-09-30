# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Run the MAVSDK transport parity cases against a deterministic peer.

`mavsdk_peer.VehiclePeer` is an ArduPilot-like UDP vehicle; these cases drive
the real `nomad` CLI and the connection probe binary against
it, so they exercise the full transport path including NOMAD's command
acknowledgement and state-verification logic.

Each command case starts the peer in a state that is observably different from
the result the command must produce, and asserts the CLI's own verified output.
A pass therefore means the core observed the change on the wire, not merely that
an acknowledgement arrived.

The link-level and zero-delivery cases live in `mavsdk_link_fixture.py` and are
re-exported here so this module stays the one entry point
(`python scripts/dev/mavsdk_connection_fixture.py`) and the one the pytest driver
imports.
"""

from __future__ import annotations

import subprocess
import sys
import time
from pathlib import Path

from mavsdk_fixture_harness import (
    describe,
    find_binary,
    find_free_udp_port,
    require,
    run_cli_case,
    run_param_probe,
    run_probe_case,
    with_peer,
)
from mavsdk_link_fixture import (
    case_coalesced_telemetry_is_verified,
    case_disabled_fence_does_not_verify,
    case_fence_upload_and_readback,
    case_gcs_heartbeat_announces_to_a_silent_peer,
    case_link_loss_is_observed_as_stale,
    case_velocity_setpoint_reaches_the_wire,
    case_zero_delivery_scenarios,
)
from mavsdk_peer import (
    ACCEPTED,
    COMMAND_ARM_DISARM,
    COMMAND_DO_MOTOR_TEST,
    COMMAND_DO_MOUNT_CONFIGURE,
    COMMAND_DO_REPOSITION,
    COMMAND_DO_SET_MODE,
    COMMAND_DO_SET_RELAY,
    COMMAND_DO_SET_SERVO,
    COMMAND_DO_VTOL_TRANSITION,
    COMMAND_NAV_LAND,
    COMMAND_NAV_RETURN_TO_LAUNCH,
    COMMAND_NAV_TAKEOFF,
    DENIED,
    GLOBAL_RELATIVE_ALT_INT_FRAME,
    CommandRecord,
    VehiclePeer,
)
from mavsdk_qland_fixture import case_qland_transport_rejects_denied_ack, case_qland_transport_wire_semantics
from mavsdk_recovery_fixture import (
    case_quadplane_fixed_wing_recovery_rejects_denied_ack,
    case_quadplane_fixed_wing_recovery_uses_reposition_command_int,
)
from mavsdk_route_fixture import case_quadplane_fixed_wing_route_uses_reposition_command_int
from mavsdk_vtol_transition_fixture import (
    case_quadplane_transition_to_vtol_is_verified,
    case_transition_directions_use_the_same_targeted_command_transport,
)
from pymavlink.dialects.v20 import ardupilotmega as mavlink


def find_parameters(observed: list[CommandRecord], command_id: int, wire_form: str = "COMMAND_LONG"):
    """Return the parameters of the first matching command, or None."""
    for kind, command, _frame, parameters in observed:
        if kind == wire_form and command == command_id:
            return parameters
    return None


def case_status(cli: Path) -> None:
    result, observed = run_cli_case(cli, "status")
    require(
        result.returncode == 0 and "position=" in result.stdout, "status telemetry parity", describe(result, observed)
    )


def case_quadplane_identity_uses_ardupilot_parameter(cli: Path) -> None:
    options = {
        "vehicle_type": mavlink.MAV_TYPE_FIXED_WING,
        "autopilot_type": mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
    }
    cases = (
        ({"Q_ENABLE": 1.0}, "QuadPlane", "Q_ENABLE=1 identifies QuadPlane"),
        ({"Q_ENABLE": 2.0}, "QuadPlane", "Q_ENABLE=2 identifies QuadPlane"),
        ({"Q_ENABLE": 0.0}, "Plane", "Q_ENABLE=0 identifies Plane"),
        ({}, "Unknown", "unavailable Q_ENABLE leaves identity unresolved"),
        ({"Q_ENABLE": 3.0}, "Unknown", "invalid Q_ENABLE leaves identity unresolved"),
    )
    for params, aircraft_class, name in cases:
        result, observed = run_cli_case(cli, "status", params=params, **options)
        require(
            result.returncode == 0
            and "vehicle_type=1" in result.stdout
            and f"aircraft_class={aircraft_class}" in result.stdout,
            name,
            describe(result, observed),
        )


def case_arm_accepted(cli: Path) -> None:
    result, observed = run_cli_case(cli, "arm")
    require(
        result.returncode == 0 and "arm verified" in result.stdout,
        "accepted command is verified",
        describe(result, observed),
    )


def case_arm_denied(cli: Path) -> None:
    result, observed = run_cli_case(cli, "arm", ack_result=DENIED)
    require(
        result.returncode != 0 and "rejected by ArduPilot" in result.stdout,
        "denied command is truthful",
        describe(result, observed),
    )


def case_arm_timeout(cli: Path) -> None:
    result, observed = run_cli_case(cli, "arm", ack_result=None, timeout=30)
    require(
        result.returncode != 0 and "timed out waiting for acknowledgement" in result.stdout,
        "missing acknowledgement is truthful",
        describe(result, observed),
    )


def case_command_timeout_honors_caller_budget(probe: Path) -> None:
    started = time.monotonic()
    result, observed = run_probe_case(
        probe,
        COMMAND_ARM_DISARM,
        "long",
        ack_result=None,
        timeout_ms=100,
    )
    elapsed = time.monotonic() - started
    require(
        result.returncode != 0 and "ack=none" in result.stdout and elapsed < 1.0,
        "command timeout honors the caller budget",
        describe(result, observed) + f" elapsed={elapsed:.3f}s",
    )


def case_param_timeout_honors_caller_budget(probe: Path) -> None:
    port = find_free_udp_port()

    def action(_peer: VehiclePeer) -> subprocess.CompletedProcess:
        return run_param_probe(probe, port, 1, "UNKNOWN_PARAMETER", 100)

    started = time.monotonic()
    result = with_peer(port, 1, ACCEPTED, action)
    elapsed = time.monotonic() - started
    require(
        result.returncode != 0 and "param=none" in result.stdout and elapsed < 1.0,
        "parameter timeout honors the caller budget",
        f"rc={result.returncode} stdout={result.stdout!r} stderr={result.stderr!r} elapsed={elapsed:.3f}s",
    )


def case_command_int(probe: Path) -> None:
    result, observed = run_probe_case(probe, COMMAND_DO_REPOSITION, "int")
    used_int = any(kind == "COMMAND_INT" and frame == GLOBAL_RELATIVE_ALT_INT_FRAME for kind, _, frame, _ in observed)
    require(
        result.returncode == 0 and used_int,
        "goto uses COMMAND_INT with the relative-altitude frame",
        describe(result, observed),
    )


def case_command_long(probe: Path) -> None:
    result, observed = run_probe_case(probe, COMMAND_ARM_DISARM, "long")
    require(
        result.returncode == 0 and any(kind == "COMMAND_LONG" for kind, _, _, _ in observed),
        "non-location command uses COMMAND_LONG",
        describe(result, observed),
    )


def case_wrong_system(probe: Path) -> None:
    result, observed = run_probe_case(probe, COMMAND_ARM_DISARM, "long", expected_system_id=1, peer_system_id=2)
    require(
        result.returncode != 0 and "connect=fail" in result.stdout,
        "wrong autopilot identity is refused",
        describe(result, observed),
    )


def case_unqualified_identity_sends_no_mode_command(cli: Path) -> None:
    cases = (
        {"vehicle_type": mavlink.MAV_TYPE_VTOL_TILTROTOR, "autopilot_type": mavlink.MAV_AUTOPILOT_GENERIC},
        {"vehicle_type": 99, "autopilot_type": mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA},
    )
    for peer_options in cases:
        result, observed = run_cli_case(cli, "mode", "15", **peer_options)
        require(
            result.returncode != 0 and "set mode is not qualified for Unknown" in result.stdout,
            "unknown aircraft identity fails closed",
            describe(result, observed),
        )
        require(
            find_parameters(observed, COMMAND_DO_SET_MODE) is None,
            "unknown aircraft sends no mode command",
            describe(result, observed),
        )


def case_mode_is_verified(cli: Path) -> None:
    result, observed = run_cli_case(cli, "mode", "4")
    parameters = find_parameters(observed, COMMAND_DO_SET_MODE)
    require(
        result.returncode == 0 and "set mode verified" in result.stdout,
        "mode change is verified from autopilot state",
        describe(result, observed),
    )
    require(
        parameters is not None and parameters[:2] == (1.0, 4.0),
        "mode uses DO_SET_MODE with the requested custom mode",
        describe(result, observed),
    )


def case_takeoff_is_verified(cli: Path) -> None:
    result, observed = run_cli_case(cli, "takeoff", "5")
    parameters = find_parameters(observed, COMMAND_NAV_TAKEOFF)
    require(
        result.returncode == 0 and "takeoff verified" in result.stdout,
        "takeoff is verified from reported altitude",
        describe(result, observed),
    )
    require(
        parameters is not None and parameters[6] == 5.0,
        "takeoff uses NAV_TAKEOFF with the requested altitude",
        describe(result, observed),
    )


def case_quadplane_vtol_takeoff_is_verified(cli: Path) -> None:
    result, observed = run_cli_case(
        cli,
        "vtol-takeoff",
        "5",
        params={"Q_ENABLE": 1.0},
        vehicle_type=mavlink.MAV_TYPE_FIXED_WING,
        autopilot_type=mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
    )
    expected_commands = {COMMAND_DO_SET_MODE, COMMAND_ARM_DISARM, COMMAND_NAV_TAKEOFF}
    command_ids = [command for _kind, command, _frame, _parameters in observed if command in expected_commands]
    parameters = find_parameters(observed, COMMAND_NAV_TAKEOFF)
    require(
        result.returncode == 0 and "vtol takeoff verified" in result.stdout,
        "QuadPlane VTOL takeoff is verified from armed guided climb state",
        describe(result, observed),
    )
    require(
        command_ids == [COMMAND_DO_SET_MODE, COMMAND_ARM_DISARM, COMMAND_NAV_TAKEOFF],
        "QuadPlane startup uses semantic GUIDED, arm and dedicated takeoff commands",
        describe(result, observed),
    )
    require(
        parameters is not None and parameters[6] == 5.0,
        "QuadPlane takeoff carries the requested altitude",
        describe(result, observed),
    )


def case_quadplane_transition_is_verified(cli: Path) -> None:
    result, observed = run_cli_case(
        cli,
        "transition-to-fixed-wing",
        params={"Q_ENABLE": 1.0},
        vehicle_type=mavlink.MAV_TYPE_FIXED_WING,
        autopilot_type=mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
        initial_mode=10,
        initial_armed=True,
        vtol_state=mavlink.MAV_VTOL_STATE_MC,
        transition_reports_intermediate=True,
        transition_reaches_fixed_wing=True,
    )
    parameters = find_parameters(observed, COMMAND_DO_VTOL_TRANSITION)
    command_ids = [command for _kind, command, _frame, _parameters in observed if command == COMMAND_DO_VTOL_TRANSITION]
    require(
        result.returncode == 0 and "transition to fixed wing verified" in result.stdout,
        "QuadPlane transition is verified from EXTENDED_SYS_STATE fixed-wing state",
        describe(result, observed),
    )
    require(
        command_ids == [COMMAND_DO_VTOL_TRANSITION]
        and parameters is not None
        and parameters[0] == float(mavlink.MAV_VTOL_STATE_FW),
        "QuadPlane transition uses DO_VTOL_TRANSITION with MAV_VTOL_STATE_FW",
        describe(result, observed),
    )


def case_goto_is_verified(cli: Path) -> None:
    result, observed = run_cli_case(cli, "goto", "45.5027", "-73.5663", "5")
    parameters = find_parameters(observed, COMMAND_DO_REPOSITION, "COMMAND_INT")
    require(
        result.returncode == 0 and "goto location verified" in result.stdout,
        "goto is verified from reported position",
        describe(result, observed),
    )
    require(
        parameters is not None
        and abs(parameters[4] / 1e7 - 45.5027) < 1e-6
        and abs(parameters[5] / 1e7 - (-73.5663)) < 1e-6
        and parameters[6] == 5.0,
        "goto carries scaled latitude/longitude and relative altitude",
        describe(result, observed),
    )


def case_rtl_is_verified(cli: Path) -> None:
    result, observed = run_cli_case(cli, "rtl")
    require(
        result.returncode == 0
        and "return to launch verified" in result.stdout
        and find_parameters(observed, COMMAND_NAV_RETURN_TO_LAUNCH) is not None,
        "RTL uses NAV_RETURN_TO_LAUNCH and is verified from mode",
        describe(result, observed),
    )


def case_land_is_verified(cli: Path) -> None:
    result, observed = run_cli_case(cli, "land")
    require(
        result.returncode == 0
        and "land verified" in result.stdout
        and find_parameters(observed, COMMAND_NAV_LAND) is not None,
        "land uses NAV_LAND and is verified from mode",
        describe(result, observed),
    )


def case_servo_parity(cli: Path) -> None:
    result, observed = run_cli_case(cli, "servo", "9", "1500")
    parameters = find_parameters(observed, COMMAND_DO_SET_SERVO)
    require(
        result.returncode == 0 and "servo command verified" in result.stdout,
        "servo command is acknowledged and reported",
        describe(result, observed),
    )
    require(
        parameters is not None and parameters[:2] == (9.0, 1500.0),
        "servo uses DO_SET_SERVO with channel and PWM",
        describe(result, observed),
    )


def case_relay_parity(cli: Path) -> None:
    result, observed = run_cli_case(cli, "relay", "3", "1")
    parameters = find_parameters(observed, COMMAND_DO_SET_RELAY)
    require(
        result.returncode == 0 and "relay command verified" in result.stdout,
        "relay command is acknowledged and reported",
        describe(result, observed),
    )
    require(
        parameters is not None and parameters[:2] == (3.0, 1.0),
        "relay uses DO_SET_RELAY with relay number and state",
        describe(result, observed),
    )


def case_gimbal_config_parity(cli: Path) -> None:
    result, observed = run_cli_case(cli, "gimbal-config", "2")
    parameters = find_parameters(observed, COMMAND_DO_MOUNT_CONFIGURE)
    require(
        result.returncode == 0 and "gimbal configuration verified" in result.stdout,
        "gimbal configuration is acknowledged and reported",
        describe(result, observed),
    )
    require(
        parameters is not None and parameters[0] == 2.0,
        "gimbal configuration uses DO_MOUNT_CONFIGURE with the mount mode",
        describe(result, observed),
    )


def case_motor_test_parity(cli: Path) -> None:
    result, observed = run_cli_case(cli, "motor-test", "1", "0", "1")
    parameters = find_parameters(observed, COMMAND_DO_MOTOR_TEST)
    require(
        result.returncode == 0 and "motor test command verified" in result.stdout,
        "motor test is acknowledged and reported",
        describe(result, observed),
    )
    require(
        parameters == (1.0, 1.0, 0.0, 1.0, 1.0, 0.0, 0.0),
        "motor test uses DO_MOTOR_TEST with instance, PWM throttle type, throttle, timeout and motor count",
        describe(result, observed),
    )


def case_quadplane_navigation(cli: Path) -> None:
    case_quadplane_fixed_wing_route_uses_reposition_command_int(cli)
    case_quadplane_fixed_wing_recovery_uses_reposition_command_int(cli)
    case_quadplane_fixed_wing_recovery_rejects_denied_ack(cli)


def case_quadplane_transitions(cli: Path) -> None:
    case_quadplane_transition_is_verified(cli)
    case_quadplane_transition_to_vtol_is_verified(cli)
    case_transition_directions_use_the_same_targeted_command_transport(cli)


def run_observation_cases(cli: Path, probe: Path, zero_delivery: Path) -> None:
    case_status(cli)
    case_quadplane_identity_uses_ardupilot_parameter(cli)
    case_gcs_heartbeat_announces_to_a_silent_peer(probe)
    case_coalesced_telemetry_is_verified(cli)
    case_link_loss_is_observed_as_stale(probe)
    case_velocity_setpoint_reaches_the_wire(probe)
    case_fence_upload_and_readback(probe)
    case_disabled_fence_does_not_verify(probe)
    case_zero_delivery_scenarios(zero_delivery)


def run_command_cases(cli: Path, probe: Path) -> None:
    case_arm_accepted(cli)
    case_arm_denied(cli)
    case_arm_timeout(cli)
    case_command_timeout_honors_caller_budget(probe)
    case_param_timeout_honors_caller_budget(probe)
    case_command_int(probe)
    case_command_long(probe)
    case_qland_transport_wire_semantics(probe)
    case_qland_transport_rejects_denied_ack(probe)
    case_wrong_system(probe)
    case_unqualified_identity_sends_no_mode_command(cli)
    case_mode_is_verified(cli)
    case_takeoff_is_verified(cli)
    case_quadplane_vtol_takeoff_is_verified(cli)


def run_vehicle_operation_cases(cli: Path) -> None:
    case_quadplane_transitions(cli)
    case_quadplane_navigation(cli)
    case_goto_is_verified(cli)
    case_rtl_is_verified(cli)
    case_land_is_verified(cli)
    case_servo_parity(cli)
    case_relay_parity(cli)
    case_gimbal_config_parity(cli)
    case_motor_test_parity(cli)


def main() -> int:
    cli = find_binary("nomad-qualification")
    probe = find_binary("nomad_mavsdk_connection_tests")
    zero_delivery = find_binary("nomad_mavsdk_zero_delivery_tests")
    if cli is None or probe is None or zero_delivery is None:
        print(
            "MAVSDK transport qualification binaries are missing; run `pixi run "
            "build-mavsdk-transport-qualification` first",
            file=sys.stderr,
        )
        return 2

    run_observation_cases(cli, probe, zero_delivery)
    run_command_cases(cli, probe)
    run_vehicle_operation_cases(cli)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
