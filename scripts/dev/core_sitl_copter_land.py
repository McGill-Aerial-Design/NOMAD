# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Qualify installed CLI/runtime Copter LAND engagement on one isolated simulator."""

from __future__ import annotations

import json
import os
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from authority_sitl_observer import OBSERVER_PORT, REVISION, ProbeError, code_identity, verify_isolated_simulator
from authority_sitl_relay import AuthorityRelay
from copter_land_sitl_observer import GUIDED, LAND, CopterLandObserver
from core_sitl_authority import RuntimeProcess, wait_status
from pymavlink import mavutil
from runtime_fixture_support import FIXTURE_CREDENTIALS, find_cli, find_runtime

ENGAGEMENT_BUDGET_SECONDS = 3.0
CLI_BUDGET_SECONDS = 3.5
SUCCESS_MESSAGE = "LAND mode observed; touchdown not verified"
SCOPE = "pinned_copter_installed_cli_runtime_land_engagement_and_separate_simulator_ground_observation"


def send_setup_command(observer: CopterLandObserver, command: int, parameters: list[float]) -> float:
    boundary = time.monotonic()
    observer.connection.mav.command_long_send(1, 1, command, 0, *parameters)
    observer.wait_acknowledgement(command, boundary)
    return boundary


def prepare_airborne_simulator(observer: CopterLandObserver, runtime: RuntimeProcess) -> None:
    if runtime.process is not None:
        raise ProbeError("runtime_present_during_external_setup")
    observer.wait_for(observer.is_ready_for_setup, "fresh_disarmed_simulator_readiness_missing", 60.0)
    send_setup_command(
        observer,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        [mavutil.mavlink.MAVLINK_MSG_ID_EXTENDED_SYS_STATE, 500000, 0, 0, 0, 0, 0],
    )
    boundary = time.monotonic()
    observer.connection.mav.set_mode_send(1, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, GUIDED)
    observer.wait_for(lambda: observer.has_mode(GUIDED, False, boundary), "simulator_guided_setup_missing", 10.0)
    boundary = send_setup_command(observer, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, [1, 0, 0, 0, 0, 0, 0])
    observer.wait_for(lambda: observer.has_mode(GUIDED, True, boundary), "simulator_arm_setup_missing", 10.0)
    boundary = send_setup_command(observer, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, [0, 0, 0, 0, 0, 0, 5])
    observer.wait_for(lambda: observer.is_airborne(boundary), "fresh_airborne_guided_simulator_missing", 40.0)


def wait_cli(process: Any, observer: CopterLandObserver, deadline: float) -> None:
    try:
        while process.poll() is None:
            observer.pump(0.02)
            if time.monotonic() >= deadline:
                raise ProbeError("cli_operation_budget_exceeded")
    except BaseException:
        if process.poll() is None:
            process.kill()
        process.communicate()
        raise


def run_cli(binary: Path, port: int, verb: str, observer: CopterLandObserver) -> tuple[str, float, float]:
    environment = os.environ.copy()
    environment["NOMAD_RUNTIME_IPC_PORT"] = str(port)
    environment["NOMAD_CLIENT_ID"] = "nomad-cli"
    environment["NOMAD_CLIENT_CREDENTIAL"] = FIXTURE_CREDENTIALS["nomad-cli"]
    started = time.monotonic()
    budget = CLI_BUDGET_SECONDS if verb == "land" else 10.0
    with subprocess.Popen(
        [str(binary), verb], env=environment, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True
    ) as process:
        wait_cli(process, observer, started + budget)
        output, _error = process.communicate()
        completed = time.monotonic()
        observer.pump(0.0)
        if process.returncode != 0:
            raise ProbeError("cli_land_failed" if verb == "land" else "cli_admission_failed")
    if completed - started > budget:
        raise ProbeError("cli_operation_budget_exceeded")
    return output.strip(), completed - started, completed


def require_valid_land_commands(commands: list[dict[str, Any]]) -> tuple[int, int]:
    if not commands:
        raise ProbeError("runtime_land_wire_command_missing")
    sources = {(command["system"], command["component"]) for command in commands}
    if len(sources) != 1:
        raise ProbeError("runtime_land_source_changed")
    for command in commands:
        if command["target_system"] != 1 or command["target_component"] != 1:
            raise ProbeError("runtime_land_target_invalid")
        if command["parameters"] != [0.0] * 7:
            raise ProbeError("runtime_land_parameters_invalid")
    return sources.pop()


def validate_land_wire(relay: AuthorityRelay, cli_seconds: float, completed_at: float) -> dict[str, Any]:
    commands = list(relay.land_commands)
    system, component = require_valid_land_commands(commands)
    first_send = commands[0]["observed_at"]
    acknowledgements = [sample for sample in relay.land_acknowledgements if first_send < sample[0] <= completed_at]
    accepted = [sample for sample in acknowledgements if sample[1] == mavutil.mavlink.MAV_RESULT_ACCEPTED]
    if not accepted:
        raise ProbeError("runtime_land_accepted_ack_missing")
    ack_time = accepted[0][0]
    heartbeats = [sample for sample in relay.heartbeats if ack_time < sample[0] <= completed_at and sample[1] == LAND]
    if not heartbeats:
        raise ProbeError("runtime_land_post_ack_heartbeat_missing")
    observed_at = heartbeats[0][0]
    if observed_at - first_send >= ENGAGEMENT_BUDGET_SECONDS or cli_seconds > CLI_BUDGET_SECONDS:
        raise ProbeError("land_engagement_budget_exceeded")
    return {
        "wire_attempts": len(commands),
        "command": mavutil.mavlink.MAV_CMD_NAV_LAND,
        "runtime_source": {"system": system, "component": component},
        "accepted_ack_after_first_send_ms": round((ack_time - first_send) * 1000.0, 3),
        "land_heartbeat_after_ack_ms": round((observed_at - ack_time) * 1000.0, 3),
        "wire_engagement_ms": round((observed_at - first_send) * 1000.0, 3),
        "cli_wall_ms": round(cli_seconds * 1000.0, 3),
    }


def exercise_land(
    runtime: RuntimeProcess, observer: CopterLandObserver, relay: AuthorityRelay, binary: Path
) -> dict[str, Any]:
    runtime.start()
    ready = wait_status(runtime.ipc_port, observer=observer)
    if ready["authority_owner"] is not None:
        raise ProbeError("runtime_startup_owner_present")
    run_cli(binary, runtime.ipc_port, "admit", observer)
    observer.wait_for(observer.is_airborne, "airborne_state_lost_before_land", 3.0)
    altitude = observer.positions[-1][1]
    output, elapsed, completed_at = run_cli(binary, runtime.ipc_port, "land", observer)
    if output != SUCCESS_MESSAGE:
        raise ProbeError("cli_land_success_claim_invalid")
    evidence = validate_land_wire(relay, elapsed, completed_at)
    observer.wait_for(lambda: observer.has_stable_ground_state(completed_at), "simulator_ground_state_missing", 60.0)
    if len(relay.land_commands) != evidence["wire_attempts"]:
        raise ProbeError("land_replayed_after_result")
    return {
        "cli_land_invocations": 1,
        "api_result": SUCCESS_MESSAGE,
        "airborne_relative_altitude_m": round(altitude, 3),
        **evidence,
        "separate_simulator_ground_state": {
            "custom_mode": observer.heartbeats[-1][1],
            "armed": observer.heartbeats[-1][2],
            "landed_state": observer.landed_states[-1][1],
            "relative_altitude_m": round(observer.positions[-1][1], 3),
            "low_altitude_samples": len(observer.ground_samples(completed_at)),
        },
    }


def close_resources(resources: list[Any], failure: str | None) -> str | None:
    for resource in resources:
        if resource is None:
            continue
        try:
            if isinstance(resource, RuntimeProcess):
                resource.stop()
            else:
                resource.close()
        except Exception as error:
            failure = failure or getattr(error, "category", "probe_shutdown_failed")
    return failure


def run_probe() -> dict[str, Any]:
    started = datetime.now(timezone.utc).isoformat(timespec="seconds")
    runtime, observer, relay = RuntimeProcess(), None, None
    failure, phase, evidence = None, "container_guard", {}
    try:
        verify_isolated_simulator()
        binary = find_cli()
        find_runtime()
        relay = AuthorityRelay(runtime.udp_port)
        connection = mavutil.mavlink_connection(
            f"udpin:0.0.0.0:{OBSERVER_PORT}", source_system=250, source_component=190
        )
        observer = CopterLandObserver(connection)
        phase = "external_simulator_preparation_runtime_absent"
        prepare_airborne_simulator(observer, runtime)
        phase = "authenticated_cli_land_and_ground_observation"
        evidence = exercise_land(runtime, observer, relay, binary)
    except ProbeError as error:
        failure = error.category
    except Exception:
        failure = "probe_execution_failed"
    finally:
        failure = close_resources([runtime, observer, relay], failure)
    result = {
        "result": "failed" if failure else "passed",
        "scope": SCOPE,
        "started_utc": started,
        "firmware_sha": REVISION,
        **code_identity(),
        "setup_source": {"system": 250, "component": 190},
        "physical_aircraft_qualification": False,
        "evidence": evidence,
    }
    if failure:
        result.update(failure=failure, phase=phase)
    return result


def main() -> int:
    result = run_probe()
    print(json.dumps(result, sort_keys=True), file=sys.stderr if result["result"] == "failed" else sys.stdout)
    return 1 if result["result"] == "failed" else 0


if __name__ == "__main__":
    raise SystemExit(main())
