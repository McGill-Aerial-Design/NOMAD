# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Qualify the pinned QuadPlane VTOL-to-fixed-wing transition."""

from __future__ import annotations

import sys
import threading
import time
from pathlib import Path

from core_sitl_quadplane_observe import (
    EXPECTED_IDENTITY,
    ScenarioError,
    get_sitl_port,
    read_status,
    request_observed_mode,
    require_fresh_telemetry,
    require_status,
    run_cli,
)
from core_sitl_status import find_binary
from pymavlink import mavutil

MODE_AUTO = 10
VTOL_STATE_UNDEFINED = mavutil.mavlink.MAV_VTOL_STATE_UNDEFINED
VTOL_STATE_TRANSITION_TO_FW = mavutil.mavlink.MAV_VTOL_STATE_TRANSITION_TO_FW
VTOL_STATE_MC = mavutil.mavlink.MAV_VTOL_STATE_MC
VTOL_STATE_FW = mavutil.mavlink.MAV_VTOL_STATE_FW
VTOL_STATE_NAMES = {
    VTOL_STATE_UNDEFINED: "undefined",
    VTOL_STATE_TRANSITION_TO_FW: "transition_to_fixed_wing",
    VTOL_STATE_MC: "multicopter",
    VTOL_STATE_FW: "fixed_wing",
}
OBSERVER_PORT = 14581
VTOL_STATE_MAX_AGE_MS = 3000
TRANSITION_TIMEOUT_SECONDS = 120


class VtolStateObserver:
    """Collect EXTENDED_SYS_STATE independently of the NOMAD CLI process."""

    def __init__(self, port: int) -> None:
        self._port = port
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self.states: list[tuple[float, int]] = []

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=3)

    def _run(self) -> None:
        connection = mavutil.mavlink_connection(f"udpin:0.0.0.0:{self._port}", source_system=251)
        try:
            while not self._stop.is_set():
                message = connection.recv_match(type="EXTENDED_SYS_STATE", blocking=True, timeout=0.5)
                if message is not None:
                    self.states.append((time.monotonic(), int(message.vtol_state)))
        finally:
            connection.close()


def parse_position(fields: dict[str, str]) -> tuple[float, float]:
    try:
        latitude, longitude = fields["position"].split(",", maxsplit=1)
        return float(latitude), float(longitude)
    except (KeyError, ValueError) as error:
        raise ScenarioError(f"missing or invalid position; full status={fields}") from error


def parse_relative_altitude(fields: dict[str, str]) -> float:
    try:
        return float(fields["relative_altitude_m"])
    except (KeyError, ValueError) as error:
        raise ScenarioError(f"missing or invalid relative_altitude_m; full status={fields}") from error


def require_fresh_vtol_state(fields: dict[str, str], expected_state: str | None = None) -> None:
    require_status(fields, {"connected": "true", "heartbeat_fresh": "true", **EXPECTED_IDENTITY})
    require_fresh_telemetry(fields)
    if expected_state is not None and fields.get("vtol_state") != expected_state:
        raise ScenarioError(f"expected vtol_state={expected_state}; full status={fields}")
    try:
        age_ms = int(fields["vtol_state_age_ms"])
    except (KeyError, ValueError) as error:
        raise ScenarioError(f"missing or invalid vtol_state_age_ms; full status={fields}") from error
    if not 0 <= age_ms <= VTOL_STATE_MAX_AGE_MS:
        raise ScenarioError(f"stale vtol_state_age_ms={age_ms}; full status={fields}")


def wait_for_auto_and_multicopter(binary: Path, port: str, timeout: float = 20.0) -> dict[str, str]:
    deadline = time.monotonic() + timeout
    last: dict[str, str] = {}
    while time.monotonic() < deadline:
        last = read_status(binary, port)
        try:
            require_status(last, {"armed": "true", "mode": str(MODE_AUTO)})
            require_fresh_vtol_state(last, "multicopter")
        except ScenarioError:
            continue
        return last
    raise ScenarioError(f"QuadPlane did not reach safe AUTO multicopter state; last status={last}")


def upload_forward_auto_mission(port: str, fields: dict[str, str]) -> None:
    """Give AUTO a long forward waypoint so the tilt-tri can build airspeed."""
    connection = mavutil.mavlink_connection(f"udpin:0.0.0.0:{port}", source_system=250)
    try:
        heartbeat = connection.wait_heartbeat(timeout=10)
        if heartbeat is None:
            raise ScenarioError("test driver did not receive a QuadPlane heartbeat for mission setup")
        latitude, longitude = parse_position(fields)
        altitude_m = max(parse_relative_altitude(fields), 5.0)
        target_system = connection.target_system
        target_component = connection.target_component
        connection.mav.mission_clear_all_send(target_system, target_component, mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
        connection.mav.mission_count_send(target_system, target_component, 1, mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
        request = connection.recv_match(type=["MISSION_REQUEST_INT", "MISSION_REQUEST"], blocking=True, timeout=10)
        if request is None:
            raise ScenarioError("ArduPlane did not request the forward AUTO mission item")
        connection.mav.mission_item_int_send(
            target_system,
            target_component,
            int(request.seq),
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            0,
            50,
            0.0,
            0.0,
            0.0,
            0.0,
            int((latitude + 0.02) * 1e7),
            int(longitude * 1e7),
            altitude_m,
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
        )
        acknowledgement = connection.recv_match(type="MISSION_ACK", blocking=True, timeout=10)
        if acknowledgement is None or int(acknowledgement.type) != mavutil.mavlink.MAV_MISSION_ACCEPTED:
            result = None if acknowledgement is None else int(acknowledgement.type)
            raise ScenarioError(f"forward AUTO mission was not accepted: {result}")
    finally:
        connection.close()


def observed_state_names(states: list[tuple[float, int]]) -> list[str]:
    names: list[str] = []
    for _timestamp, state in states:
        name = VTOL_STATE_NAMES.get(state, f"unknown_{state}")
        if not names or names[-1] != name:
            names.append(name)
    return names


def prepare_transition(binary: Path, port: str) -> tuple[VtolStateObserver, str, float, str, dict[str, str]]:
    initial = read_status(binary, port)
    require_status(initial, {"armed": "false", **EXPECTED_IDENTITY})
    require_fresh_vtol_state(initial)
    starting_vtol_state = initial["vtol_state"]
    baseline_altitude_m = parse_relative_altitude(initial)

    takeoff_output = run_cli(binary, port, "vtol-takeoff", "5")
    if "vtol takeoff verified" not in takeoff_output:
        raise ScenarioError(f"NOMAD did not report verified VTOL takeoff: {takeoff_output!r}")
    after_takeoff = read_status(binary, port)
    require_status(after_takeoff, {"armed": "true", "mode": "15"})
    require_fresh_vtol_state(after_takeoff, "multicopter")

    upload_forward_auto_mission(port, after_takeoff)
    observer = VtolStateObserver(OBSERVER_PORT)
    observer.start()
    request_observed_mode(port, MODE_AUTO)
    pre_transition = wait_for_auto_and_multicopter(binary, port)
    return observer, starting_vtol_state, baseline_altitude_m, after_takeoff["vtol_state"], pre_transition


def complete_transition(
    binary: Path, port: str, observer: VtolStateObserver
) -> tuple[list[str], float, dict[str, str]]:
    transition_started = time.monotonic()
    transition_output = run_cli(binary, port, "transition-to-fixed-wing", timeout=TRANSITION_TIMEOUT_SECONDS)
    transition_elapsed = time.monotonic() - transition_started
    if "transition to fixed wing verified" not in transition_output:
        raise ScenarioError(f"NOMAD did not report verified fixed-wing transition: {transition_output!r}")
    final = read_status(binary, port)
    require_fresh_vtol_state(final, "fixed_wing")
    observer.stop()

    states = observed_state_names(observer.states)
    if not states:
        raise ScenarioError("independent observer recorded no EXTENDED_SYS_STATE transitions")
    if states[0] != "multicopter" or states[-1] != "fixed_wing":
        raise ScenarioError(f"unexpected authoritative VTOL state sequence: {states}")
    if "transition_to_fixed_wing" not in states:
        raise ScenarioError(f"intermediate transition state was not observed: {states}")
    return states, transition_elapsed, final


def format_evidence(
    starting_vtol_state: str,
    baseline_altitude_m: float,
    after_takeoff_vtol_state: str,
    pre_transition: dict[str, str],
    states: list[str],
    transition_elapsed: float,
    final: dict[str, str],
) -> str:
    return (
        "QuadPlane VTOL-to-fixed-wing transition qualification passed: "
        f"starting_vtol_state={starting_vtol_state} baseline_relative_altitude_m={baseline_altitude_m:.2f} "
        f"after_takeoff_vtol_state={after_takeoff_vtol_state} "
        f"pre_transition_mode={pre_transition['mode']} "
        f"pre_transition_vtol_state={pre_transition['vtol_state']} "
        f"command=MAV_CMD_DO_VTOL_TRANSITION param1=MAV_VTOL_STATE_FW ack_result=accepted "
        f"observed_states={states} completion_time_s={transition_elapsed:.1f} "
        f"final_vtol_state={final['vtol_state']} final_vtol_state_age_ms={final['vtol_state_age_ms']}"
    )


def main() -> int:
    binary = find_binary()
    if binary is None:
        print("error: qualification driver not found; run `pixi run build-qualification-cli` first", file=sys.stderr)
        return 2

    observer: VtolStateObserver | None = None
    try:
        port = get_sitl_port()
        observer, starting_vtol_state, baseline_altitude_m, after_takeoff_vtol_state, pre_transition = (
            prepare_transition(binary, port)
        )
        states, transition_elapsed, final = complete_transition(binary, port, observer)
    except (ValueError, ScenarioError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 1
    finally:
        if observer is not None:
            observer.stop()

    print(
        format_evidence(
            starting_vtol_state,
            baseline_altitude_m,
            after_takeoff_vtol_state,
            pre_transition,
            states,
            transition_elapsed,
            final,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
