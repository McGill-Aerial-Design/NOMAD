# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Qualify a stabilized fixed-wing-to-VTOL transition after recovery."""

from __future__ import annotations

import subprocess
import sys
import time
from pathlib import Path

from core_sitl_quadplane_observe import ScenarioError, get_sitl_port, read_status, request_observed_mode, run_cli
from core_sitl_quadplane_recovery import execute_recovery, recovery_point
from core_sitl_quadplane_route import (
    MODE_GUIDED,
    RouteObserver,
    distance_m,
    execute_and_observe_route,
    prepare_route,
    require_route_setup,
    wait_for_observer_start,
)
from core_sitl_quadplane_transition import (
    MODE_AUTO,
    OBSERVER_PORT,
    VTOL_STATE_FW,
    VTOL_STATE_MC,
    observed_state_names,
    parse_position,
    parse_relative_altitude,
    require_fresh_vtol_state,
    wait_for_auto_and_multicopter,
)
from core_sitl_status import find_binary
from pymavlink import mavutil

TRANSITION_TIMEOUT_SECONDS = 90
READY_TIMEOUT_SECONDS = 45
READY_RADIUS_METERS = 55.0
READY_MIN_ALTITUDE_METERS = 15.0
READY_MAX_ALTITUDE_METERS = 25.0
READY_MAX_GROUNDSPEED_MPS = 28.0
READY_MAX_GROUNDSPEED_VARIATION_MPS = 3.0
READY_MAX_CLIMB_RATE_MPS = 1.0
READY_MAX_ALTITUDE_VARIATION_METERS = 1.0
READY_MAX_RADIAL_VARIATION_METERS = 8.0
READY_SAMPLE_COUNT = 5
READY_DWELL_SECONDS = 2.0
VTOL_MAX_GROUNDSPEED_MPS = 3.0
VTOL_MAX_CLIMB_RATE_MPS = 0.5
VTOL_MAX_POSITION_SPREAD_METERS = 4.0
VTOL_MAX_ALTITUDE_VARIATION_METERS = 1.0
VTOL_SAMPLE_COUNT = 3
VTOL_DWELL_SECONDS = 2.0


def upload_transition_loiter_mission(port: str, point: tuple[float, float, float]) -> None:
    """Replace the completed route with one explicit transition-region loiter."""
    connection = mavutil.mavlink_connection(f"udpin:0.0.0.0:{port}", source_system=250)
    try:
        if connection.wait_heartbeat(timeout=10) is None:
            raise ScenarioError("test driver did not receive a QuadPlane heartbeat for transition setup")
        target_system, target_component = connection.target_system, connection.target_component
        mission_type = mavutil.mavlink.MAV_MISSION_TYPE_MISSION
        connection.mav.mission_clear_all_send(target_system, target_component, mission_type)
        connection.mav.mission_count_send(target_system, target_component, 1, mission_type)
        request = connection.recv_match(type=["MISSION_REQUEST_INT", "MISSION_REQUEST"], blocking=True, timeout=10)
        if request is None or int(request.seq) != 0:
            raise ScenarioError("ArduPlane did not request the single transition loiter mission item")
        connection.mav.mission_item_int_send(
            target_system,
            target_component,
            0,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM,
            1,
            1,
            0.0,
            0.0,
            30.0,
            0.0,
            int(point[0] * 1e7),
            int(point[1] * 1e7),
            point[2],
            mission_type,
        )
        acknowledgement = connection.recv_match(type="MISSION_ACK", blocking=True, timeout=10)
        if acknowledgement is None or int(acknowledgement.type) != mavutil.mavlink.MAV_MISSION_ACCEPTED:
            result = None if acknowledgement is None else int(acknowledgement.type)
            raise ScenarioError(f"transition loiter mission was not accepted: {result}")
    finally:
        connection.close()


def wait_for_observed_mode_state(
    observer: RouteObserver, after: float, mode: int, vtol_state: int, timeout: float = 10.0
) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        modes = [item for item in observer.modes if item[0] >= after]
        states = [item for item in observer.vtol_states if item[0] >= after]
        if (
            modes
            and states
            and modes[-1][1:] == (mode, True)
            and states[-1][1] == vtol_state
            and time.monotonic() - modes[-1][0] <= 1.5
            and time.monotonic() - states[-1][0] <= 1.5
        ):
            return
        time.sleep(0.1)
    raise ScenarioError(f"independent observer did not prove armed mode={mode}, vtol_state={vtol_state}")


def transition_ready_samples(
    observer: RouteObserver, after: float, point: tuple[float, float, float]
) -> list[tuple[float, float, float, float, float]]:
    samples = []
    for position, velocity in zip(observer.positions, observer.velocities, strict=False):
        timestamp, latitude, longitude, altitude = position
        velocity_time, groundspeed, climb_rate = velocity
        if timestamp <= after or abs(timestamp - velocity_time) > 0.05:
            continue
        distance = distance_m((latitude, longitude), point[:2])
        if (
            distance <= READY_RADIUS_METERS
            and READY_MIN_ALTITUDE_METERS <= altitude <= READY_MAX_ALTITUDE_METERS
            and groundspeed <= READY_MAX_GROUNDSPEED_MPS
            and abs(climb_rate) <= READY_MAX_CLIMB_RATE_MPS
        ):
            sample = (timestamp, distance, altitude, groundspeed, climb_rate)
            candidate = [*samples, sample]
            altitudes = [value[2] for value in candidate]
            distances = [value[1] for value in candidate]
            groundspeeds = [value[3] for value in candidate]
            if (
                max(altitudes) - min(altitudes) > READY_MAX_ALTITUDE_VARIATION_METERS
                or max(distances) - min(distances) > READY_MAX_RADIAL_VARIATION_METERS
                or max(groundspeeds) - min(groundspeeds) > READY_MAX_GROUNDSPEED_VARIATION_MPS
            ):
                samples.clear()
            samples.append(sample)
        else:
            samples.clear()
    return samples


def transition_ready_diagnostic(observer: RouteObserver, after: float, point: tuple[float, float, float]) -> str:
    paired = []
    for position, velocity in zip(observer.positions, observer.velocities, strict=False):
        timestamp, latitude, longitude, altitude = position
        velocity_time, groundspeed, climb_rate = velocity
        if timestamp <= after or abs(timestamp - velocity_time) > 0.05:
            continue
        distance = distance_m((latitude, longitude), point[:2])
        paired.append((timestamp, distance, altitude, groundspeed, climb_rate))
    if not paired:
        return "paired_samples=0"

    radius_ok = sum(sample[1] <= READY_RADIUS_METERS for sample in paired)
    altitude_ok = sum(READY_MIN_ALTITUDE_METERS <= sample[2] <= READY_MAX_ALTITUDE_METERS for sample in paired)
    speed_ok = sum(sample[3] <= READY_MAX_GROUNDSPEED_MPS for sample in paired)
    climb_ok = sum(abs(sample[4]) <= READY_MAX_CLIMB_RATE_MPS for sample in paired)
    ready = transition_ready_samples(observer, after, point)
    recent = paired[-READY_SAMPLE_COUNT:]
    recent_speed_variation = max(sample[3] for sample in recent) - min(sample[3] for sample in recent)
    last = paired[-1]
    return (
        f"paired_samples={len(paired)} ready_samples={len(ready)} radius_ok={radius_ok} "
        f"altitude_ok={altitude_ok} speed_ok={speed_ok} climb_ok={climb_ok} "
        f"recent_speed_variation_mps={recent_speed_variation:.1f} "
        f"max_groundspeed_mps={max(sample[3] for sample in paired):.1f} "
        f"max_abs_climb_mps={max(abs(sample[4]) for sample in paired):.1f} "
        f"min_distance_m={min(sample[1] for sample in paired):.1f} "
        f"max_distance_m={max(sample[1] for sample in paired):.1f} "
        f"last_distance_m={last[1]:.1f} last_altitude_m={last[2]:.1f} "
        f"last_groundspeed_mps={last[3]:.1f} last_climb_mps={last[4]:.1f}"
    )


def has_current_transition_ready_window(
    samples: list[tuple[float, float, float, float, float]], observer: RouteObserver, started: float
) -> bool:
    if len(samples) < READY_SAMPLE_COUNT or samples[-1][0] - samples[0][0] < READY_DWELL_SECONDS:
        return False
    modes = [item for item in observer.modes if item[0] >= started]
    states = [item for item in observer.vtol_states if item[0] >= started]
    if not modes or modes[-1][1:] != (MODE_AUTO, True) or time.monotonic() - modes[-1][0] > 1.5:
        return False
    if not states or states[-1][1] != VTOL_STATE_FW or time.monotonic() - states[-1][0] > 1.5:
        return False
    return time.monotonic() - samples[-1][0] <= 1.5


def wait_for_transition_ready(
    binary: Path, port: str, observer: RouteObserver, started: float, point: tuple[float, float, float]
) -> tuple[dict[str, str], float, float, float, float]:
    deadline = time.monotonic() + READY_TIMEOUT_SECONDS
    stabilization_started: float | None = None
    while time.monotonic() < deadline:
        modes = [item for item in observer.modes if item[0] >= started]
        if not modes or modes[-1][1:] != (MODE_AUTO, True) or time.monotonic() - modes[-1][0] > 1.5:
            stabilization_started = None
            time.sleep(0.1)
            continue
        if stabilization_started is None:
            stabilization_started = modes[-1][0]
        samples = transition_ready_samples(observer, stabilization_started, point)
        if has_current_transition_ready_window(samples, observer, stabilization_started):
            status = read_status(binary, port)
            require_fresh_vtol_state(status, "fixed_wing")
            if status.get("mode") != str(MODE_AUTO) or status.get("armed") != "true":
                raise ScenarioError(f"transition-ready core state is not armed AUTO: {status}")
            return (
                status,
                time.monotonic() - started,
                max(sample[3] for sample in samples),
                max(abs(sample[4]) for sample in samples),
                max(sample[1] for sample in samples),
            )
        time.sleep(0.1)
    diagnostic = transition_ready_diagnostic(observer, started, point)
    raise ScenarioError("independent observer did not prove the stabilized transition-ready envelope: " + diagnostic)


def has_stable_multicopter_window(samples: list[tuple[float, float, float, float]]) -> bool:
    if len(samples) < VTOL_SAMPLE_COUNT or samples[-1][0] - samples[0][0] < VTOL_DWELL_SECONDS:
        return False
    spread = max(
        distance_m((left[1], left[2]), (right[1], right[2]))
        for index, left in enumerate(samples)
        for right in samples[index + 1 :]
    )
    altitude_range = max(row[3] for row in samples) - min(row[3] for row in samples)
    return spread <= VTOL_MAX_POSITION_SPREAD_METERS and altitude_range <= VTOL_MAX_ALTITUDE_VARIATION_METERS


def verify_post_transition(
    observer: RouteObserver, started: float, point: tuple[float, float, float]
) -> tuple[list[str], float]:
    states = [sample for sample in observer.vtol_states if sample[0] >= started]
    names = observed_state_names(states)
    allowed = {"fixed_wing", "transition_to_multicopter", "multicopter"}
    if not names or names[0] != "fixed_wing" or names[-1] != "multicopter" or not set(names) <= allowed:
        raise ScenarioError(f"independent observer saw unexpected transition state progression: {names}")

    stable = []
    for position, velocity in zip(observer.positions, observer.velocities, strict=False):
        timestamp, latitude, longitude, altitude = position
        velocity_time, groundspeed, climb_rate = velocity
        if timestamp < started or timestamp != velocity_time:
            continue
        state = next((value for state_time, value in reversed(states) if state_time <= timestamp), None)
        if state != VTOL_STATE_MC:
            stable.clear()
            continue
        if (
            distance_m((latitude, longitude), point[:2]) > READY_RADIUS_METERS
            or groundspeed > VTOL_MAX_GROUNDSPEED_MPS
            or abs(climb_rate) > VTOL_MAX_CLIMB_RATE_MPS
        ):
            stable.clear()
            continue
        stable.append((timestamp, latitude, longitude, altitude))
        while len(stable) > VTOL_SAMPLE_COUNT and stable[-1][0] - stable[1][0] > VTOL_DWELL_SECONDS:
            stable.pop(0)
        if has_stable_multicopter_window(stable):
            return names, time.monotonic() - started
    raise ScenarioError("independent observer did not prove a stable armed multicopter position")


def establish_transition_ready_state(
    binary: Path, port: str, observer: RouteObserver, point: tuple[float, float, float]
) -> tuple[float, float, float, float]:
    wait_for_observer_start(observer)
    if observer.modes[-1][1:] != (MODE_GUIDED, True) or observer.vtol_states[-1][1] != VTOL_STATE_FW:
        raise ScenarioError("independent observer did not confirm armed GUIDED fixed-wing recovery state")
    auto_requested = time.monotonic()
    upload_transition_loiter_mission(port, point)
    request_observed_mode(port, MODE_AUTO)
    wait_for_auto_and_multicopter(binary, port)
    wait_for_observed_mode_state(observer, auto_requested, MODE_AUTO, VTOL_STATE_MC)

    fixed_wing_setup_started = time.monotonic()
    setup_output = run_cli(binary, port, "transition-to-fixed-wing", timeout=TRANSITION_TIMEOUT_SECONDS)
    if "transition to fixed wing verified" not in setup_output:
        raise ScenarioError(f"NOMAD did not re-establish fixed-wing AUTO state: {setup_output!r}")
    wait_for_observed_mode_state(observer, fixed_wing_setup_started, MODE_AUTO, VTOL_STATE_FW)
    ready_started = time.monotonic()
    _ready_state, stabilization_time, ready_max_speed, ready_max_climb, ready_max_distance = wait_for_transition_ready(
        binary, port, observer, ready_started, point
    )
    return stabilization_time, ready_max_speed, ready_max_climb, ready_max_distance


def request_vtol_transition(
    binary: Path, port: str, observer: RouteObserver, point: tuple[float, float, float]
) -> tuple[list[str], float]:
    command_started = time.monotonic()
    output = run_cli(
        binary,
        port,
        "transition-to-vtol",
        *(str(value) for value in point),
        timeout=TRANSITION_TIMEOUT_SECONDS + 20,
    )
    if "transition to VTOL verified" not in output:
        raise ScenarioError(f"NOMAD did not report verified multicopter transition: {output!r}")
    return verify_post_transition(observer, command_started, point)


def get_transition_point_from_recovery(
    binary: Path, port: str, recovery_point: tuple[float, float, float]
) -> tuple[tuple[float, float, float], float, float]:
    status = read_status(binary, port)
    require_fresh_vtol_state(status, "fixed_wing")
    if status.get("mode") != str(MODE_GUIDED) or status.get("armed") != "true":
        raise ScenarioError(f"recovery must end in armed GUIDED fixed-wing state: {status}")
    recovery_distance = distance_m(parse_position(status), recovery_point[:2])
    recovered_altitude = parse_relative_altitude(status)
    altitude_error = abs(recovered_altitude - recovery_point[2])
    if not READY_MIN_ALTITUDE_METERS <= recovered_altitude <= READY_MAX_ALTITUDE_METERS:
        raise ScenarioError(
            f"recovered altitude is outside the reviewed 15-25 m transition band: {recovered_altitude:.1f} m"
        )
    return recovery_point, recovery_distance, altitude_error


def execute_transition_to_vtol(
    binary: Path, port: str, observer: RouteObserver, point: tuple[float, float, float]
) -> tuple[list[str], float, float, float, float, float, dict[str, str], float, float, float]:
    transition_point, recovery_distance, altitude_error = get_transition_point_from_recovery(binary, port, point)
    observer.start()
    try:
        stabilization_time, ready_max_speed, ready_max_climb, ready_max_distance = establish_transition_ready_state(
            binary, port, observer, transition_point
        )
        names, observed_completion_time = request_vtol_transition(binary, port, observer, transition_point)
    finally:
        observer.stop()

    final = read_status(binary, port)
    require_fresh_vtol_state(final, "multicopter")
    if final.get("mode") != str(MODE_AUTO) or final.get("armed") != "true":
        raise ScenarioError(f"transition did not retain armed AUTO multicopter state: {final}")
    return (
        names,
        stabilization_time,
        ready_max_speed,
        ready_max_climb,
        ready_max_distance,
        transition_point[2],
        final,
        observed_completion_time,
        recovery_distance,
        altitude_error,
    )


def run_transition_qualification(
    binary: Path,
) -> tuple[list[str], float, float, float, float, float, dict[str, str], float, float, float]:
    port = get_sitl_port()
    start_position, waypoints = prepare_route(binary, port)
    route_observer = RouteObserver(OBSERVER_PORT)
    route_observer.start()
    try:
        require_route_setup(route_observer)
        execute_and_observe_route(binary, port, route_observer, start_position, waypoints)
    finally:
        route_observer.stop()
    point = recovery_point(binary, port, start_position, waypoints)
    execute_recovery(binary, port, point)
    return execute_transition_to_vtol(binary, port, RouteObserver(OBSERVER_PORT), point)


def report_transition_qualification(
    result: tuple[list[str], float, float, float, float, float, dict[str, str], float, float, float],
) -> None:
    (
        states,
        stabilization,
        max_speed,
        max_climb,
        max_distance,
        transition_altitude,
        final,
        completion,
        recovery_distance,
        altitude_error,
    ) = result
    print(
        "QuadPlane fixed-wing-to-VTOL transition qualification passed: "
        f"recovery_mode=GUIDED transition_mode=AUTO pre_vtol_state=fixed_wing "
        f"recovery_distance_m={recovery_distance:.1f} recovery_altitude_error_m={altitude_error:.1f} "
        f"transition_point_altitude_m={transition_altitude:.1f} "
        f"transition_ready_max_distance_m={max_distance:.1f} "
        f"stabilization_time_s={stabilization:.1f} command=MAV_CMD_DO_VTOL_TRANSITION "
        f"stabilization_max_groundspeed_mps={max_speed:.1f} "
        f"stabilization_max_abs_climb_mps={max_climb:.1f} target=MAV_VTOL_STATE_MC "
        f"observed_states={states} completion_time_s={completion:.1f} "
        f"final_mode={final['mode']} final_armed={final['armed']}"
    )


def main() -> int:
    binary = find_binary()
    if binary is None:
        print("error: qualification driver not found; run `pixi run build-qualification-cli` first", file=sys.stderr)
        return 2
    try:
        report_transition_qualification(run_transition_qualification(binary))
        return 0
    except subprocess.TimeoutExpired as error:
        print(f"error: CLI command exceeded its {error.timeout}s deadline", file=sys.stderr)
    except (ValueError, ScenarioError) as error:
        print(f"error: {error}", file=sys.stderr)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
