# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Qualify the full pinned QuadPlane chain through NOMAD's QLAND operation."""

from __future__ import annotations

import subprocess
import sys
import time
from pathlib import Path

from core_sitl_quadplane_observe import ScenarioError, get_sitl_port, read_status, run_cli
from core_sitl_quadplane_route import RouteObserver, distance_m, wait_for_observer_start
from core_sitl_quadplane_transition import (
    MODE_AUTO,
    OBSERVER_PORT,
    VTOL_STATE_MC,
    parse_position,
    require_fresh_vtol_state,
)
from core_sitl_quadplane_transition_back import report_transition_qualification, run_transition_qualification
from core_sitl_status import find_binary
from pymavlink import mavutil

LANDING_TIMEOUT_SECONDS = 120
READY_RADIUS_METERS = 5.0
READY_MIN_ALTITUDE_METERS = 15.0
READY_MAX_ALTITUDE_METERS = 25.0
READY_MAX_GROUNDSPEED_MPS = 1.0
READY_MAX_CLIMB_RATE_MPS = 0.25
READY_SAMPLE_COUNT = 5
READY_DWELL_SECONDS = 2.0
FINAL_RADIUS_METERS = 5.0
FINAL_POSITION_SPREAD_METERS = 1.5
FINAL_MIN_ALTITUDE_METERS = -1.0
FINAL_MAX_ALTITUDE_METERS = 1.5
FINAL_MAX_GROUNDSPEED_MPS = 0.5
FINAL_MAX_CLIMB_RATE_MPS = 0.2
DESCENT_EVIDENCE_METERS = 5.0
LANDED_STATE_NAMES = {
    mavutil.mavlink.MAV_LANDED_STATE_UNDEFINED: "unknown",
    mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND: "on_ground",
    mavutil.mavlink.MAV_LANDED_STATE_IN_AIR: "in_air",
    mavutil.mavlink.MAV_LANDED_STATE_TAKEOFF: "taking_off",
    mavutil.mavlink.MAV_LANDED_STATE_LANDING: "landing",
}


def latest_before(samples, timestamp, maximum_age=1.5):
    matches = [sample for sample in samples if sample[0] <= timestamp]
    if not matches or timestamp - matches[-1][0] > maximum_age:
        return None
    return matches[-1]


def is_landing_ready_sample(observer, position, velocity, started, landing_point):
    timestamp, latitude, longitude, altitude = position
    velocity_time, groundspeed, climb_rate = velocity
    if timestamp < started or abs(timestamp - velocity_time) > 0.05:
        return False
    mode = latest_before(observer.modes, timestamp)
    vtol = latest_before(observer.vtol_states, timestamp)
    landed = latest_before(observer.landed_states, timestamp)
    return (
        mode is not None
        and mode[1:] == (MODE_AUTO, True)
        and vtol is not None
        and vtol[1] == VTOL_STATE_MC
        and landed is not None
        and landed[1] == mavutil.mavlink.MAV_LANDED_STATE_IN_AIR
        and READY_MIN_ALTITUDE_METERS <= altitude <= READY_MAX_ALTITUDE_METERS
        and groundspeed <= READY_MAX_GROUNDSPEED_MPS
        and abs(climb_rate) <= READY_MAX_CLIMB_RATE_MPS
        and distance_m((latitude, longitude), landing_point) <= READY_RADIUS_METERS
    )


def has_stable_ready_envelope(samples):
    if len(samples) < READY_SAMPLE_COUNT or samples[-1][0] - samples[0][0] < READY_DWELL_SECONDS:
        return False
    if time.monotonic() - samples[-1][0] > 1.5:
        return False
    position_spread = max(
        distance_m((first[1], first[2]), (second[1], second[2]))
        for index, first in enumerate(samples)
        for second in samples[index + 1 :]
    )
    altitude_spread = max(sample[3] for sample in samples) - min(sample[3] for sample in samples)
    return position_spread <= FINAL_POSITION_SPREAD_METERS and altitude_spread <= 1.0


def trim_samples_to_dwell(samples):
    while len(samples) > READY_SAMPLE_COUNT and samples[-1][0] - samples[1][0] >= READY_DWELL_SECONDS:
        samples.pop(0)


def wait_for_landing_ready(observer: RouteObserver, started: float, landing_point: tuple[float, float]):
    deadline = time.monotonic() + 45.0
    while time.monotonic() < deadline:
        ready_samples = []
        for position, velocity in zip(observer.positions, observer.velocities, strict=False):
            timestamp, latitude, longitude, altitude = position
            if not is_landing_ready_sample(observer, position, velocity, started, landing_point):
                ready_samples.clear()
                continue
            ready_samples.append((timestamp, latitude, longitude, altitude, velocity[1], velocity[2]))
        trim_samples_to_dwell(ready_samples)
        if has_stable_ready_envelope(ready_samples):
            mode = latest_before(observer.modes, ready_samples[-1][0])
            vtol = latest_before(observer.vtol_states, ready_samples[-1][0])
            landed = latest_before(observer.landed_states, ready_samples[-1][0])
            if mode and vtol and landed and mode[1:] == (MODE_AUTO, True) and vtol[1] == VTOL_STATE_MC:
                return ready_samples
        time.sleep(0.1)
    recent_samples = describe_recent_landing_samples(observer, started, landing_point)
    raise ScenarioError(
        f"independent observer did not prove the two-second landing-ready dwell; recent_samples={recent_samples}"
    )


def describe_recent_landing_samples(observer, started, landing_point):
    samples = []
    for position, velocity in zip(observer.positions, observer.velocities, strict=False):
        timestamp, latitude, longitude, altitude = position
        velocity_time, groundspeed, climb_rate = velocity
        if timestamp < started:
            continue
        mode = latest_before(observer.modes, timestamp)
        vtol = latest_before(observer.vtol_states, timestamp)
        landed = latest_before(observer.landed_states, timestamp)
        mode_state = None if mode is None else (mode[1], mode[2])
        vtol_state = None if vtol is None else vtol[1]
        landed_state = None if landed is None else landed[1]
        point_distance = distance_m((latitude, longitude), landing_point)
        samples.append(
            f"age_s={time.monotonic() - timestamp:.2f} position_velocity_skew_s={abs(timestamp - velocity_time):.2f} "
            f"altitude_m={altitude:.2f} landing_distance_m={point_distance:.2f} "
            f"groundspeed_mps={groundspeed:.2f} climb_rate_mps={climb_rate:.2f} "
            f"mode_armed={mode_state} vtol={vtol_state} landed={landed_state}"
        )
    return samples[-5:]


def verify_landing_mode(modes, started):
    observed = [sample for sample in modes if sample[0] >= started]
    if not observed:
        return False, None
    qland = [sample for sample in observed if sample[1] == 20]
    if any(sample[1] not in (MODE_AUTO, 20) for sample in observed):
        raise ScenarioError(f"independent observer saw unexpected landing modes {observed}")
    if qland and observed[-1][1] != 20:
        raise ScenarioError(f"independent observer saw QLAND exit to mode {observed[-1][1]}")
    return bool(qland), qland[0][0] if qland else None


def find_descent_time(positions, qland_seen_at, entry_altitude):
    if qland_seen_at is None:
        return None
    for sample in positions:
        if sample[0] >= qland_seen_at and entry_altitude - sample[3] >= DESCENT_EVIDENCE_METERS:
            return sample[0]
    return None


def find_touchdown_time(landed_states, descent_seen_at):
    if descent_seen_at is None:
        return None
    for timestamp, landed_state in landed_states:
        if timestamp >= descent_seen_at and landed_state == mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND:
            return timestamp
    return None


def collect_stable_final_samples(observer, started, landed_at, landing_point):
    candidate = []
    for position, velocity in zip(observer.positions, observer.velocities, strict=False):
        timestamp, latitude, longitude, altitude = position
        velocity_time, groundspeed, climb_rate = velocity
        if timestamp < max(landed_at, started) or abs(timestamp - velocity_time) > 0.05:
            continue
        mode = latest_before(observer.modes, timestamp)
        vtol = latest_before(observer.vtol_states, timestamp)
        landed = latest_before(observer.landed_states, timestamp)
        if (
            mode is None
            or mode[1:] != (20, False)
            or vtol is None
            or vtol[1] != VTOL_STATE_MC
            or landed is None
            or landed[1] != mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND
            or not FINAL_MIN_ALTITUDE_METERS <= altitude <= FINAL_MAX_ALTITUDE_METERS
            or groundspeed > FINAL_MAX_GROUNDSPEED_MPS
            or abs(climb_rate) > FINAL_MAX_CLIMB_RATE_MPS
            or distance_m((latitude, longitude), landing_point) > FINAL_RADIUS_METERS
        ):
            candidate.clear()
            continue
        candidate.append((timestamp, latitude, longitude, altitude, groundspeed, climb_rate))
        trim_samples_to_dwell(candidate)
    return candidate


def has_stable_final_envelope(candidate):
    if len(candidate) < READY_SAMPLE_COUNT or candidate[-1][0] - candidate[0][0] < READY_DWELL_SECONDS:
        return False
    if time.monotonic() - candidate[-1][0] > 1.5:
        return False
    position_spread = max(
        distance_m((first[1], first[2]), (second[1], second[2]))
        for index, first in enumerate(candidate)
        for second in candidate[index + 1 :]
    )
    altitude_spread = max(sample[3] for sample in candidate) - min(sample[3] for sample in candidate)
    return position_spread <= FINAL_POSITION_SPREAD_METERS and altitude_spread <= 0.5


def verify_landing_trace(
    observer: RouteObserver, started: float, landing_point: tuple[float, float], entry_altitude: float
):
    deadline = started + LANDING_TIMEOUT_SECONDS
    qland_seen = False
    qland_seen_at = None
    descent_seen_at = None
    landed_at = None
    while time.monotonic() < deadline:
        modes = [sample for sample in observer.modes if sample[0] >= started]
        states = [sample for sample in observer.vtol_states if sample[0] >= started]
        landed_states = [sample for sample in observer.landed_states if sample[0] >= started]
        positions = [sample for sample in observer.positions if sample[0] >= started]
        if states and states[-1][1] != VTOL_STATE_MC:
            raise ScenarioError(f"independent observer saw unexpected VTOL state {states[-1]}")
        qland_seen, observed_qland_time = verify_landing_mode(modes, started)
        qland_seen_at = qland_seen_at or observed_qland_time
        descent_seen_at = descent_seen_at or find_descent_time(positions, qland_seen_at, entry_altitude)
        landed_at = landed_at or find_touchdown_time(landed_states, descent_seen_at)
        if landed_at is not None:
            candidate = collect_stable_final_samples(observer, started, landed_at, landing_point)
            if has_stable_final_envelope(candidate):
                return qland_seen, descent_seen_at, landed_at, candidate
        time.sleep(0.1)
    raise ScenarioError(
        "independent observer did not verify QLAND descent, ON_GROUND, disarm, and stable final state: "
        f"qland={qland_seen} descent_at={descent_seen_at} on_ground_at={landed_at}"
    )


def prepare_landing_entry(binary: Path, port: str) -> tuple[float, float]:
    chain = run_transition_qualification(binary)
    report_transition_qualification(chain)
    starting = read_status(binary, port)
    require_fresh_vtol_state(starting, "multicopter")
    if starting.get("mode") != str(MODE_AUTO) or starting.get("armed") != "true":
        raise ScenarioError(f"landing chain did not end in armed AUTO multicopter state: {starting}")
    return parse_position(starting)


def invoke_landing_and_observe(binary, port, observer, landing_point):
    wait_for_observer_start(observer)
    ready_started = time.monotonic()
    ready_samples = wait_for_landing_ready(observer, ready_started, landing_point)
    landing_distance = distance_m((ready_samples[-1][1], ready_samples[-1][2]), landing_point)
    start_altitude = ready_samples[-1][3]
    command_started = time.monotonic()
    output = run_cli(
        binary,
        port,
        "quadplane-vtol-land",
        str(landing_point[0]),
        str(landing_point[1]),
        timeout=LANDING_TIMEOUT_SECONDS + 15,
    )
    if "landing verified by post-command descent" not in output:
        raise ScenarioError(f"NOMAD did not report verified physical landing: {output!r}")
    trace = verify_landing_trace(observer, command_started, landing_point, start_altitude)
    readiness_dwell = ready_samples[-1][0] - ready_samples[0][0]
    return start_altitude, landing_distance, readiness_dwell, command_started, trace


def report_landing_qualification(binary, port, observer, evidence):
    start_altitude, landing_distance, readiness_dwell, command_started, trace = evidence
    qland_seen, descent_seen_at, landed_at, final_samples = trace
    final = read_status(binary, port)
    require_fresh_vtol_state(final, "multicopter")
    final_sample = final_samples[-1]
    states = [value for timestamp, value in observer.landed_states if timestamp >= command_started]
    state_names = [LANDED_STATE_NAMES.get(value, f"unknown_{value}") for value in states]
    print(
        "Pinned QuadPlane VTOL landing qualification passed: "
        f"firmware=ArduPlane-4.7.1 sha=dbe792162d06cab66c3475fd5556bf7a120f119e "
        f"command=DO_SET_MODE(custom_mode=20/QLAND) start_altitude_m={start_altitude:.2f} "
        f"landing_point_distance_m={landing_distance:.2f} readiness_dwell_s={readiness_dwell:.2f} "
        f"qland_observed={qland_seen} descent_duration_s={descent_seen_at - command_started:.2f} "
        f"touchdown_time_s={landed_at - command_started:.2f} landed_states={state_names} "
        f"final_mode={final.get('mode')} final_armed={final.get('armed')} final_vtol_state=multicopter "
        f"final_landed_state=on_ground final_altitude_m={final_sample[3]:.2f} "
        f"final_groundspeed_mps={final_sample[4]:.2f} final_climb_rate_mps={final_sample[5]:.2f}"
    )


def run_landing_qualification(binary: Path, port: str) -> None:
    landing_point = prepare_landing_entry(binary, port)

    observer = RouteObserver(OBSERVER_PORT)
    observer.start()
    try:
        evidence = invoke_landing_and_observe(binary, port, observer, landing_point)
    finally:
        observer.stop()
    report_landing_qualification(binary, port, observer, evidence)


def main() -> int:
    binary = find_binary()
    if binary is None:
        print("error: qualification driver not found; run `pixi run build-qualification-cli` first", file=sys.stderr)
        return 2
    try:
        run_landing_qualification(binary, get_sitl_port())
    except subprocess.TimeoutExpired as error:
        print(f"error: CLI command exceeded its {error.timeout}s deadline", file=sys.stderr)
        return 1
    except (ValueError, ScenarioError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
