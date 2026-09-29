# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Qualify an explicit fixed-wing recovery point after the pinned two-point route."""

from __future__ import annotations

import subprocess
import sys
import time

from core_sitl_quadplane_observe import ScenarioError, get_sitl_port, read_status, run_cli
from core_sitl_quadplane_route import (
    ARRIVAL_RADIUS_METERS,
    MODE_GUIDED,
    RouteObserver,
    distance_m,
    execute_and_observe_route,
    prepare_route,
    wait_for_observer_start,
)
from core_sitl_quadplane_transition import OBSERVER_PORT, VTOL_STATE_FW, parse_position, require_fresh_vtol_state
from core_sitl_status import find_binary

RECOVERY_TIMEOUT_SECONDS = 180
ALTITUDE_TOLERANCE_METERS = 5.0
REQUIRED_PROGRESS_METERS = 10.0


def observe_recovery(observer: RouteObserver, started_at: float, point: tuple[float, float, float]) -> tuple:
    samples = [
        (timestamp, distance_m((latitude, longitude), point[:2]), altitude)
        for timestamp, latitude, longitude, altitude in observer.positions
        if timestamp > started_at
    ]
    if len(samples) < 2:
        raise ScenarioError("independent recovery observer captured fewer than two post-command positions")
    start_distance = samples[0][1]
    arrival = next(
        (
            sample
            for sample in samples[1:]
            if start_distance - sample[1] >= REQUIRED_PROGRESS_METERS
            and sample[1] <= ARRIVAL_RADIUS_METERS
            and abs(sample[2] - point[2]) <= ALTITUDE_TOLERANCE_METERS
        ),
        None,
    )
    if arrival is None:
        raise ScenarioError(f"observer did not prove recovery progress and arrival; distances={samples}")
    if not any(timestamp > started_at and mode == MODE_GUIDED and armed for timestamp, mode, armed in observer.modes):
        raise ScenarioError("independent observer did not retain armed GUIDED mode during recovery")
    if not any(timestamp > started_at and state == VTOL_STATE_FW for timestamp, state in observer.vtol_states):
        raise ScenarioError("independent observer did not retain fixed-wing state during recovery")
    return start_distance, min(sample[1] for sample in samples), arrival, samples


def recovery_point(binary, port, start_position, waypoints) -> tuple[float, float, float]:
    before = read_status(binary, port)
    require_fresh_vtol_state(before, "fixed_wing")
    if before.get("mode") != str(MODE_GUIDED) or before.get("armed") != "true":
        raise ScenarioError(f"route did not leave armed GUIDED fixed-wing state: {before}")
    point = (*start_position, max(20.0, waypoints[-1][2]))
    distance_before = distance_m(parse_position(before), point[:2])
    if distance_before <= ARRIVAL_RADIUS_METERS + REQUIRED_PROGRESS_METERS:
        raise ScenarioError(f"recovery target is too near after route: {distance_before:.1f} m")
    return point


def report_recovery(point, start_distance, minimum_distance, arrival, samples, completed_in) -> None:
    progress = [round(sample[1], 1) for sample in samples[:: max(1, len(samples) // 8)]]
    print(
        "QuadPlane fixed-wing recovery qualification passed: "
        f"command=MAV_CMD_DO_REPOSITION mode=GUIDED vtol_state=FW "
        f"target={point[0]:.7f},{point[1]:.7f},{point[2]:.1f} "
        f"start_distance_m={start_distance:.1f} progress_samples_m={progress} "
        f"minimum_distance_m={minimum_distance:.1f} completion_distance_m={arrival[1]:.1f} "
        f"altitude_error_m={abs(arrival[2] - point[2]):.1f} completion_time_s={completed_in:.1f}"
    )


def execute_recovery(binary, port, point) -> None:
    observer = RouteObserver(OBSERVER_PORT)
    observer.start()
    try:
        wait_for_observer_start(observer)
        started_at = time.monotonic()
        output = run_cli(
            binary, port, "fixed-wing-recovery", *(str(value) for value in point), timeout=RECOVERY_TIMEOUT_SECONDS + 20
        )
        completed_in = time.monotonic() - started_at
        if "fixed-wing recovery verified: recovery region reached" not in output:
            raise ScenarioError(f"NOMAD did not verify the recovery region: {output!r}")
        observer.stop()
        start_distance, minimum_distance, arrival, samples = observe_recovery(observer, started_at, point)
        final = read_status(binary, port)
        require_fresh_vtol_state(final, "fixed_wing")
        if final.get("mode") != str(MODE_GUIDED) or final.get("armed") != "true":
            raise ScenarioError(f"recovery completion lost armed GUIDED fixed-wing state: {final}")
        report_recovery(point, start_distance, minimum_distance, arrival, samples, completed_in)
    finally:
        observer.stop()


def main() -> int:
    binary = find_binary()
    if binary is None:
        print("error: qualification driver not found; run `pixi run build-qualification-cli` first", file=sys.stderr)
        return 2
    route_observer = None
    try:
        port = get_sitl_port()
        start_position, waypoints = prepare_route(binary, port)
        route_observer = RouteObserver(OBSERVER_PORT)
        route_observer.start()
        wait_for_observer_start(route_observer)
        execute_and_observe_route(binary, port, route_observer, start_position, waypoints)
        point = recovery_point(binary, port, start_position, waypoints)
        execute_recovery(binary, port, point)
        return 0
    except subprocess.TimeoutExpired as error:
        print(f"error: CLI command exceeded its {error.timeout}s deadline", file=sys.stderr)
    except (ValueError, ScenarioError) as error:
        print(f"error: {error}", file=sys.stderr)
    finally:
        if route_observer is not None:
            route_observer.stop()
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
