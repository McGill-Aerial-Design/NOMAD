# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Qualify a two-point fixed-wing GUIDED route on the pinned QuadPlane SITL."""

from __future__ import annotations

import math
import subprocess
import sys
import threading
import time
from pathlib import Path

from core_sitl_quadplane_observe import ScenarioError, get_sitl_port, read_status, run_cli
from core_sitl_quadplane_transition import (
    MODE_AUTO,
    OBSERVER_PORT,
    VTOL_STATE_FW,
    complete_transition,
    parse_position,
    parse_relative_altitude,
    prepare_transition,
    require_fresh_vtol_state,
)
from core_sitl_status import find_binary
from pymavlink import mavutil

ARRIVAL_RADIUS_METERS = 45.0
ROUTE_TIMEOUT_SECONDS = 180
EARTH_RADIUS_METERS = 6_371_000.0
MODE_GUIDED = 15
VTOL_STATE_NAMES = {
    mavutil.mavlink.MAV_VTOL_STATE_MC: "multicopter",
    mavutil.mavlink.MAV_VTOL_STATE_TRANSITION_TO_FW: "transition_to_fixed_wing",
    mavutil.mavlink.MAV_VTOL_STATE_TRANSITION_TO_MC: "transition_to_multicopter",
    VTOL_STATE_FW: "fixed_wing",
}


class RouteObserver:
    """Collect independent heartbeat, position and VTOL telemetry samples."""

    def __init__(self, port: int) -> None:
        self._port = port
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self.positions: list[tuple[float, float, float, float]] = []
        self.velocities: list[tuple[float, float, float]] = []
        self.modes: list[tuple[float, int, bool]] = []
        self.vtol_states: list[tuple[float, int]] = []
        self.landed_states: list[tuple[float, int]] = []

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
                message = connection.recv_match(
                    type=["GLOBAL_POSITION_INT", "HEARTBEAT", "EXTENDED_SYS_STATE"], blocking=True, timeout=0.5
                )
                if message is None:
                    continue
                observed_at = time.monotonic()
                kind = message.get_type()
                if kind == "GLOBAL_POSITION_INT":
                    self.positions.append(
                        (observed_at, message.lat / 1e7, message.lon / 1e7, message.relative_alt / 1000.0)
                    )
                    self.velocities.append(
                        (observed_at, math.hypot(message.vx, message.vy) / 100.0, -message.vz / 100.0)
                    )
                elif kind == "HEARTBEAT":
                    is_armed = bool(message.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    self.modes.append((observed_at, int(message.custom_mode), is_armed))
                else:
                    self.vtol_states.append((observed_at, int(message.vtol_state)))
                    self.landed_states.append((observed_at, int(message.landed_state)))
        finally:
            connection.close()


def distance_m(point_a: tuple[float, float], point_b: tuple[float, float]) -> float:
    latitude_a, longitude_a = map(math.radians, point_a)
    latitude_b, longitude_b = map(math.radians, point_b)
    latitude_delta = latitude_b - latitude_a
    longitude_delta = longitude_b - longitude_a
    haversine = (
        math.sin(latitude_delta / 2.0) ** 2
        + math.cos(latitude_a) * math.cos(latitude_b) * math.sin(longitude_delta / 2.0) ** 2
    )
    return 2.0 * EARTH_RADIUS_METERS * math.asin(math.sqrt(min(1.0, haversine)))


def route_waypoints(fields: dict[str, str]) -> tuple[tuple[float, float, float], ...]:
    latitude, longitude = parse_position(fields)
    altitude = max(8.0, parse_relative_altitude(fields))
    if not math.isfinite(altitude) or altitude > 100.0:
        raise ScenarioError(f"starting relative altitude is outside the route limits: {altitude}")
    try:
        _roll, _pitch, heading_degrees = map(float, fields["attitude"].split(","))
    except (KeyError, ValueError) as error:
        raise ScenarioError(f"missing or invalid heading; full status={fields}") from error

    heading = math.radians(heading_degrees)
    forward_north, forward_east = math.cos(heading), math.sin(heading)
    right_north, right_east = -forward_east, forward_north
    meters_per_degree_lat = 111_111.0
    meters_per_degree_lon = meters_per_degree_lat * math.cos(math.radians(latitude))
    offsets = ((220.0, 0.0), (220.0, 160.0))
    waypoints = []
    for forward_m, right_m in offsets:
        north_m = forward_north * forward_m + right_north * right_m
        east_m = forward_east * forward_m + right_east * right_m
        waypoints.append(
            (
                latitude + north_m / meters_per_degree_lat,
                longitude + east_m / meters_per_degree_lon,
                altitude,
            )
        )
    return tuple(waypoints)


def wait_for_observer_start(observer: RouteObserver, timeout: float = 10.0) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if observer.positions and observer.modes and observer.vtol_states:
            latest_samples = (observer.positions[-1][0], observer.modes[-1][0], observer.vtol_states[-1][0])
            if all(time.monotonic() - timestamp <= 1.5 for timestamp in latest_samples):
                return
        time.sleep(0.1)
    raise ScenarioError("independent route observer did not receive position, heartbeat and VTOL telemetry")


def verify_observed_route(
    observer: RouteObserver, start_position: tuple[float, float], waypoints: tuple[tuple[float, float, float], ...]
) -> tuple[list[int], list[float]]:
    progress: list[int] = []
    final_distances: list[float] = []
    last_arrival_time = 0.0
    for index, waypoint in enumerate(waypoints, start=1):
        start_distance = distance_m(start_position, waypoint[:2])
        if start_distance <= ARRIVAL_RADIUS_METERS + 10.0:
            raise ScenarioError(f"waypoint {index} is too close to the route start to prove navigation")
        candidates = [
            (timestamp, distance_m((latitude, longitude), waypoint[:2]))
            for timestamp, latitude, longitude, _altitude in observer.positions
            if timestamp > last_arrival_time
        ]
        if not candidates:
            raise ScenarioError(f"independent observer captured no position samples for waypoint {index}")
        arrival = next((sample for sample in candidates if sample[1] <= ARRIVAL_RADIUS_METERS), None)
        if arrival is None:
            closest = min(distance for _timestamp, distance in candidates)
            raise ScenarioError(
                f"independent observer did not see waypoint {index} within {ARRIVAL_RADIUS_METERS:.0f} m; "
                f"closest={closest:.1f} m"
            )
        last_arrival_time = arrival[0]
        progress.append(index)
        final_distances.append(arrival[1])
    return progress, final_distances


def prepare_route(binary: Path, port: str) -> tuple[tuple[float, float], tuple[tuple[float, float, float], ...]]:
    transition_observer, _starting_state, _baseline, _takeoff_state, _pre_transition = prepare_transition(binary, port)
    complete_transition(binary, port, transition_observer)
    starting = read_status(binary, port)
    require_fresh_vtol_state(starting, "fixed_wing")
    if starting.get("mode") != str(MODE_AUTO) or starting.get("armed") != "true":
        raise ScenarioError(f"route setup requires armed AUTO after transition; status={starting}")
    return parse_position(starting), route_waypoints(starting)


def require_route_setup(observer: RouteObserver) -> None:
    wait_for_observer_start(observer)
    if observer.vtol_states[-1][1] != VTOL_STATE_FW:
        raise ScenarioError("independent observer did not confirm fixed-wing transition state")
    if observer.modes[-1][1:] != (MODE_AUTO, True):
        raise ScenarioError("independent observer did not confirm armed AUTO route setup")


def execute_and_observe_route(
    binary: Path,
    port: str,
    observer: RouteObserver,
    start_position: tuple[float, float],
    waypoints: tuple[tuple[float, float, float], ...],
) -> tuple[float, list[int], list[float], float]:
    command_started = time.monotonic()
    arguments = tuple(str(value) for waypoint in waypoints for value in waypoint)
    output = run_cli(binary, port, "fixed-wing-route", *arguments, timeout=ROUTE_TIMEOUT_SECONDS + 20)
    completion_time = time.monotonic() - command_started
    if "fixed-wing route verified: waypoints=2" not in output:
        raise ScenarioError(f"NOMAD did not report verified two-point route: {output!r}")
    observer.stop()
    guided_times = [
        timestamp
        for timestamp, mode, armed in observer.modes
        if timestamp >= command_started and mode == MODE_GUIDED and armed
    ]
    if not guided_times:
        raise ScenarioError("independent observer did not confirm armed GUIDED route execution")
    if not any(timestamp >= command_started and state == VTOL_STATE_FW for timestamp, state in observer.vtol_states):
        raise ScenarioError("independent observer did not retain fixed-wing state during route execution")
    observer.positions = [sample for sample in observer.positions if sample[0] >= guided_times[0]]
    progress, distances = verify_observed_route(observer, start_position, waypoints)
    final_distance = distances[-1]
    final = read_status(binary, port)
    require_fresh_vtol_state(final, "fixed_wing")
    if final.get("mode") != str(MODE_GUIDED) or final.get("armed") != "true":
        raise ScenarioError(f"route completion lost armed GUIDED state; status={final}")
    return completion_time, progress, distances, final_distance


def report_route_result(
    start_position: tuple[float, float],
    waypoints: tuple[tuple[float, float, float], ...],
    completion_time: float,
    progress: list[int],
    distances: list[float],
    final_distance: float,
) -> None:
    print(
        "QuadPlane fixed-wing route qualification passed: "
        f"route_points={len(waypoints)} starting_mode=AUTO starting_vtol_state=fixed_wing route_mode=GUIDED "
        f"route_start_position={start_position[0]:.7f},{start_position[1]:.7f} "
        f"waypoint_1={waypoints[0][0]:.7f},{waypoints[0][1]:.7f},{waypoints[0][2]:.1f} "
        f"waypoint_2={waypoints[1][0]:.7f},{waypoints[1][1]:.7f},{waypoints[1][2]:.1f} "
        f"observed_progress={progress} observed_arrival_distances_m=[{distances[0]:.1f},{distances[1]:.1f}] "
        f"final_distance_m={final_distance:.1f} completion_time_s={completion_time:.1f}"
    )


def main() -> int:
    binary = find_binary()
    if binary is None:
        print("error: qualification driver not found; run `pixi run build-qualification-cli` first", file=sys.stderr)
        return 2
    observer: RouteObserver | None = None
    try:
        port = get_sitl_port()
        start_position, waypoints = prepare_route(binary, port)
        observer = RouteObserver(OBSERVER_PORT)
        observer.start()
        require_route_setup(observer)
        result = execute_and_observe_route(binary, port, observer, start_position, waypoints)
    except subprocess.TimeoutExpired as error:
        print(f"error: CLI command exceeded its {error.timeout}s deadline", file=sys.stderr)
        return 1
    except (ValueError, ScenarioError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 1
    finally:
        if observer is not None:
            observer.stop()
    report_route_result(start_position, waypoints, *result)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
