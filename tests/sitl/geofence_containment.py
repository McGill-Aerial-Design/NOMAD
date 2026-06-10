# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""SITL containment scenario for the NOMAD-side geofence (SR-FEN-02, H-05).

Drives a real ArduPilot SITL vehicle and exercises the geofence enforcement in
``MavlinkCommands`` (Tier SC) against it end-to-end:

  read home -> configure a keep-in fence box around it
    -> arm -> GUIDED -> takeoff
    -> global target INSIDE the fence   (accepted; vehicle actually moves)
    -> global target OUTSIDE the fence  (rejected; vehicle stays contained)
    -> local NED target OUTSIDE         (rejected)

Two independent MAVLink connections are used: an "operator" link that arms /
mode-switches / observes, and the MavlinkService under test on its own link.

Run standalone inside the dev network (see tests/sitl/README.md):

    NOMAD_SITL_OPERATOR=tcp:host:5762 NOMAD_SITL_CONTROLLER=tcp:host:5763 \
        python tests/sitl/geofence_containment.py
"""

from __future__ import annotations

import math
import os
import sys
import threading
import time
from dataclasses import dataclass, field

from pymavlink import mavutil

sys.path.insert(0, os.path.dirname(__file__))
from velocity_loop_closure import (  # noqa: E402
    ScenarioError,
    _command_long,
    _log,
    _set_mode,
    _wait_until,
)

from edge_core.services.geospatial import GPSCoordinate, calculate_gps_offset_meters  # noqa: E402
from edge_core.services.mavlink import MavlinkService  # noqa: E402
from edge_core.services.state import StateManager  # noqa: E402

# Keep-in box half-extent around home (m) and margin (m). The out-of-fence
# target is well beyond the box; the in-fence target well within it.
FENCE_HALF_EXTENT_M = 100.0
FENCE_MARGIN_M = 5.0
INSIDE_TARGET_NORTH_M = 30.0
OUTSIDE_TARGET_NORTH_M = 300.0


@dataclass
class PositionTelemetry:
    """Latest telemetry incl. position, kept current by a reader thread."""

    mode: str = "UNKNOWN"
    armed: bool = False
    relative_alt_m: float = 0.0
    alt_msl_m: float = math.nan
    lat: float = math.nan
    lon: float = math.nan
    _lock: threading.Lock = field(default_factory=threading.Lock)

    def update_from(self, conn) -> None:
        msg = conn.recv_match(blocking=True, timeout=1.0)
        if msg is None:
            return
        kind = msg.get_type()
        with self._lock:
            if kind == "HEARTBEAT":
                self.mode = mavutil.mode_string_v10(msg) or "UNKNOWN"
                self.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            elif kind == "GLOBAL_POSITION_INT":
                self.relative_alt_m = msg.relative_alt / 1000.0
                self.alt_msl_m = msg.alt / 1000.0
                self.lat = msg.lat / 1e7
                self.lon = msg.lon / 1e7

    def snapshot(self) -> tuple[str, bool, float, float, float]:
        with self._lock:
            return self.mode, self.armed, self.relative_alt_m, self.lat, self.lon

    def altitude_msl(self) -> float:
        with self._lock:
            return self.alt_msl_m


def _north_of(home: GPSCoordinate, lat: float, lon: float) -> float:
    return calculate_gps_offset_meters(home, GPSCoordinate(lat, lon)).north


def _fence_env_around(home: GPSCoordinate) -> str:
    """A square keep-in boundary FENCE_HALF_EXTENT_M around home, as env syntax."""
    dlat = FENCE_HALF_EXTENT_M / 111_194.9
    dlon = FENCE_HALF_EXTENT_M / (111_194.9 * math.cos(math.radians(home.lat)))
    corners = [
        (home.lat + dlat, home.lon - dlon),
        (home.lat + dlat, home.lon + dlon),
        (home.lat - dlat, home.lon + dlon),
        (home.lat - dlat, home.lon - dlon),
    ]
    return ";".join(f"{lat:.7f},{lon:.7f}" for lat, lon in corners)


def run_scenario(operator_ep: str, controller_ep: str) -> dict:
    """Execute the geofence containment scenario. Raises ScenarioError on failure."""
    results: dict[str, object] = {}

    _log(f"operator link  -> {operator_ep}")
    op = mavutil.mavlink_connection(operator_ep)
    if not op.wait_heartbeat(timeout=40):
        raise ScenarioError("no heartbeat on operator link")
    op.mav.request_data_stream_send(op.target_system, op.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 5, 1)
    telem = PositionTelemetry()
    reader_stop = threading.Event()

    def _reader():
        while not reader_stop.is_set():
            telem.update_from(op)

    threading.Thread(target=_reader, name="sitl-operator-reader", daemon=True).start()

    service: MavlinkService | None = None
    try:
        # --- read home, configure the fence BEFORE building the service ----
        snap = _wait_until(lambda s: math.isfinite(s[3]), 30, telem, "a GPS position")
        home = GPSCoordinate(snap[3], snap[4])
        os.environ["NOMAD_FENCE_POLYGON"] = _fence_env_around(home)
        os.environ["NOMAD_FENCE_MARGIN_M"] = str(FENCE_MARGIN_M)
        _log(f"fence configured: +/-{FENCE_HALF_EXTENT_M:.0f} m box around ({home.lat:.6f}, {home.lon:.6f})")

        _log(f"service link   -> {controller_ep}")
        service = MavlinkService(StateManager(), endpoint=controller_ep)
        service.start()

        def _service_ready(_s) -> bool:
            state = service.state_manager.get_state()
            if state.connected and state.home_lat is None:
                service.request_home_position()
            return state.connected and state.home_lat is not None

        _wait_until(_service_ready, 30, telem, "service connected with known home")
        _log("service connected; home position known")

        # --- arm -> GUIDED -> takeoff --------------------------------------
        _set_mode(op, "GUIDED")
        _wait_until(lambda s: s[0] == "GUIDED", 15, telem, "mode GUIDED")
        for attempt in range(20):
            _command_long(op, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)
            try:
                _wait_until(lambda s: s[1], 2, telem, "armed")
                break
            except ScenarioError:
                _log(f"  arm retry {attempt + 1} (prearm may still be settling)")
        else:
            raise ScenarioError("vehicle would not arm")
        _command_long(op, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 10.0)
        _wait_until(lambda s: s[2] >= 8.0, 40, telem, "takeoff to >=8 m")
        _log(f"reached altitude {telem.snapshot()[2]:.1f} m")

        # --- in-fence target: accepted and the vehicle actually moves ------
        inside = GPSCoordinate(home.lat + INSIDE_TARGET_NORTH_M / 111_194.9, home.lon)
        _log(f"in-fence target: {INSIDE_TARGET_NORTH_M:.0f} m north (expect accept + motion)")
        if not service.send_global_position_target(inside.lat, inside.lon, telem.altitude_msl()):
            raise ScenarioError("in-fence global position target was rejected")
        _wait_until(
            lambda s: _north_of(home, s[3], s[4]) >= INSIDE_TARGET_NORTH_M * 0.5,
            30,
            telem,
            "movement toward the in-fence target",
        )
        results["inside_target_accepted"] = True
        _log("  PASS: in-fence target accepted and flown (loop closed)")

        # --- out-of-fence target: rejected and the vehicle stays inside ----
        outside = GPSCoordinate(home.lat + OUTSIDE_TARGET_NORTH_M / 111_194.9, home.lon)
        _log(f"out-of-fence target: {OUTSIDE_TARGET_NORTH_M:.0f} m north (expect reject + containment)")
        if service.send_global_position_target(outside.lat, outside.lon, telem.altitude_msl()):
            raise ScenarioError("out-of-fence global position target was ACCEPTED")
        max_north = 0.0
        watch_end = time.time() + 10.0
        while time.time() < watch_end:
            snap = telem.snapshot()
            max_north = max(max_north, _north_of(home, snap[3], snap[4]))
            time.sleep(0.2)
        results["max_north_after_reject_m"] = round(max_north, 1)
        if max_north > FENCE_HALF_EXTENT_M:
            raise ScenarioError(f"vehicle left the fence after a rejected target (north={max_north:.1f} m)")
        _log(f"  PASS: target rejected, vehicle contained (max north {max_north:.1f} m)")

        # --- local NED target outside: rejected -----------------------------
        if service.send_position_target(OUTSIDE_TARGET_NORTH_M, 0.0, -10.0, 0.0):
            raise ScenarioError("out-of-fence LOCAL_NED target was ACCEPTED")
        results["local_outside_rejected"] = True
        _log("  PASS: out-of-fence LOCAL_NED target rejected")

        results["status"] = "PASS"
        return results
    finally:
        _log("cleanup: stopping service and returning the vehicle")
        if service is not None:
            service.stop()
        try:
            _set_mode(op, "RTL")
        except Exception:
            pass
        reader_stop.set()
        time.sleep(0.5)


def main() -> int:
    operator_ep = os.environ.get("NOMAD_SITL_OPERATOR")
    controller_ep = os.environ.get("NOMAD_SITL_CONTROLLER")
    if not operator_ep or not controller_ep:
        print("set NOMAD_SITL_OPERATOR and NOMAD_SITL_CONTROLLER (MAVLink endpoints)")
        return 2
    try:
        results = run_scenario(operator_ep, controller_ep)
    except ScenarioError as exc:
        _log(f"SCENARIO FAILED: {exc}")
        return 1
    _log(f"SCENARIO PASSED: {results}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
