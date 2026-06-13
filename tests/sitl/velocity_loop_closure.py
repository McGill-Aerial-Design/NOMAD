# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""SITL loop-closure scenario for the safety-critical velocity core.

Drives a real ArduPilot SITL vehicle and exercises
``MavlinkVelocityController`` (Tier SC) against it end-to-end, proving the
mitigations in ``docs/safety/hazards.md`` close the loop on a real autopilot:

  arm -> GUIDED -> takeoff
    -> velocity step      (H-01/SR-VEL: commanded motion actually happens)
    -> stop commanding    (H-03/SR-LNK-02 watchdog: vehicle stops on its own)
    -> switch to LOITER    (H-04/SR-VEL-05 gate: setpoints refused out of GUIDED)

Two independent MAVLink connections are used: an "operator" link that arms /
mode-switches / observes, and the controller's own link. ArduPilot SITL exposes
SERIAL1/SERIAL2 on TCP 5762/5763 for exactly this.

Run standalone inside the dev network (see tests/sitl/README.md):

    NOMAD_SITL_OPERATOR=tcp:host:5762 NOMAD_SITL_CONTROLLER=tcp:host:5763 \
        python tests/sitl/velocity_loop_closure.py
"""

from __future__ import annotations

import os
import threading
import time
from dataclasses import dataclass, field

from pymavlink import mavutil

from edge_core.ros_http_bridge.mavlink_velocity import MavlinkVelocityController


@dataclass
class Telemetry:
    """Latest telemetry, kept current by a background reader thread."""

    mode: str = "UNKNOWN"
    armed: bool = False
    groundspeed: float = 0.0
    relative_alt_m: float = 0.0
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
            elif kind == "VFR_HUD":
                self.groundspeed = float(msg.groundspeed)
            elif kind == "GLOBAL_POSITION_INT":
                self.relative_alt_m = msg.relative_alt / 1000.0

    def snapshot(self) -> tuple[str, bool, float, float]:
        with self._lock:
            return self.mode, self.armed, self.groundspeed, self.relative_alt_m


class ScenarioError(AssertionError):
    """A scenario assertion failed (the SC behaviour was not observed)."""


def _log(msg: str) -> None:
    print(f"[sitl] {time.strftime('%H:%M:%S')} {msg}", flush=True)


def _command_long(conn, command: int, *params: float) -> None:
    args = list(params) + [0.0] * (7 - len(params))
    conn.mav.command_long_send(conn.target_system, conn.target_component, command, 0, *args)


def _set_mode(conn, mode: str) -> None:
    mode_id = conn.mode_mapping()[mode]
    conn.mav.set_mode_send(conn.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id)


def _wait_until(predicate, timeout: float, telem: Telemetry, what: str):
    deadline = time.time() + timeout
    while time.time() < deadline:
        snap = telem.snapshot()
        if predicate(snap):
            return snap
        time.sleep(0.2)
    raise ScenarioError(f"timed out after {timeout:.0f}s waiting for {what}; last telemetry={telem.snapshot()}")


def _ensure_landed_and_disarmed(conn, telem, timeout: float = 120.0) -> None:
    """Bring the vehicle to a clean landed + disarmed state before arming it.

    Sequential SITL scenarios share one vehicle, and a scenario's cleanup only
    *initiates* RTL - it does not wait for landing. So the next scenario can
    start while the vehicle is still airborne (mid-RTL), which derails its
    arm -> GUIDED -> takeoff (the takeoff is ignored while already flying, and
    home gets read mid-transit). Calling this first makes each scenario
    independent of however the previous one left the vehicle.

    Relies only on ``snapshot()[1]`` (armed), which both scenario telemetry
    types expose, so it is shared. Disarmed in SITL means motors off on the
    ground (RTL auto-disarms after landing).
    """
    # Let the reader latch a real heartbeat first; ``armed`` defaults to False.
    time.sleep(2.0)
    if not telem.snapshot()[1]:
        return  # already disarmed -> on the ground
    _log("vehicle still airborne from a prior scenario; RTL + waiting for disarm")
    try:
        _set_mode(conn, "RTL")
    except Exception:
        pass
    deadline = time.time() + timeout
    while time.time() < deadline:
        if not telem.snapshot()[1]:
            _log("landed and disarmed; settling before takeoff")
            time.sleep(3.0)  # let baro/EKF settle on the ground
            return
        time.sleep(0.5)
    raise ScenarioError(f"vehicle did not land + disarm within {timeout:.0f}s for a clean takeoff")


def run_scenario(operator_ep: str, controller_ep: str) -> dict:
    """Execute the loop-closure scenario. Raises ScenarioError on failure."""
    results: dict[str, object] = {}

    _log(f"operator link  -> {operator_ep}")
    op = mavutil.mavlink_connection(operator_ep)
    if not op.wait_heartbeat(timeout=40):
        raise ScenarioError("no heartbeat on operator link")
    _log(f"operator heartbeat: sys={op.target_system} comp={op.target_component}")

    # Stream telemetry and keep the latest in `telem`.
    op.mav.request_data_stream_send(op.target_system, op.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 5, 1)
    telem = Telemetry()
    reader_stop = threading.Event()

    def _reader():
        while not reader_stop.is_set():
            telem.update_from(op)

    reader = threading.Thread(target=_reader, name="sitl-operator-reader", daemon=True)
    reader.start()

    controller: MavlinkVelocityController | None = None
    vio_stop = threading.Event()
    try:
        # Start from a clean ground state regardless of how a prior scenario
        # (sharing this vehicle) left it.
        _ensure_landed_and_disarmed(op, telem)

        # --- arm -> GUIDED -> takeoff -------------------------------------
        _set_mode(op, "GUIDED")
        _wait_until(lambda s: s[0] == "GUIDED", 15, telem, "mode GUIDED")
        _log("mode is GUIDED; arming")

        for attempt in range(20):
            _command_long(op, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)
            try:
                _wait_until(lambda s: s[1], 2, telem, "armed")
                break
            except ScenarioError:
                _log(f"  arm retry {attempt + 1} (prearm may still be settling)")
        else:
            raise ScenarioError("vehicle would not arm")
        _log("armed; commanding takeoff to 10 m")

        _command_long(op, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 10.0)
        _wait_until(lambda s: s[3] >= 8.0, 40, telem, "takeoff to >=8 m")
        _log(f"reached altitude {telem.snapshot()[3]:.1f} m")

        # --- start the SC controller on its own link ----------------------
        controller = MavlinkVelocityController(endpoint=controller_ep, require_armed=True, require_guided=True)
        if not controller.start():
            raise ScenarioError("controller failed to start (pymavlink unavailable?)")

        # Feed VIO freshness continuously, as the ROS bridge would.
        def _feed_vio():
            while not vio_stop.is_set():
                controller.note_vio(1.0, healthy=True)
                time.sleep(0.05)

        threading.Thread(target=_feed_vio, name="sitl-vio-feeder", daemon=True).start()

        # Let the controller's rx thread lock onto armed + GUIDED heartbeats.
        time.sleep(3.0)

        # --- velocity step: command forward, expect real motion -----------
        _log("velocity step: submitting vx=1.5 m/s (forward) for 6 s")
        peak_speed = 0.0
        accepted = 0
        end = time.time() + 6.0
        while time.time() < end:
            if controller.submit(1.5, 0.0, 0.0, 0.0):
                accepted += 1
            peak_speed = max(peak_speed, telem.snapshot()[2])
            time.sleep(0.05)
        peak_speed = max(peak_speed, telem.snapshot()[2])
        _log(f"  accepted submits={accepted} sent_count={controller.sent_count} peak_groundspeed={peak_speed:.2f} m/s")
        results["peak_groundspeed"] = peak_speed
        results["sent_count"] = controller.sent_count
        if controller.sent_count <= 0:
            raise ScenarioError("controller sent no setpoints despite armed+GUIDED+fresh-VIO")
        if peak_speed < 0.8:
            raise ScenarioError(f"vehicle did not move (peak groundspeed {peak_speed:.2f} < 0.8 m/s)")
        _log("  PASS: commanded velocity produced real motion (H-01 loop closed)")

        # --- watchdog: stop commanding, expect the vehicle to stop --------
        _log("watchdog: stop submitting + stop VIO; expect zero-velocity failsafe")
        vio_stop.set()  # also makes VIO go stale
        time.sleep(5.0)  # > COMMAND_TIMEOUT and > VIO max age; let it settle
        stopped_speed = telem.snapshot()[2]
        results["stopped_groundspeed"] = stopped_speed
        _log(f"  groundspeed after watchdog: {stopped_speed:.2f} m/s (was {peak_speed:.2f})")
        if stopped_speed > max(0.5, peak_speed * 0.5):
            raise ScenarioError(f"watchdog did not stop the vehicle (groundspeed {stopped_speed:.2f})")
        if controller._active:  # noqa: SLF001 - asserting the failsafe latched
            raise ScenarioError("watchdog fired but controller still marked active")
        _log("  PASS: vehicle stopped on command-timeout/VIO-stale (H-03/H-02 loop closed)")

        # --- mode gate: leave GUIDED, expect setpoints refused ------------
        _log("mode gate: switching to LOITER; controller must refuse setpoints")
        _set_mode(op, "LOITER")
        _wait_until(lambda s: s[0] == "LOITER", 15, telem, "mode LOITER")
        time.sleep(2.0)  # let the controller's rx thread see the new mode
        before = controller.rejected_count
        # Refresh VIO so the ONLY failing gate is the flight mode.
        controller.note_vio(1.0, healthy=True)
        accepted_in_loiter = controller.submit(1.0, 0.0, 0.0, 0.0)
        results["accepted_in_loiter"] = accepted_in_loiter
        results["rejected_delta"] = controller.rejected_count - before
        if accepted_in_loiter:
            raise ScenarioError("controller accepted a setpoint while NOT in GUIDED")
        _log("  PASS: setpoint refused outside GUIDED (H-04 gate enforced)")

        results["status"] = "PASS"
        return results
    finally:
        vio_stop.set()
        _log("cleanup: stopping controller and returning the vehicle")
        if controller is not None:
            controller.stop()  # commands zero velocity (SR-LNK-03)
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
