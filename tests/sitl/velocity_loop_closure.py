# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""SITL loop-closure scenario for the safety-critical velocity core.

Drives a real ArduPilot SITL vehicle through the C++ core CLI (``nomad
velocity``) and proves the mitigations in ``docs/safety.md`` close the loop on
a real autopilot:

  arm -> GUIDED -> takeoff
    -> velocity step      (H-01/SR-VEL: commanded motion actually happens)
    -> watchdog stop      (H-03/SR-LNK-02: vehicle stops on command timeout)
    -> switch to LOITER   (H-04/SR-VEL-05 gate: setpoints refused out of GUIDED)

Two independent MAVLink paths are used: a pymavlink "operator" link that arms /
mode-switches / observes, and the C++ core CLI, which owns the velocity
command path. ArduPilot SITL exposes SERIAL1/SERIAL2 on TCP 5762/5763 for the
operator link; the core CLI receives its own UDP copy of the stream on the
host (NOMAD_CORE_SITL_PORT, default 14570).

Run against the dev stack (see tests/sitl/README.md):

    pixi run dev-up
    pixi run sitl-scenario
"""

from __future__ import annotations

import os
import subprocess
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path

from pymavlink import mavutil

ROOT = Path(__file__).resolve().parents[2]


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


def get_sitl_port() -> str:
    port = os.environ.get("NOMAD_CORE_SITL_PORT", "14570")
    if not port.isdecimal() or not 1 <= int(port) <= 65535:
        raise ValueError("NOMAD_CORE_SITL_PORT must be a UDP port from 1 to 65535")
    return port


def find_binary() -> Path | None:
    names = ("nomad.exe", "nomad")
    build_dirs = (ROOT / "build" / "core", ROOT / "build-core")
    configurations = tuple(
        directory for build_dir in build_dirs for directory in (build_dir, build_dir / "Debug", build_dir / "Release")
    )
    for directory in configurations:
        for name in names:
            candidate = directory / name
            if candidate.is_file():
                return candidate
    return None


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


def _connect_operator(endpoint: str):
    _log(f"operator link  -> {endpoint}")
    connection = mavutil.mavlink_connection(endpoint)
    if not connection.wait_heartbeat(timeout=40):
        raise ScenarioError("no heartbeat on operator link")
    _log(f"operator heartbeat: sys={connection.target_system} comp={connection.target_component}")
    return connection


def _start_telemetry_reader(connection) -> tuple[Telemetry, threading.Event]:
    connection.mav.request_data_stream_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL,
        5,
        1,
    )
    telemetry = Telemetry()
    stop_event = threading.Event()

    def read_telemetry():
        while not stop_event.is_set():
            telemetry.update_from(connection)

    reader = threading.Thread(target=read_telemetry, name="sitl-operator-reader", daemon=True)
    reader.start()
    return telemetry, stop_event


def _arm_vehicle(connection, telemetry: Telemetry) -> None:
    _log("mode is GUIDED; arming")
    for attempt in range(20):
        _command_long(connection, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)
        try:
            _wait_until(lambda state: state[1], 2, telemetry, "armed")
            return
        except ScenarioError:
            _log(f"  arm retry {attempt + 1} (prearm may still be settling)")
    raise ScenarioError("vehicle would not arm")


def _prepare_takeoff(connection, telemetry: Telemetry) -> None:
    _ensure_landed_and_disarmed(connection, telemetry)
    _set_mode(connection, "GUIDED")
    _wait_until(lambda state: state[0] == "GUIDED", 15, telemetry, "mode GUIDED")
    _arm_vehicle(connection, telemetry)
    _log("armed; commanding takeoff to 10 m")
    _command_long(connection, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 10.0)
    _wait_until(lambda state: state[3] >= 8.0, 40, telemetry, "takeoff to >=8 m")
    _log(f"reached altitude {telemetry.snapshot()[3]:.1f} m")


def _start_velocity_session(binary: Path, port: str, vx: float, duration_s: float) -> subprocess.Popen:
    """Spawn the C++ core CLI velocity stream (owns the command path)."""
    command = [
        str(binary),
        "velocity",
        "--vx",
        f"{vx}",
        "--duration",
        f"{duration_s}",
        "--endpoint",
        f"udpin:0.0.0.0:{port}",
    ]
    _log(f"$ {' '.join(command)}")
    return subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, env=os.environ.copy())


def _parse_fields(text: str) -> dict[str, str]:
    fields: dict[str, str] = {}
    for line in text.splitlines():
        for item in line.split():
            key, separator, value = item.partition("=")
            if separator:
                fields[key] = value
    return fields


def _wait_speed_below(telemetry: Telemetry, limit: float, timeout: float, what: str) -> None:
    deadline = time.time() + timeout
    while time.time() < deadline:
        speed = telemetry.snapshot()[2]
        if speed <= limit:
            _log(f"  groundspeed {speed:.2f} m/s <= {limit:.2f} m/s ({what})")
            return
        time.sleep(0.2)
    raise ScenarioError(f"vehicle did not slow below {limit:.2f} m/s within {timeout:.0f}s ({what})")


def _velocity_step(binary: Path, port: str, telemetry: Telemetry, results: dict[str, object]) -> float:
    _log("velocity step: core streams vx=1.5 m/s (forward) for 6 s")
    process = _start_velocity_session(binary, port, 1.5, 6.0)
    peak_speed = 0.0
    while process.poll() is None:
        peak_speed = max(peak_speed, telemetry.snapshot()[2])
        time.sleep(0.1)
    peak_speed = max(peak_speed, telemetry.snapshot()[2])
    stdout, stderr = process.communicate()
    fields = _parse_fields(stdout)
    results["velocity_exit"] = process.returncode
    results["velocity_accepted"] = fields.get("velocity_accepted")
    results["watchdog_reason"] = fields.get("watchdog_reason")
    results["peak_groundspeed"] = peak_speed
    _log(f"  exit={process.returncode} {stdout.strip()} peak_groundspeed={peak_speed:.2f} m/s")

    if process.returncode != 0:
        raise ScenarioError(f"core velocity session failed: {stderr.strip() or stdout.strip()}")
    if int(fields.get("velocity_accepted", "0")) <= 0:
        raise ScenarioError("core sent no velocity setpoints during the step")
    if peak_speed < 0.8:
        raise ScenarioError(f"vehicle did not move (peak groundspeed {peak_speed:.2f} < 0.8 m/s)")
    if fields.get("velocity_active") != "false" or fields.get("watchdog_reason") != "command_timeout":
        raise ScenarioError(f"watchdog did not stop the session cleanly: {fields}")
    _log("  PASS: commanded velocity produced real motion and the watchdog stopped it (H-01/H-03 loop closed)")
    # The CLI exits ~2 s after the watchdog stop, so the vehicle must already
    # be decelerating toward a stop.
    _wait_speed_below(telemetry, max(0.5, peak_speed * 0.5), 15.0, "watchdog stop")
    return peak_speed


def _check_mode_gate(connection, telemetry: Telemetry, binary: Path, port: str, results: dict[str, object]) -> None:
    _log("mode gate: switching to LOITER; core must refuse setpoints")
    _set_mode(connection, "LOITER")
    _wait_until(lambda state: state[0] == "LOITER", 15, telemetry, "mode LOITER")
    time.sleep(2.0)
    process = _start_velocity_session(binary, port, 1.0, 2.0)
    stdout, stderr = process.communicate(timeout=60)
    results["loiter_exit"] = process.returncode
    results["loiter_stderr"] = stderr.strip()
    _log(f"  exit={process.returncode} {stderr.strip()}")
    if process.returncode == 0:
        raise ScenarioError(f"core accepted a velocity session while NOT in GUIDED: {stdout.strip()}")
    if "GUIDED" not in stderr:
        raise ScenarioError(f"unexpected rejection outside GUIDED: {stderr.strip()}")
    _log("  PASS: setpoint refused outside GUIDED (H-04 gate enforced)")


def _cleanup_scenario(connection, reader_stop: threading.Event) -> None:
    _log("cleanup: returning the vehicle")
    try:
        _set_mode(connection, "RTL")
    except Exception:
        pass
    reader_stop.set()
    time.sleep(0.5)


def run_scenario(operator_ep: str, binary: Path, port: str) -> dict:
    """Execute the loop-closure scenario. Raises ScenarioError on failure."""
    operator = _connect_operator(operator_ep)
    telemetry, reader_stop = _start_telemetry_reader(operator)
    results: dict[str, object] = {}
    try:
        _prepare_takeoff(operator, telemetry)
        peak_speed = _velocity_step(binary, port, telemetry, results)
        results["peak_groundspeed"] = peak_speed
        _check_mode_gate(operator, telemetry, binary, port, results)
        results["status"] = "PASS"
        return results
    finally:
        _cleanup_scenario(operator, reader_stop)


def main() -> int:
    operator_ep = os.environ.get("NOMAD_SITL_OPERATOR")
    if not operator_ep:
        print("set NOMAD_SITL_OPERATOR (MAVLink endpoint, e.g. tcp:127.0.0.1:5762)")
        return 2
    binary = find_binary()
    if binary is None:
        print("error: C++ core binary not found; run `pixi run build-core` first", file=sys.stderr)
        return 2
    _log("watch this run live: Mission Planner -> CONNECT -> TCP -> 127.0.0.1:5762 (passive observer)")
    try:
        port = get_sitl_port()
    except ValueError as error:
        print(f"error: {error}", file=sys.stderr)
        return 2
    try:
        results = run_scenario(operator_ep, binary, port)
    except ScenarioError as exc:
        _log(f"SCENARIO FAILED: {exc}")
        return 1
    _log(f"SCENARIO PASSED: {results}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
