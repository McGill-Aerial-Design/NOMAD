# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Transport-level zero-delivery proof against live SITL (SR-LNK-03).

The deterministic C++ tests (tests/mavsdk_zero_delivery_test.cpp) prove the zero
setpoint reaches a peer socket. This scenario proves the same thing against a
real ArduPilot vehicle: while the C++ CLI streams velocity setpoints,
a pymavlink observer watches every SET_POSITION_TARGET_LOCAL_NED frame the CLI
puts on the wire, and after the stream ends it must observe the watchdog's
all-zero setpoint. The vehicle itself is the second witness: it must stop
(hover) instead of drifting.

A tee relay sits between the SITL stream and the CLI (same topology as the
link-recovery scenario): upstream -> client is forwarded, and client ->
upstream datagrams — the CLI's setpoints — are copied to the observer socket.
"""

from __future__ import annotations

import socket
import subprocess
import sys
import threading
import time
from pathlib import Path

from core_sitl_command_flow import (
    ScenarioError,
    parse_status,
    run_cli,
    wait_for_altitude,
    wait_for_status,
)
from core_sitl_status import find_binary, get_sitl_port, print_watch_hint
from pymavlink import mavutil

SET_POSITION_TARGET_LOCAL_NED = 84
ZERO_AFTER_STREAM_BUDGET_S = 3.0
HOVER_TOLERANCE_M = 1.0


class TeeRelay:
    """Bidirectional UDP relay that copies client datagrams to an observer.

    Same single-socket topology as the link-recovery relay: telemetry from
    the SITL stream is forwarded to the client port, and datagrams from the
    client are forwarded back to the SITL peer. Additionally every client
    datagram is copied verbatim to the observer port so a passive pymavlink
    listener sees exactly what the CLI puts on the wire.
    """

    def __init__(self, upstream_port: int, client_port: int, observer_port: int) -> None:
        socket_ = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            socket_.bind(("0.0.0.0", upstream_port))
            socket_.settimeout(0.1)
        except OSError:
            socket_.close()
            raise
        self._socket = socket_
        self._client_address = ("127.0.0.1", client_port)
        self._observer_address = ("127.0.0.1", observer_port)
        self._sitl_peer: tuple[str, int] | None = None
        self._running = True
        try:
            self._thread = threading.Thread(target=self._run, daemon=True)
            self._thread.start()
        except (RuntimeError, OSError):
            self._socket.close()
            raise

    def _run(self) -> None:
        while self._running:
            try:
                data, sender = self._socket.recvfrom(4096)
            except TimeoutError:
                continue
            except OSError as error:
                # Windows reports ICMP port-unreachable as WSAECONNRESET on
                # the next recvfrom; that is normal relay traffic, not death.
                if getattr(error, "winerror", None) == 10054:
                    continue
                return
            if sender == self._client_address:
                self._socket.sendto(data, self._observer_address)
                if self._sitl_peer is not None:
                    self._socket.sendto(data, self._sitl_peer)
                continue
            self._sitl_peer = sender
            self._socket.sendto(data, self._client_address)

    def close(self) -> None:
        self._running = False
        self._socket.close()
        self._thread.join(timeout=1.0)


class SetpointObserver:
    """Records velocity setpoints seen on the wire through pymavlink."""

    def __init__(self, port: int) -> None:
        self._mav = mavutil.mavlink.MAVLink(None, srcSystem=250, srcComponent=190)
        socket_ = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            socket_.bind(("0.0.0.0", port))
            socket_.settimeout(0.1)
        except OSError:
            socket_.close()
            raise
        self._socket = socket_
        self._lock = threading.Lock()
        self._setpoints: list[tuple[float, float, float, float, float]] = []
        self._running = True
        try:
            self._thread = threading.Thread(target=self._run, daemon=True)
            self._thread.start()
        except (RuntimeError, OSError):
            self._socket.close()
            raise

    def _run(self) -> None:
        while self._running:
            try:
                data, _ = self._socket.recvfrom(4096)
            except TimeoutError:
                continue
            except OSError:
                return
            self._record_datagram(data)

    def _record_datagram(self, data: bytes) -> None:
        for message in self._mav.parse_buffer(data) or []:
            if message.get_type() == "SET_POSITION_TARGET_LOCAL_NED":
                self._record(message)

    def _record(self, message) -> None:
        with self._lock:
            self._setpoints.append(
                (
                    time.monotonic(),
                    float(message.vx),
                    float(message.vy),
                    float(message.vz),
                    float(message.yaw_rate),
                )
            )

    def setpoints_in(self, window_s: float) -> list[tuple[float, float, float, float, float]]:
        cutoff = time.monotonic() - window_s
        with self._lock:
            return [point for point in self._setpoints if point[0] >= cutoff]

    def close(self) -> None:
        self._running = False
        self._thread.join(timeout=1.0)
        self._socket.close()


def get_relay_ports(upstream_port: str) -> tuple[int, int, int]:
    """Reserve the upstream port plus two adjacent local relay ports."""
    upstream = int(upstream_port)
    if not 1 <= upstream <= 65533:
        raise ValueError("NOMAD_CORE_SITL_PORT must leave two UDP ports for the relay")
    return upstream, upstream + 1, upstream + 2


def is_nonzero(point: tuple[float, float, float, float, float]) -> bool:
    return any(abs(value) > 1e-6 for value in point[1:])


def is_zero(point: tuple[float, float, float, float, float]) -> bool:
    return all(abs(value) <= 1e-6 for value in point[1:])


def wait_for_zero(observer: SetpointObserver, started_s: float) -> None:
    """Require the all-zero setpoint on the wire after the stream ended."""
    deadline = time.monotonic() + ZERO_AFTER_STREAM_BUDGET_S
    while time.monotonic() < deadline:
        recent = [point for point in observer.setpoints_in(ZERO_AFTER_STREAM_BUDGET_S) if point[0] >= started_s]
        nonzero_seen = any(is_nonzero(point) for point in recent)
        zero_seen = any(is_zero(point) for point in recent)
        if nonzero_seen and zero_seen:
            zero_time = max(point[0] for point in recent if is_zero(point))
            first_nonzero = min(point[0] for point in recent if is_nonzero(point))
            if zero_time > first_nonzero:
                return
        time.sleep(0.05)
    raise ScenarioError(f"no zero setpoint observed on the wire after the stream; wire={recent}")


def read_position(binary: Path, port: str) -> tuple[float, float]:
    fields = parse_status(run_cli(binary, port, "status"))
    try:
        latitude, longitude = (float(part) for part in fields["position"].split(","))
    except (KeyError, ValueError) as error:
        raise ScenarioError(f"no valid position in status: {fields}") from error
    return latitude, longitude


def verify_vehicle_stopped(binary: Path, port: str) -> None:
    """The vehicle must hover after the zero: no continued drift."""
    before = read_position(binary, port)
    time.sleep(2.0)
    after = read_position(binary, port)
    drift_deg = max(abs(after[0] - before[0]), abs(after[1] - before[1]))
    # ~1.1 m per 0.00001 deg at these latitudes; keep the margin generous so
    # the check proves "stopped" without being flaky about SITL GPS noise.
    if drift_deg > HOVER_TOLERANCE_M / 100000.0:
        raise ScenarioError(f"vehicle kept moving after the zero setpoint: {before} -> {after}")


def arm_and_takeoff(binary: Path, port: str) -> None:
    run_cli(binary, port, "mode", "4")
    wait_for_status(binary, port, {"mode": "4"}, 15.0)
    run_cli(binary, port, "arm")
    wait_for_status(binary, port, {"armed": "true"}, 15.0)
    run_cli(binary, port, "takeoff", "5")
    wait_for_altitude(binary, port, 4.0, 45.0)


def run_velocity_stream(binary: Path, port: str) -> None:
    """Stream a short guided velocity command and require success."""
    command = [
        str(binary),
        "velocity",
        "--vx",
        "0.3",
        "--duration",
        "2",
        "--endpoint",
        f"udpin:0.0.0.0:{port}",
    ]
    result = subprocess.run(command, capture_output=True, text=True, check=False)
    print(f"$ {' '.join(command)}\n{result.stdout.strip()}\n{result.stderr.strip()}", flush=True)
    if result.returncode != 0:
        raise ScenarioError(f"velocity run failed: {result.stderr.strip()}")


def cleanup_zero_delivery(binary: Path, port: str) -> None:
    """Require bounded RTL/land cleanup and authoritative disarm."""
    print("cleanup: returning and landing the vehicle", flush=True)
    errors: list[str] = []
    for action in ("rtl", "land"):
        try:
            run_cli(binary, port, action, attempts=2)
        except (OSError, ScenarioError) as error:
            errors.append(f"{action}: {error}")
    try:
        wait_for_status(binary, port, {"armed": "false"}, 120.0)
    except (OSError, ScenarioError) as error:
        errors.append(f"disarm verification: {error}")
    try:
        run_cli(binary, port, "disarm", attempts=2)
    except (OSError, ScenarioError) as error:
        errors.append(f"disarm command: {error}")
    if errors:
        raise ScenarioError("; ".join(errors))


def run_zero_delivery(binary: Path, upstream_port: str) -> None:
    upstream, client_port, observer_port = get_relay_ports(upstream_port)
    relay = TeeRelay(upstream, client_port, observer_port)
    try:
        observer = SetpointObserver(observer_port)
    except (OSError, RuntimeError):
        relay.close()
        raise
    port = str(client_port)
    flight_may_be_armed = False
    try:
        wait_for_status(binary, port, {"connected": "true"}, 15.0)
        flight_may_be_armed = True
        arm_and_takeoff(binary, port)

        started_s = time.monotonic()
        run_velocity_stream(binary, port)
        wait_for_zero(observer, started_s)
        print("zero setpoint observed on the wire after the stream ended", flush=True)

        verify_vehicle_stopped(binary, port)
        print("vehicle stopped: hover position held after the zero", flush=True)
    except (OSError, ScenarioError) as error:
        if flight_may_be_armed:
            try:
                cleanup_zero_delivery(binary, port)
            except (OSError, ScenarioError) as cleanup_error:
                raise ScenarioError(f"{error}; safe cleanup failed: {cleanup_error}") from error
        raise
    else:
        cleanup_zero_delivery(binary, port)
        print("C++ SITL zero-delivery proof passed", flush=True)
    finally:
        observer.close()
        relay.close()


def main() -> int:
    try:
        upstream_port = get_sitl_port()
        binary = find_binary()
        if binary is None:
            print(
                "error: qualification driver not found; run `pixi run build-qualification-cli` first", file=sys.stderr
            )
            return 2
        print_watch_hint()
        run_zero_delivery(binary, upstream_port)
        return 0
    except (OSError, ValueError, ScenarioError) as error:
        print(f"C++ SITL zero-delivery proof failed: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
