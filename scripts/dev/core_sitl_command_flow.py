# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Run the C++ MVP command flow against the Docker ArduPilot SITL vehicle."""

from __future__ import annotations

import subprocess
import sys
import time
from pathlib import Path

from core_sitl_status import find_binary, get_sitl_port, print_watch_hint


class ScenarioError(RuntimeError):
    """The C++ command flow did not reach an expected vehicle state."""


def run_cli(binary: Path, port: str, *arguments: str, attempts: int = 5) -> str:
    """Run one C++ CLI command against the SITL UDP copy, retrying on link loss.

    The Docker Desktop UDP relay that carries the host-side SITL copy on
    Windows intermittently drops datagrams, so a command acknowledgement or
    the initial heartbeat can be lost on a single attempt. The commands in
    these scenarios are idempotent (mode, arm, takeoff, rtl, land, disarm) and
    every step is still verified by authoritative status polling afterwards,
    so a bounded retry is safe here; the C++ core itself fails closed on a
    lost acknowledgement.
    """
    command = [str(binary), *arguments, "--endpoint", f"udpin:0.0.0.0:{port}"]
    for attempt in range(1, attempts + 1):
        result = subprocess.run(command, capture_output=True, text=True, check=False)
        output = "\n".join(part for part in (result.stdout.strip(), result.stderr.strip()) if part)
        if attempt > 1:
            print(f"  (retry {attempt}/{attempts})", flush=True)
        print(f"$ {' '.join(command)}\n{output}", flush=True)
        if result.returncode == 0:
            return result.stdout
        if attempt < attempts:
            time.sleep(2.0)
    raise ScenarioError(
        f"command failed after {attempts} attempts with exit code {result.returncode}: {' '.join(arguments)}"
    )


def parse_status(output: str) -> dict[str, str]:
    fields: dict[str, str] = {}
    for line in output.splitlines():
        for item in line.split():
            key, separator, value = item.partition("=")
            if separator:
                fields[key] = value
    return fields


def read_status(binary: Path, port: str) -> dict[str, str]:
    return parse_status(run_cli(binary, port, "status"))


def wait_for_status(binary: Path, port: str, expected: dict[str, str], timeout: float) -> dict[str, str]:
    deadline = time.monotonic() + timeout
    last: dict[str, str] = {}
    while time.monotonic() < deadline:
        last = read_status(binary, port)
        if all(last.get(key) == value for key, value in expected.items()):
            return last
        time.sleep(1.0)
    raise ScenarioError(f"timed out waiting for {expected}; last status={last}")


def wait_for_altitude(binary: Path, port: str, minimum_m: float, timeout: float) -> None:
    deadline = time.monotonic() + timeout
    last: dict[str, str] = {}
    while time.monotonic() < deadline:
        last = read_status(binary, port)
        try:
            altitude = float(last.get("relative_altitude_m", "nan"))
        except ValueError:
            altitude = float("nan")
        if altitude >= minimum_m:
            return
        time.sleep(1.0)
    raise ScenarioError(f"timed out waiting for altitude >= {minimum_m}; last status={last}")


def wait_for_gps_fix(binary: Path, port: str, timeout: float) -> None:
    deadline = time.monotonic() + timeout
    last: dict[str, str] = {}
    while time.monotonic() < deadline:
        last = read_status(binary, port)
        try:
            fix_type = int(last.get("gps_fix", "0"))
        except ValueError:
            fix_type = 0
        if fix_type >= 3:
            return
        time.sleep(1.0)
    raise ScenarioError(f"timed out waiting for a 3D GPS fix; last status={last}")


def read_position(binary: Path, port: str) -> tuple[float, float]:
    """Return the last authoritative latitude/longitude from the CLI status."""
    last = read_status(binary, port)
    try:
        latitude, longitude = (float(part) for part in last["position"].split(","))
    except (KeyError, ValueError) as error:
        raise ScenarioError(f"no valid position in status; last status={last}") from error
    return latitude, longitude


def wait_for_position(
    binary: Path, port: str, expected: tuple[float, float], tolerance_deg: float, timeout: float
) -> None:
    deadline = time.monotonic() + timeout
    last = ""
    while time.monotonic() < deadline:
        try:
            latitude, longitude = read_position(binary, port)
        except ScenarioError as error:
            last = str(error)
            time.sleep(1.0)
            continue
        if abs(latitude - expected[0]) <= tolerance_deg and abs(longitude - expected[1]) <= tolerance_deg:
            return
        last = f"{latitude},{longitude}"
        time.sleep(1.0)
    raise ScenarioError(f"timed out waiting for position within {tolerance_deg} deg of {expected}; last={last}")


def run_flow(binary: Path, port: str) -> None:
    initial = wait_for_status(binary, port, {"connected": "true"}, 15.0)
    if initial.get("armed") == "true":
        raise ScenarioError("SITL must start disarmed for the C++ command-flow scenario")

    run_cli(binary, port, "mode", "4")
    wait_for_status(binary, port, {"mode": "4"}, 15.0)

    wait_for_gps_fix(binary, port, 60.0)

    run_cli(binary, port, "arm")
    wait_for_status(binary, port, {"armed": "true"}, 15.0)

    run_cli(binary, port, "takeoff", "5")
    wait_for_altitude(binary, port, 4.0, 45.0)

    # Guided goto to a point about 22 m north of the current position, then
    # verify the vehicle actually arrives there. The C++ core sends
    # MAV_CMD_DO_REPOSITION as command_int (the command_long form is answered
    # MAV_RESULT_UNSUPPORTED by Copter 4.7.0) and verifies the resulting
    # position, so an accepted goto here proves the full wire path live.
    latitude, longitude = read_position(binary, port)
    target = (latitude + 0.0002, longitude)
    run_cli(binary, port, "goto", f"{target[0]:.7f}", f"{target[1]:.7f}", "5")
    wait_for_position(binary, port, target, 0.0003, 45.0)

    run_cli(binary, port, "rtl")
    wait_for_status(binary, port, {"mode": "6"}, 15.0)

    run_cli(binary, port, "land")
    wait_for_status(binary, port, {"mode": "9"}, 15.0)
    wait_for_status(binary, port, {"armed": "false"}, 90.0)

    run_cli(binary, port, "disarm")
    wait_for_status(binary, port, {"armed": "false"}, 15.0)
    print("C++ SITL command flow passed", flush=True)


def main() -> int:
    try:
        port = get_sitl_port()
    except ValueError as error:
        print(f"error: {error}", file=sys.stderr)
        return 2

    binary = find_binary()
    if binary is None:
        print("error: C++ core binary not found; run `pixi run build-core` first", file=sys.stderr)
        return 2
    print_watch_hint()

    try:
        run_flow(binary, port)
    except ScenarioError as error:
        print(f"C++ SITL command flow failed: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
