# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Observe the pinned QuadPlane SITL identity, telemetry, and baseline modes."""

from __future__ import annotations

import os
import subprocess
import sys
import time
from pathlib import Path

from core_sitl_status import find_binary

EXPECTED_ARDUPILOT_REVISION = "dbe792162d06cab66c3475fd5556bf7a120f119e"
EXPECTED_IDENTITY = {
    "autopilot_type": "3",
    "vehicle_type": "1",
    "aircraft_class": "QuadPlane",
}
MAX_SAMPLE_AGE_MS = 1500


class ScenarioError(RuntimeError):
    pass


def get_sitl_port() -> str:
    port = os.environ.get("NOMAD_QUADPLANE_SITL_PORT", "14580")
    if not port.isdecimal() or not 1 <= int(port) <= 65535:
        raise ValueError("NOMAD_QUADPLANE_SITL_PORT must be a UDP port from 1 to 65535")
    return port


def run_cli(binary: Path, port: str, *arguments: str, timeout: int = 30) -> str:
    endpoint = f"udpin:0.0.0.0:{port}"
    result = subprocess.run(
        [str(binary), *arguments, "--endpoint", endpoint],
        capture_output=True,
        text=True,
        timeout=timeout,
        check=False,
    )
    if result.returncode != 0:
        raise ScenarioError(f"{' '.join(arguments)} failed: stdout={result.stdout!r} stderr={result.stderr!r}")
    return result.stdout


def parse_status(output: str) -> dict[str, str]:
    fields: dict[str, str] = {}
    for token in output.split():
        if "=" in token:
            key, value = token.split("=", maxsplit=1)
            fields[key] = value
    return fields


def require_status(fields: dict[str, str], expected: dict[str, str]) -> None:
    mismatch = {key: (fields.get(key), value) for key, value in expected.items() if fields.get(key) != value}
    if mismatch:
        raise ScenarioError(f"unexpected QuadPlane status {mismatch}; full status={fields}")


def require_fresh_telemetry(fields: dict[str, str]) -> None:
    for key in ("position_age_ms", "gps_age_ms", "attitude_age_ms"):
        try:
            age_ms = int(fields[key])
        except (KeyError, ValueError) as error:
            raise ScenarioError(f"missing or invalid {key}; full status={fields}") from error
        if not 0 <= age_ms <= MAX_SAMPLE_AGE_MS:
            raise ScenarioError(f"stale {key}={age_ms}; maximum is {MAX_SAMPLE_AGE_MS}")
    for key in ("position", "altitude_m", "gps_fix", "attitude"):
        if key not in fields:
            raise ScenarioError(f"missing {key}; full status={fields}")
    if int(fields["gps_fix"]) < 3:
        raise ScenarioError(f"GPS fix is below 3D: {fields['gps_fix']}")


def read_status(binary: Path, port: str) -> dict[str, str]:
    return parse_status(run_cli(binary, port, "status"))


def wait_for_mode(binary: Path, port: str, expected_mode: int, timeout: float = 15.0) -> None:
    deadline = time.monotonic() + timeout
    last: dict[str, str] = {}
    while time.monotonic() < deadline:
        last = read_status(binary, port)
        if last.get("mode") == str(expected_mode):
            require_fresh_telemetry(last)
            return
    raise ScenarioError(f"mode {expected_mode} was not reported; last status={last}")


def request_observed_mode(port: str, mode: int) -> None:
    """Use an independent test driver without granting the production Vehicle capability."""
    from pymavlink import mavutil

    connection = mavutil.mavlink_connection(f"udpin:0.0.0.0:{port}", source_system=250)
    try:
        heartbeat = connection.wait_heartbeat(timeout=10)
        if heartbeat is None:
            raise ScenarioError("test driver did not receive a QuadPlane heartbeat")
        connection.mav.set_mode_send(
            connection.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode,
        )
    finally:
        connection.close()


def verify_baseline_modes(binary: Path, port: str) -> None:
    for mode, name in ((15, "GUIDED"), (19, "QLOITER"), (21, "QRTL")):
        request_observed_mode(port, mode)
        wait_for_mode(binary, port, mode)
        print(f"observed {name} custom mode {mode}")
    request_observed_mode(port, 11)
    wait_for_mode(binary, port, 11)
    print("observed RTL custom mode 11")
    request_observed_mode(port, 0)
    wait_for_mode(binary, port, 0)


def main() -> int:
    binary = find_binary()
    if binary is None:
        print("error: qualification driver not found; run `pixi run build-qualification-cli` first", file=sys.stderr)
        return 2
    try:
        port = get_sitl_port()
        initial = read_status(binary, port)
        require_status(initial, {"connected": "true", "heartbeat_fresh": "true", **EXPECTED_IDENTITY})
        require_status(initial, {"armed": "false"})
        require_fresh_telemetry(initial)
        verify_baseline_modes(binary, port)
    except (ValueError, ScenarioError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 1
    print("QuadPlane identity/telemetry observation passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
