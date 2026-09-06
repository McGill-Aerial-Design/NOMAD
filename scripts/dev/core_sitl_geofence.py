# SPDX-License-Identifier: Apache-2.0
"""Run the C++ geofence acceptance checks against the Docker SITL vehicle."""

from __future__ import annotations

import os
import sys
import time
from pathlib import Path
from typing import Any

from core_sitl_command_flow import ScenarioError, run_cli, wait_for_status
from core_sitl_status import find_binary, get_sitl_port, print_watch_hint


def get_operator_port() -> str:
    """TCP port that mirrors the SITL vehicle's operator MAVLink link."""
    port = os.environ.get("NOMAD_CORE_SITL_OPERATOR_PORT", "5762")
    if not port.isdecimal() or not 1 <= int(port) <= 65535:
        raise ValueError("NOMAD_CORE_SITL_OPERATOR_PORT must be a TCP port from 1 to 65535")
    return port


def connect_operator(port: str) -> Any:
    import pymavlink.mavutil as mavutil

    connection = mavutil.mavlink_connection(f"tcp:127.0.0.1:{port}")
    connection.wait_heartbeat(timeout=10)
    return connection


def read_fence_enable(port: str) -> float:
    connection = connect_operator(port)
    try:
        connection.mav.param_request_read_send(
            connection.target_system, connection.target_component, b"FENCE_ENABLE", -1
        )
        message = connection.recv_match(type="PARAM_VALUE", blocking=True, timeout=5)
        if message is None:
            raise ScenarioError("could not read FENCE_ENABLE for restore")
        return float(message.param_value)
    finally:
        connection.close()


def restore_fence_enable(port: str, previous: float) -> None:
    """Restore FENCE_ENABLE so a shared dev vehicle keeps its prior state."""
    import pymavlink.mavutil as mavutil

    connection = connect_operator(port)
    try:
        connection.mav.param_set_send(
            connection.target_system,
            connection.target_component,
            b"FENCE_ENABLE",
            int(previous),
            mavutil.mavlink.MAV_PARAM_TYPE_INT32,
        )
        time.sleep(1.0)
    finally:
        connection.close()


def run_geofence(binary: Path, port: str, operator_port: str) -> None:
    status = wait_for_status(binary, port, {"connected": "true"}, 15.0)
    if status.get("armed") == "true":
        raise ScenarioError("SITL must start disarmed for the geofence scenario")

    previous_enable = read_fence_enable(operator_port)
    output = run_cli(binary, port, "fence-demo")
    if "fence upload verified" not in output:
        raise ScenarioError(f"fence upload/readback was not verified: {output!r}")
    print("C++ SITL geofence upload/readback passed", flush=True)
    restore_fence_enable(operator_port, previous_enable)


def main() -> int:
    try:
        port = get_sitl_port()
        operator_port = get_operator_port()
        binary = find_binary()
        if binary is None:
            print("error: C++ core binary not found; run `pixi run build-core` first", file=sys.stderr)
            return 2
        print_watch_hint()
        run_geofence(binary, port, operator_port)
    except (ValueError, ScenarioError) as error:
        print(f"C++ SITL geofence failed: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
