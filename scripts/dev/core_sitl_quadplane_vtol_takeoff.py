# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Qualify the pinned QuadPlane GUIDED arm and VTOL takeoff path."""

from __future__ import annotations

import sys
import time
from pathlib import Path

from core_sitl_quadplane_observe import (
    EXPECTED_IDENTITY,
    ScenarioError,
    get_sitl_port,
    read_status,
    require_fresh_telemetry,
    require_status,
    run_cli,
)
from core_sitl_status import find_binary

TARGET_CLIMB_M = 5.0
COMPLETION_TOLERANCE_M = 0.5
MINIMUM_VERIFIED_ALTITUDE_M = TARGET_CLIMB_M - COMPLETION_TOLERANCE_M


def parse_relative_altitude(fields: dict[str, str]) -> float:
    try:
        return float(fields["relative_altitude_m"])
    except (KeyError, ValueError) as error:
        raise ScenarioError(f"missing or invalid relative_altitude_m; full status={fields}") from error


def require_climb_status(fields: dict[str, str], target_altitude_m: float) -> None:
    require_status(fields, {"connected": "true", "heartbeat_fresh": "true", **EXPECTED_IDENTITY})
    require_status(fields, {"armed": "true", "mode": "15"})
    require_fresh_telemetry(fields)
    altitude_m = parse_relative_altitude(fields)
    minimum_altitude_m = target_altitude_m - COMPLETION_TOLERANCE_M
    if altitude_m < minimum_altitude_m:
        raise ScenarioError(
            f"QuadPlane climb is below {minimum_altitude_m:.1f} m: {altitude_m:.2f}; full status={fields}"
        )


def wait_for_climb(binary: Path, port: str, target_altitude_m: float, timeout: float = 30.0) -> dict[str, str]:
    deadline = time.monotonic() + timeout
    last: dict[str, str] = {}
    while time.monotonic() < deadline:
        last = read_status(binary, port)
        try:
            require_climb_status(last, target_altitude_m)
        except ScenarioError:
            continue
        return last
    raise ScenarioError(f"QuadPlane VTOL takeoff did not verify actual climb; last status={last}")


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
        baseline_altitude_m = parse_relative_altitude(initial)
        target_altitude_m = baseline_altitude_m + TARGET_CLIMB_M

        result = run_cli(binary, port, "vtol-takeoff", str(TARGET_CLIMB_M))
        if "vtol takeoff verified" not in result:
            raise ScenarioError(f"NOMAD did not report verified VTOL takeoff: {result!r}")
        final = wait_for_climb(binary, port, target_altitude_m)
    except (ValueError, ScenarioError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 1
    print(
        "QuadPlane arm + VTOL takeoff qualification passed: "
        f"baseline_relative_altitude_m={baseline_altitude_m:.2f} target_relative_altitude_m={target_altitude_m:.2f} "
        f"armed={final['armed']} mode={final['mode']} relative_altitude_m={final['relative_altitude_m']}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
