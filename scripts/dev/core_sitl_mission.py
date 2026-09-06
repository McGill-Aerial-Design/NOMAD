# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Run the C++ mission executor against the Docker ArduPilot SITL vehicle."""

from __future__ import annotations

import sys
from pathlib import Path

from core_sitl_command_flow import (
    ScenarioError,
    parse_status,
    run_cli,
    wait_for_status,
)
from core_sitl_status import find_binary, get_sitl_port, print_watch_hint


def run_mission(binary: Path, port: str) -> None:
    initial = wait_for_status(binary, port, {"connected": "true"}, 15.0)
    if initial.get("armed") == "true":
        raise ScenarioError("SITL must start disarmed for the C++ mission scenario")
    output = run_cli(binary, port, "mission-demo")
    fields = parse_status(output)
    # The executor itself verifies each step's authoritative state, including
    # the takeoff altitude (Vehicle::takeoff waits for 80% of the target). The
    # scenario only needs the mission to have reported all six steps complete.
    if fields.get("completed_steps") != "6" or "mission completed" not in output:
        raise ScenarioError(f"mission executor did not complete all steps: {output!r}")
    wait_for_status(binary, port, {"mode": "9"}, 90.0)
    wait_for_status(binary, port, {"armed": "false"}, 90.0)
    print("C++ SITL mission passed", flush=True)


def main() -> int:
    try:
        port = get_sitl_port()
        binary = find_binary()
        if binary is None:
            print("error: C++ core binary not found; run `pixi run build-core` first", file=sys.stderr)
            return 2
        print_watch_hint()
        run_mission(binary, port)
    except ValueError as error:
        print(f"error: {error}", file=sys.stderr)
        return 2
    except ScenarioError as error:
        print(f"C++ SITL mission failed: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
