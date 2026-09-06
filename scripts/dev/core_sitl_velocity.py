# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Run the C++ velocity watchdog demo against Docker ArduPilot SITL."""

from __future__ import annotations

import sys
from pathlib import Path

from core_sitl_command_flow import ScenarioError, parse_status, run_cli, wait_for_status
from core_sitl_status import find_binary, get_sitl_port, print_watch_hint


def run_velocity_demo(binary: Path, port: str) -> None:
    wait_for_status(binary, port, {"connected": "true", "armed": "false"}, 15.0)
    run_cli(binary, port, "mode", "4")
    wait_for_status(binary, port, {"mode": "4"}, 15.0)
    run_cli(binary, port, "arm")
    wait_for_status(binary, port, {"armed": "true"}, 15.0)
    output = run_cli(binary, port, "velocity-demo")
    fields = parse_status(output)
    if fields.get("velocity_active") != "false" or fields.get("watchdog_reason") != "command_timeout":
        raise ScenarioError(f"velocity watchdog did not stop the session: {output!r}")
    run_cli(binary, port, "land")
    wait_for_status(binary, port, {"mode": "9"}, 90.0)
    wait_for_status(binary, port, {"armed": "false"}, 90.0)
    print("C++ SITL velocity watchdog passed", flush=True)


def main() -> int:
    try:
        port = get_sitl_port()
        binary = find_binary()
        if binary is None:
            print("error: C++ core binary not found; run `pixi run build-core` first", file=sys.stderr)
            return 2
        print_watch_hint()
        run_velocity_demo(binary, port)
    except ValueError as error:
        print(f"error: {error}", file=sys.stderr)
        return 2
    except ScenarioError as error:
        print(f"C++ SITL velocity watchdog failed: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
