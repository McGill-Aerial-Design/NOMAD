# SPDX-License-Identifier: Apache-2.0
"""Run the C++ payload command boundary against Docker SITL."""

from __future__ import annotations

import sys

from core_sitl_command_flow import ScenarioError, run_cli, wait_for_status
from core_sitl_status import find_binary, get_sitl_port, print_watch_hint


def main() -> int:
    try:
        port = get_sitl_port()
        binary = find_binary()
        if binary is None:
            print("error: C++ core binary not found; run `pixi run build-core` first", file=sys.stderr)
            return 2
        print_watch_hint()
        initial = wait_for_status(binary, port, {"connected": "true"}, 15.0)
        if initial.get("armed") == "true":
            raise ScenarioError("SITL must start disarmed for payload acceptance")
        output = run_cli(binary, port, "payload-demo", "0", "0.1")
        if "payload release verified" not in output:
            raise ScenarioError(f"payload release was not verified: {output!r}")
        print("C++ SITL payload acceptance passed", flush=True)
        return 0
    except (ValueError, ScenarioError) as error:
        print(f"C++ SITL payload failed: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
