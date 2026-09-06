# SPDX-License-Identifier: Apache-2.0
"""Exercise the C++ link-loss watchdog boundary against Docker SITL."""

from __future__ import annotations

import subprocess
import sys
import time
from pathlib import Path

from core_sitl_command_flow import ScenarioError, run_cli, wait_for_status
from core_sitl_status import find_binary, get_sitl_port, print_watch_hint


def run_link_loss(binary: Path, port: str) -> None:
    wait_for_status(binary, port, {"connected": "true"}, 15.0)
    run_cli(binary, port, "mode", "4")
    wait_for_status(binary, port, {"mode": "4"}, 15.0)
    run_cli(binary, port, "arm")
    wait_for_status(binary, port, {"armed": "true"}, 15.0)

    # The C++ process owns its UDP socket. Terminating it is the supported
    # software-side link-loss injection; Vehicle destruction sends zero before
    # the transport closes, while ArduPilot remains untouched.
    process = subprocess.Popen(
        [str(binary), "velocity-demo", "--endpoint", f"udpin:0.0.0.0:{port}"],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    time.sleep(0.2)
    process.terminate()
    try:
        process.wait(timeout=5.0)
    except subprocess.TimeoutExpired as error:
        process.kill()
        raise ScenarioError("velocity process did not stop after link-loss injection") from error
    if process.returncode == 0:
        raise ScenarioError("link-loss injection unexpectedly reported success")
    print("C++ SITL link-loss injection stopped the client safely", flush=True)


def main() -> int:
    try:
        port = get_sitl_port()
        binary = find_binary()
        if binary is None:
            print("error: C++ core binary not found; run `pixi run build-core` first", file=sys.stderr)
            return 2
        print_watch_hint()
        run_link_loss(binary, port)
        return 0
    except (ValueError, ScenarioError) as error:
        print(f"C++ SITL link-loss failed: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
