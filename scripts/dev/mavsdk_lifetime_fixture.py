# SPDX-License-Identifier: Apache-2.0
"""Exercise connection generations and publication barriers against the deterministic peer."""

from __future__ import annotations

import subprocess

from mavsdk_fixture_harness import find_binary, find_free_udp_port, require, with_peer
from mavsdk_peer import ACCEPTED, COMMAND_DO_SET_RELAY, VehiclePeer


def main() -> int:
    binary = find_binary("nomad_mavsdk_lifetime_tests")
    if binary is None:
        print("MAVSDK lifetime test binary is missing")
        return 2
    port = find_free_udp_port()
    discovery = subprocess.run(
        [str(binary), "--no-peer", f"udpin:127.0.0.1:{port}"],
        capture_output=True,
        text=True,
        timeout=15,
        check=False,
    )
    require(discovery.returncode == 0, "failed discovery retries with readers", discovery.stderr)

    def action(peer: VehiclePeer) -> None:
        result = subprocess.run(
            [str(binary), f"udpin:127.0.0.1:{port}"],
            capture_output=True,
            text=True,
            timeout=90,
            check=False,
        )
        require(result.returncode == 0, "concurrent resource lifetime", f"{result.stdout}\n{result.stderr}")
        relay_commands = [command for command in peer.commands() if command[1] == COMMAND_DO_SET_RELAY]
        require(not relay_commands, "retired command never reaches wire", str(relay_commands))

    with_peer(port, 1, ACCEPTED, action)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
