# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Prove degraded-link behavior against Docker SITL.

The ground-station configurations rely on link aggregation (LTE/Tailscale +
ELRS). This scenario simulates a whole ground link dropping and returning: a
bidirectional UDP relay sits between the SITL stream and the C++ client, and
can be paused. It verifies the degraded-link semantics documented in
docs/operations.md:

- link up: telemetry and commands work through the relay;
- link down: the client fails closed (reports disconnected, refuses);
- link back: telemetry and commands work again with no client restart.

The relay binds the SITL stream port as its upstream, so the stack must
forward the SITL copy to a port the relay can own. Run the stack with
``NOMAD_CORE_SITL_PORT`` set to a non-published port (the pixi task uses
14572) or the relay cannot bind it.
"""

from __future__ import annotations

import socket
import subprocess
import sys
import threading
from pathlib import Path

from core_sitl_command_flow import ScenarioError, run_cli, wait_for_status
from core_sitl_status import find_binary, get_sitl_port, print_watch_hint


class UdpRelay:
    """Bidirectional UDP proxy with a pause switch, on a single socket.

    The C++ CLI binds the client port itself and, once it latches the relay as
    its peer, sends replies to the relay socket. So one socket bound on the
    upstream port is enough: telemetry from the SITL stream is forwarded to
    the client port, and datagrams from the client are forwarded back to the
    SITL peer. Pausing drains the socket and forwards nothing, which looks to
    the client like a lost ground link.
    """

    def __init__(self, upstream_port: int, client_port: int) -> None:
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.bind(("0.0.0.0", upstream_port))
        self._client_address = ("127.0.0.1", client_port)
        self._sitl_peer: tuple[str, int] | None = None
        self._paused = threading.Event()
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self) -> None:
        while self._running:
            try:
                data, sender = self._socket.recvfrom(4096)
            except OSError as error:
                # Windows turns an ICMP port-unreachable (a datagram sent into
                # the gap between two client processes) into WSAECONNRESET on
                # the next recvfrom; that is normal relay traffic, not death.
                if getattr(error, "winerror", None) == 10054:
                    continue
                return
            if self._paused.is_set():
                continue
            if sender == self._client_address:
                if self._sitl_peer is not None:
                    self._socket.sendto(data, self._sitl_peer)
                continue
            self._sitl_peer = sender
            self._socket.sendto(data, self._client_address)

    def pause(self) -> None:
        self._paused.set()

    def resume(self) -> None:
        self._paused.clear()

    def close(self) -> None:
        self._running = False
        self._socket.close()


def status_fails_closed(binary: Path, port: str) -> None:
    """A status read over a dead link must fail with the documented diagnostic."""
    result = subprocess.run(
        [str(binary), "status", "--endpoint", f"udpin:0.0.0.0:{port}"],
        capture_output=True,
        text=True,
        check=False,
    )
    if result.returncode == 0:
        raise ScenarioError("status succeeded while the link was down")
    if "timed out waiting for ArduPilot heartbeat" not in result.stderr:
        raise ScenarioError(f"unexpected failure while the link was down: {result.stderr.strip()}")


def run_link_recovery(binary: Path, upstream_port: str) -> None:
    relay = UdpRelay(int(upstream_port), int(upstream_port) + 1)
    try:
        port = str(int(upstream_port) + 1)
        wait_for_status(binary, port, {"connected": "true"}, 15.0)
        run_cli(binary, port, "mode", "4")
        wait_for_status(binary, port, {"mode": "4"}, 15.0)
        run_cli(binary, port, "arm")
        wait_for_status(binary, port, {"armed": "true"}, 15.0)
        run_cli(binary, port, "disarm")
        wait_for_status(binary, port, {"armed": "false"}, 15.0)
        print("link up: telemetry and commands work through the relay", flush=True)

        relay.pause()
        status_fails_closed(binary, port)
        print("link down: client failed closed (reports disconnected)", flush=True)

        relay.resume()
        wait_for_status(binary, port, {"connected": "true"}, 15.0)
        run_cli(binary, port, "mode", "4")
        wait_for_status(binary, port, {"mode": "4"}, 15.0)
        run_cli(binary, port, "arm")
        wait_for_status(binary, port, {"armed": "true"}, 15.0)
        run_cli(binary, port, "disarm")
        wait_for_status(binary, port, {"armed": "false"}, 15.0)
        print("link back: telemetry and commands work again without a client restart", flush=True)
    finally:
        relay.close()


def main() -> int:
    try:
        upstream_port = get_sitl_port()
        binary = find_binary()
        if binary is None:
            print("error: C++ core binary not found; run `pixi run build-core` first", file=sys.stderr)
            return 2
        print_watch_hint()
        run_link_recovery(binary, upstream_port)
        return 0
    except (ValueError, ScenarioError) as error:
        print(f"C++ SITL link-recovery failed: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
