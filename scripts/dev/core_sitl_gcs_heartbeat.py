# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Prove the GCS heartbeat opens a heartbeat-gated UDP relay leg against SITL.

Real mavlink-router and MAVProxy setups only start streaming a UDP leg after
the endpoint announces itself with a GCS heartbeat. The C++ core therefore
emits a 1 Hz standard GCS heartbeat while it waits for the vehicle. MAVSDK does
that: the transport configures its ``Mavsdk`` as ``ComponentType::GroundStation``
(``src/mavlink/mavsdk_mavlink_connection.cpp``), and
``test_unlatched_link_announces_a_gcs_heartbeat`` in
``tests/test_mavsdk_connection.py`` requires the cadence on the wire before any
autopilot is latched.

The scenario places a heartbeat-gated UDP relay between the always-on SITL
stream (host UDP 14572, see docker-compose.dev.yml) and a fresh client port,
then runs the C++ CLI ``status`` verb through it twice:

1. Gate open: the relay starts forwarding the vehicle stream only after it
   sees a datagram from the client (mavlink-router behavior). The status read
   succeeds, the relayed datagrams are counted, and every captured client
   announcement is verified to be a standard GCS heartbeat (sysid 245,
   compid 190, MAV_TYPE_GCS) at ~1 Hz.
2. Gate closed (negative control): a relay copy that drops client
   announcements never forwards anything, so the status read must fail closed
   with the heartbeat-timeout diagnostic. The captured announcements prove the
   CLI actually announced — so the failure is the closed gate, not a missing
   announcement.

The status read is the authoritative check (telemetry through the whole relay
path), not a command acknowledgement.

The CLI uses a MAVSDK ``udpout:`` endpoint aimed at the relay's routable
upstream port. MAVSDK chooses the local source port, and the relay learns that
source from the pre-latch announcement before forwarding the SITL stream back.
The relay binds the SITL stream port as its upstream — the same convention
core-sitl-link-recovery uses.

Run against the dev stack (see tests/sitl/README.md):

    pixi run core-sitl-gcs-heartbeat
"""

from __future__ import annotations

import math
import os
import socket
import subprocess
import sys
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts" / "dev"))

from core_sitl_status import find_binary, print_watch_hint  # noqa: E402

MIN_CADENCE_ANNOUNCEMENTS = 4
MIN_ANNOUNCEMENT_INTERVAL_S = 0.9
MAX_ANNOUNCEMENT_INTERVAL_S = 1.3
CADENCE_COMPARISON_EPSILON_S = 1e-9
GCS_SYSTEM_ID = 245
GCS_COMPONENT_ID = 190


class ScenarioError(AssertionError):
    """A scenario assertion failed (the SC behaviour was not observed)."""


def _now_s() -> float:
    return time.monotonic()


class GatedRelay:
    """Heartbeat-gated one-way UDP relay: upstream stream -> client endpoint.

    Datagrams from the SITL stream are forwarded to the UDP source that sent a
    valid GCS heartbeat only once ``gate_opens`` is true. MAVSDK's ``udpout``
    connection chooses its local source port, so the relay must learn that
    address from the announcement instead of assuming a fixed client port. A
    relay with ``gate_opens`` false captures announcements but never forwards,
    which models a leg that stayed closed.
    """

    def __init__(self, upstream_port: int, gate_opens: bool) -> None:
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.bind(("0.0.0.0", upstream_port))
        self._socket.settimeout(0.1)
        self._client: tuple[str, int] | None = None
        self._gate_opens = gate_opens
        self._saw_client = False
        self._running = True
        self.announcements: list[tuple[float, bytes]] = []
        self.forwarded = 0
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self) -> None:
        while self._running:
            try:
                data, sender = self._socket.recvfrom(4096)
            except TimeoutError:
                continue
            except OSError as error:
                # Windows turns an ICMP port-unreachable into WSAECONNRESET on
                # the next recvfrom; that is normal relay traffic, not death.
                if getattr(error, "winerror", None) == 10054:
                    continue
                return
            heartbeat = parse_mavlink_heartbeat(data)
            if heartbeat and heartbeat["sysid"] == GCS_SYSTEM_ID and heartbeat["compid"] == GCS_COMPONENT_ID:
                self.announcements.append((_now_s(), data))
                self._client = sender
                self._saw_client = True
                continue
            if self._gate_opens and self._saw_client and self._client is not None:
                self._socket.sendto(data, self._client)
                self.forwarded += 1

    def close(self) -> None:
        self._running = False
        self._thread.join(timeout=1.0)
        self._socket.close()


def parse_mavlink_heartbeat(frame: bytes) -> dict[str, int] | None:
    """Decode the heartbeat fields the announcement must carry, or None."""
    if len(frame) < 21 or frame[0] != 0xFD or frame[7] != 0:
        return None
    return {
        "sysid": frame[5],
        "compid": frame[6],
        "type": frame[14],
        "autopilot": frame[15],
    }


def verify_announcements(announced: list[tuple[float, bytes]], require_cadence: bool) -> None:
    """Require standard GCS heartbeats and, when requested, their cadence."""
    if not announced:
        raise ScenarioError("no announcements captured; cannot prove the GCS heartbeat opened the gate")
    for _, frame in announced:
        heartbeat = parse_mavlink_heartbeat(frame)
        if heartbeat is None:
            raise ScenarioError(f"announced frame is not a heartbeat: {frame.hex()}")
        if heartbeat["sysid"] != GCS_SYSTEM_ID or heartbeat["compid"] != GCS_COMPONENT_ID:
            raise ScenarioError(f"heartbeat is not GCS-sourced: {heartbeat}")
        if heartbeat["type"] != 6 or heartbeat["autopilot"] != 8:
            raise ScenarioError(f"heartbeat is not MAV_TYPE_GCS/MAV_AUTOPILOT_INVALID: {heartbeat}")
    if not require_cadence:
        return
    if len(announced) < MIN_CADENCE_ANNOUNCEMENTS:
        raise ScenarioError(
            f"fewer than {MIN_CADENCE_ANNOUNCEMENTS} announcements captured; cannot prove heartbeat cadence"
        )
    intervals = [current[0] - previous[0] for previous, current in zip(announced, announced[1:])]
    if not all(math.isfinite(interval) for interval in intervals):
        raise ScenarioError("announcement cadence contains a non-finite interval")
    fastest = min(intervals)
    slowest = max(intervals)
    if fastest < MIN_ANNOUNCEMENT_INTERVAL_S - CADENCE_COMPARISON_EPSILON_S:
        raise ScenarioError(f"announcement interval {fastest:.3f}s is faster than the 1 Hz tolerance")
    if slowest > MAX_ANNOUNCEMENT_INTERVAL_S + CADENCE_COMPARISON_EPSILON_S:
        raise ScenarioError(f"announcement interval {slowest:.3f}s is slower than the 1 Hz tolerance")


def measured_rate(announced: list[tuple[float, bytes]]) -> float | None:
    if len(announced) < 2:
        return None
    elapsed = announced[-1][0] - announced[0][0]
    return (len(announced) - 1) / max(elapsed, 1e-6)


def run_status(binary: Path, endpoint: str) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [str(binary), "status", "--endpoint", endpoint],
        capture_output=True,
        text=True,
        check=False,
        env=os.environ.copy(),
    )


def relay_endpoint(upstream_port: int) -> str:
    """Use MAVSDK's outbound UDP mode so the relay can learn our source port."""
    return f"udpout:127.0.0.1:{upstream_port}"


def run_gate_open(binary: Path, upstream_port: int) -> None:
    """Announcements flow, the relay opens, and status reads live telemetry."""
    relay = GatedRelay(upstream_port, gate_opens=True)
    try:
        started = _now_s()
        result = run_status(binary, relay_endpoint(upstream_port))
        elapsed = _now_s() - started
        if result.returncode != 0:
            # Record what the run did prove before failing closed: the
            # announcement cadence and gate-open behaviour may be healthy even
            # when a lossy relay starves the telemetry itself.
            if relay.announcements:
                verify_announcements(relay.announcements, require_cadence=False)
                print(
                    f"diagnostic: {len(relay.announcements)} valid announcements and "
                    f"{relay.forwarded} datagrams relayed, but status failed (lossy relay?)",
                    flush=True,
                )
            raise ScenarioError(
                f"status through the heartbeat-gated relay failed after {elapsed:.1f}s: "
                f"{(result.stderr or result.stdout).strip()}"
            )
        if relay.forwarded == 0:
            raise ScenarioError("status succeeded but no vehicle datagrams were relayed")
        verify_announcements(relay.announcements, require_cadence=False)
        print(
            f"gate open: status succeeded in {elapsed:.1f}s; {relay.forwarded} vehicle datagrams relayed; "
            f"{len(relay.announcements)} valid GCS heartbeat announcements",
            flush=True,
        )
    finally:
        relay.close()


def run_gate_closed(binary: Path, upstream_port: int) -> None:
    """Even after announcements, a closed gate must fail status closed."""
    relay = GatedRelay(upstream_port, gate_opens=False)
    try:
        started = _now_s()
        result = run_status(binary, relay_endpoint(upstream_port))
        elapsed = _now_s() - started
        if not relay.announcements:
            raise ScenarioError("CLI announced nothing on the closed gate; the negative control proves nothing")
        verify_announcements(relay.announcements, require_cadence=True)
        if result.returncode == 0:
            raise ScenarioError("status succeeded through a closed gate; the gate is not actually heartbeat-gated")
        if "timed out waiting for ArduPilot heartbeat" not in result.stderr:
            raise ScenarioError(f"unexpected failure through the closed gate: {result.stderr.strip()}")
        rate = measured_rate(relay.announcements)
        assert rate is not None
        print(
            f"gate closed: {len(relay.announcements)} announcements at ~{rate:.1f} Hz captured then dropped; "
            f"status failed closed after {elapsed:.1f}s",
            flush=True,
        )
    finally:
        relay.close()


def get_upstream_port() -> int:
    """The always-on SITL copy the scenario's relay binds as its upstream.

    The dev stack forwards a MAVLink copy to host UDP 14572 (see
    docker-compose.dev.yml), so the relay can own that port without a
    special stack restart — the same convention core-sitl-link-recovery uses.
    """
    port = os.environ.get("NOMAD_GCS_HEARTBEAT_UPSTREAM", "14572")
    if not port.isdecimal() or not 1 <= int(port) <= 65535:
        raise ValueError("NOMAD_GCS_HEARTBEAT_UPSTREAM must be a UDP port from 1 to 65535")
    return int(port)


def main() -> int:
    try:
        upstream_port = get_upstream_port()
        binary = find_binary()
        if binary is None:
            print(
                "error: qualification driver not found; run `pixi run build-qualification-cli` first", file=sys.stderr
            )
            return 2
        print_watch_hint()
        run_gate_open(binary, upstream_port)
        run_gate_closed(binary, upstream_port)
        return 0
    except (ValueError, ScenarioError, OSError) as error:
        print(f"C++ SITL GCS-heartbeat scenario failed: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
