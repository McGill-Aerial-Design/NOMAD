# SPDX-License-Identifier: Apache-2.0
"""Prove runtime authority fencing against an independent MAVLink UDP peer."""

from __future__ import annotations

import concurrent.futures
import os
import socket
import time
from pathlib import Path

from mavsdk_authority_peer import AuthorityPeer
from mavsdk_peer import ACCEPTED, COMMAND_DO_SET_SERVO
from runtime_ipc_smoke import (
    ROOT,
    free_port,
    read_logs,
    request,
    start_runtime,
    stop_runtime,
    wait_for_listener,
)

SOURCE = "runtime-smoke"
RETRY_WINDOW_SECONDS = 1.8


def find_runtime() -> Path:
    """Find the release qualification binary or the normal core build."""
    if os.environ.get("NOMAD_RESOURCE_BUILD_DIR"):
        from resource_footprint import release_binary

        return release_binary(Path(os.environ["NOMAD_RESOURCE_BUILD_DIR"]), "nomad-runtime")
    names = ("nomad-runtime.exe", "nomad-runtime")
    for base in (ROOT / "build/mavsdk-qualification", ROOT / "build/core"):
        for directory in (base / "Release", base / "Debug", base):
            for name in names:
                candidate = directory / name
                if candidate.is_file():
                    return candidate
    raise FileNotFoundError("build nomad-runtime before running the authority wire fixture")


def current_context(port: int) -> dict[str, object]:
    """Read the runtime's current fencing fields from its public status response."""
    status = request(port, "wire-status", "status")["status"]
    return {
        "runtime_incarnation": status["runtime_incarnation"],
        "vehicle_session": status["vehicle_session"],
        "authority_generation": status["authority_generation"],
    }


def bound_request(port: int, request_id: str, request_type: str, sequence: int, **fields: object):
    """Send one typed request with an explicit source, generation, and expiry."""
    context = current_context(port)
    return request(
        port,
        request_id,
        request_type,
        command_source=SOURCE,
        sequence=sequence,
        expires_at_ms=int(time.time() * 1000) + 4000,
        **context,
        **fields,
    )


def servo_request(port: int, request_id: str, sequence: int, pwm: int):
    """Issue one runtime-owned COMMAND_LONG through the production transport."""
    return bound_request(
        port,
        request_id,
        "set_servo",
        sequence,
        channel=8,
        pwm_microseconds=pwm,
    )


def servo_frames(peer: AuthorityPeer):
    """Read only SET_SERVO frames decoded from the peer's UDP socket."""
    return [frame for frame in peer.commands() if frame[0] == "COMMAND_LONG" and frame[1] == COMMAND_DO_SET_SERVO]


def wait_for_frames(peer: AuthorityPeer, count: int, timeout: float = 5.0) -> None:
    """Wait for a physical frame, with an observed-count failure diagnostic."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if len(servo_frames(peer)) >= count:
            return
        time.sleep(0.01)
    raise AssertionError(f"peer observed {len(servo_frames(peer))} SET_SERVO frames; expected {count}")


def wait_for_session(port: int, timeout: float = 10.0) -> None:
    """Wait until the fake vehicle has established a fresh runtime session."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        status = request(port, "wire-ready", "status")["status"]
        if status["vehicle_connected"] and status["vehicle_session"]:
            return
        time.sleep(0.05)
    raise AssertionError("runtime never established a vehicle session")


def verify_retry_control(peer: AuthorityPeer, port: int) -> None:
    """With no revocation and no ACK, MAVSDK must retransmit on the wire."""
    assert bound_request(port, "control-admit", "admit_authority", 1)["ok"]
    result = servo_request(port, "control-servo", 1, 1400)
    assert result["command_result"]["success"] is False, result
    wait_for_frames(peer, 2)
    assert bound_request(port, "control-revoke", "revoke_authority", 2)["ok"]


def verify_revoke_and_handback(peer: AuthorityPeer, port: int) -> None:
    """Revoke between first send and retry; then prove fresh handback traffic."""
    before = len(servo_frames(peer))
    assert bound_request(port, "wire-handback", "handback_authority", 1)["ok"]
    old_context = current_context(port)
    with concurrent.futures.ThreadPoolExecutor(max_workers=1) as worker:
        pending = worker.submit(servo_request, port, "wire-old-command", 1, 1500)
        wait_for_frames(peer, before + 1)
        assert bound_request(port, "wire-revoke", "revoke_authority", 2)["ok"]
        interrupted = pending.result(timeout=8)
    assert interrupted["error"]["code"] == "authority_interrupted", interrupted
    time.sleep(RETRY_WINDOW_SECONDS)
    assert len(servo_frames(peer)) == before + 1, "revoked command retransmitted to UDP peer"

    peer.set_ack_result(ACCEPTED)
    assert bound_request(port, "wire-fresh-handback", "handback_authority", 1)["ok"]
    fresh = servo_request(port, "wire-fresh-command", 1, 1600)
    assert fresh["command_result"]["success"] is True, fresh
    wait_for_frames(peer, before + 2)
    stale = request(
        port,
        "wire-stale-replay",
        "set_servo",
        channel=8,
        pwm_microseconds=1500,
        command_source=SOURCE,
        sequence=2,
        expires_at_ms=int(time.time() * 1000) + 3000,
        **old_context,
    )
    assert stale["error"]["code"] == "stale_authority", stale
    time.sleep(RETRY_WINDOW_SECONDS)
    assert len(servo_frames(peer)) == before + 2, "old generation appeared after handback"


def verify_shutdown_pending(peer: AuthorityPeer, port: int, process) -> None:
    """Shut the production runtime down with a real retry still outstanding."""
    before = len(servo_frames(peer))
    peer.set_ack_result(None)
    with concurrent.futures.ThreadPoolExecutor(max_workers=1) as worker:
        pending = worker.submit(servo_request, port, "wire-shutdown-command", 2, 1700)
        wait_for_frames(peer, before + 1)
        stop_runtime(process)
        try:
            result = pending.result(timeout=8)
        except (ConnectionError, OSError):
            result = None
    if result is not None:
        assert result.get("command_result", {}).get("success") is not True, result
    time.sleep(RETRY_WINDOW_SECONDS)
    assert len(servo_frames(peer)) == before + 1, "runtime shutdown emitted a retry"


def verify_session_rollover(peer: AuthorityPeer, port: int) -> None:
    """A lost vehicle session revokes authority and requires explicit handback."""
    before = len(servo_frames(peer))
    old_context = current_context(port)
    peer.set_streaming(False)
    deadline = time.monotonic() + 12.0
    while time.monotonic() < deadline:
        status = request(port, "wire-session-loss", "status")["status"]
        if status["authority_owner"] is None and status["authority_generation"] != old_context["authority_generation"]:
            break
        time.sleep(0.1)
    else:
        raise AssertionError("vehicle-session loss did not revoke runtime authority")

    peer.set_streaming(True)
    wait_for_session(port)
    denied = servo_request(port, "wire-no-reclaim", 1, 1650)
    assert denied["ok"] is False, denied
    assert len(servo_frames(peer)) == before, "reconnect silently reclaimed authority"
    assert bound_request(port, "wire-session-handback", "handback_authority", 1)["ok"]
    fresh = servo_request(port, "wire-post-session-command", 1, 1650)
    assert fresh["command_result"]["success"] is True, fresh
    wait_for_frames(peer, before + 1)


def main() -> None:
    """Run both controls against a real MAVSDK runtime and socket observer."""
    udp_port = free_port(socket.SOCK_DGRAM)
    ipc_port = free_port(socket.SOCK_STREAM)
    peer = AuthorityPeer(udp_port, 1, ack_result=None)
    peer.start()
    process, stdout, stderr = start_runtime(find_runtime(), udp_port, ipc_port)
    try:
        wait_for_listener(ipc_port)
        wait_for_session(ipc_port)
        verify_retry_control(peer, ipc_port)
        verify_revoke_and_handback(peer, ipc_port)
        verify_session_rollover(peer, ipc_port)
        verify_shutdown_pending(peer, ipc_port, process)
    finally:
        stop_runtime(process)
        peer.stop()
    assert process.returncode == 0, read_logs(stdout, stderr)
    stdout.close()
    stderr.close()
    print("MAVSDK authority wire fixture passed")


if __name__ == "__main__":
    main()
