# SPDX-License-Identifier: Apache-2.0
"""Drive the COMMAND_LONG/INT wire probe with an independent UDP observer."""

from __future__ import annotations

import os
import queue
import socket
import subprocess
import threading
import time
from pathlib import Path

from mavsdk_authority_peer import AuthorityPeer
from mavsdk_peer import ACCEPTED, COMMAND_DO_REPOSITION, COMMAND_DO_SET_SERVO
from runtime_ipc_smoke import ROOT, free_port

RETRY_WINDOW_SECONDS = 1.8


def find_probe() -> Path:
    """Find the CMake-built interactive MAVSDK test probe."""
    if os.environ.get("NOMAD_RESOURCE_BUILD_DIR"):
        from resource_footprint import release_binary

        return release_binary(Path(os.environ["NOMAD_RESOURCE_BUILD_DIR"]), "nomad_mavsdk_authority_wire_probe")
    names = ("nomad_mavsdk_authority_wire_probe.exe", "nomad_mavsdk_authority_wire_probe")
    for base in (ROOT / "build/mavsdk-qualification", ROOT / "build/core"):
        for directory in (base / "Release", base / "Debug", base):
            for name in names:
                candidate = directory / name
                if candidate.is_file():
                    return candidate
    raise FileNotFoundError("build nomad_mavsdk_authority_wire_probe first")


class Probe:
    """Keep the test process responsive while commands wait for SDK retries."""

    def __init__(self, endpoint: str) -> None:
        self.process = subprocess.Popen(
            [str(find_probe()), endpoint],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1,
        )
        self.lines: queue.Queue[str] = queue.Queue()
        self.backlog: list[str] = []
        self.reader = threading.Thread(target=self._read, daemon=True)
        self.reader.start()

    def _read(self) -> None:
        assert self.process.stdout is not None
        for line in self.process.stdout:
            self.lines.put(line.strip())

    def command(self, value: str) -> None:
        assert self.process.stdin is not None
        self.process.stdin.write(value + "\n")
        self.process.stdin.flush()

    def expect(self, expected: str, timeout: float = 8.0) -> None:
        deadline = time.monotonic() + timeout
        seen: list[str] = []
        while time.monotonic() < deadline:
            if expected in self.backlog:
                self.backlog.remove(expected)
                return
            try:
                line = self.lines.get(timeout=min(0.1, deadline - time.monotonic()))
            except queue.Empty:
                continue
            seen.append(line)
            if line == expected:
                return
            self.backlog.append(line)
        raise AssertionError(f"probe did not report {expected!r}; observed {seen!r}")

    def stop(self) -> None:
        if self.process.poll() is None:
            self.command("stop")
            self.expect("stopped")
            self.process.wait(timeout=8)
        assert self.process.returncode == 0, self.process.stderr.read()


def command_count(peer: AuthorityPeer, kind: str) -> int:
    """Count decoded command frames at the peer, not SDK method calls."""
    command_id = COMMAND_DO_REPOSITION if kind == "int" else COMMAND_DO_SET_SERVO
    wire_kind = "COMMAND_INT" if kind == "int" else "COMMAND_LONG"
    return sum(frame[0] == wire_kind and frame[1] == command_id for frame in peer.commands())


def wait_for_command(peer: AuthorityPeer, kind: str, count: int) -> None:
    """Wait for the first physical frame of an operation."""
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        if command_count(peer, kind) >= count:
            return
        time.sleep(0.01)
    raise AssertionError(f"peer saw {command_count(peer, kind)} {kind} frames; expected {count}")


def verify_kind(probe: Probe, peer: AuthorityPeer, kind: str) -> None:
    """Revoke after first frame, then hand back and send a fresh command."""
    before = command_count(peer, kind)
    peer.set_ack_result(None)
    probe.command("admit" if kind == "long" else "handback")
    probe.expect("admit=ok" if kind == "long" else "handback=ok")
    probe.command(kind)
    probe.expect(f"{kind}=started")
    wait_for_command(peer, kind, before + 1)
    probe.command("revoke")
    probe.expect("revoke=ok")
    probe.expect(f"{kind}=cancelled_or_unacknowledged")
    time.sleep(RETRY_WINDOW_SECONDS)
    assert command_count(peer, kind) == before + 1, f"revoked {kind} command retried"

    peer.set_ack_result(ACCEPTED)
    probe.command("handback")
    probe.expect("handback=ok")
    probe.command(kind)
    probe.expect(f"{kind}=started")
    wait_for_command(peer, kind, before + 2)
    probe.expect(f"{kind}=accepted")
    time.sleep(RETRY_WINDOW_SECONDS)
    assert command_count(peer, kind) == before + 2, f"old {kind} command resurrected"


def verify_pending_shutdown(probe: Probe, peer: AuthorityPeer) -> None:
    """Stop the SDK while an unacknowledged command is still pending."""
    before = command_count(peer, "long")
    peer.set_ack_result(None)
    probe.command("long")
    probe.expect("long=started")
    wait_for_command(peer, "long", before + 1)
    probe.stop()
    time.sleep(RETRY_WINDOW_SECONDS)
    assert command_count(peer, "long") == before + 1, "shutdown emitted a late retry"


def verify_cancel_before_first_frame(probe: Probe, peer: AuthorityPeer) -> None:
    """A same-ID operation waiting behind another is denied before its first frame."""
    before = command_count(peer, "long")
    peer.set_ack_result(None)
    probe.command("long")
    probe.expect("long=started")
    wait_for_command(peer, "long", before + 1)
    probe.command("long")
    probe.expect("long=started")
    probe.command("revoke")
    probe.expect("revoke=ok")
    probe.expect("long=cancelled_or_unacknowledged")
    probe.expect("long=cancelled_or_unacknowledged")
    time.sleep(RETRY_WINDOW_SECONDS)
    assert command_count(peer, "long") == before + 1, "queued stale command reached the peer"


def main() -> None:
    """Qualify both command encodings and pending shutdown on a real socket."""
    port = free_port(socket.SOCK_DGRAM)
    peer = AuthorityPeer(port, 1, ack_result=None)
    peer.start()
    probe = Probe(f"udpin:127.0.0.1:{port}")
    try:
        probe.expect("ready")
        verify_kind(probe, peer, "long")
        verify_cancel_before_first_frame(probe, peer)
        verify_kind(probe, peer, "int")
        verify_pending_shutdown(probe, peer)
    finally:
        if probe.process.poll() is None:
            probe.stop()
        peer.stop()
    print("MAVSDK authority command probe passed")


if __name__ == "__main__":
    main()
