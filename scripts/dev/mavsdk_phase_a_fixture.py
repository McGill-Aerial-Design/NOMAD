# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Run deterministic MAVSDK Phase A positive and negative UDP peer cases."""

from __future__ import annotations

import socket
import subprocess
import threading
from pathlib import Path

from mavsdk_phase_a_smoke import find_binary, has_required_output
from pymavlink.dialects.v20 import ardupilotmega as mavlink


class ArduPilotPeer:
    def __init__(self, port: int, system_id: int) -> None:
        self._address = ("127.0.0.1", port)
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._mavlink = mavlink.MAVLink(None, srcSystem=system_id, srcComponent=1)
        self._position_boot_ms = 1000
        self._gps_time_usec = 1_000_000
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._send_until_stopped, daemon=True)

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2)
        self._socket.close()

    def _send(self, message) -> None:
        packet = message.pack(self._mavlink)
        self._mavlink.seq = (self._mavlink.seq + 1) & 0xFF
        self._socket.sendto(packet, self._address)

    def _send_until_stopped(self) -> None:
        while not self._stop.wait(0.2):
            self._send_telemetry()

    def _send_telemetry(self) -> None:
        self._send(
            self._mavlink.heartbeat_encode(
                mavlink.MAV_TYPE_QUADROTOR,
                mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
                mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                4,
                mavlink.MAV_STATE_ACTIVE,
            )
        )
        self._position_boot_ms += 200
        self._gps_time_usec += 200_000
        self._send(
            self._mavlink.global_position_int_encode(
                self._position_boot_ms,
                int(45.5017 * 1e7),
                int(-73.5673 * 1e7),
                25000,
                8000,
                0,
                0,
                0,
                0,
            )
        )
        self._send(self._gps_message())
        self._send(self._system_status_message())

    def _gps_message(self):
        return self._mavlink.gps_raw_int_encode(
            self._gps_time_usec,
            3,
            int(45.5017 * 1e7),
            int(-73.5673 * 1e7),
            25000,
            100,
            100,
            0,
            0,
            12,
        )

    def _system_status_message(self):
        return self._mavlink.sys_status_encode(
            0xFFFFFFFF,
            0xFFFFFFFF,
            0xFFFFFFFF,
            100,
            12600,
            -1,
            75,
            0,
            0,
            0,
            0,
            0,
            0,
        )


def find_free_udp_port() -> int:
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    probe.bind(("127.0.0.1", 0))
    port = probe.getsockname()[1]
    probe.close()
    return port


def run_binary(binary: Path, command: str, port: int, system_id: int, timeout: int = 15) -> subprocess.CompletedProcess:
    endpoint = f"udpin:127.0.0.1:{port}"
    return subprocess.run(
        [str(binary), command, endpoint, str(system_id)],
        capture_output=True,
        text=True,
        timeout=timeout,
        check=False,
    )


def run_peer_case(binary: Path, peer_id: int, expected_id: int, command: str) -> subprocess.CompletedProcess:
    port = find_free_udp_port()
    peer = ArduPilotPeer(port, peer_id)
    peer.start()
    try:
        return run_binary(binary, command, port, expected_id)
    finally:
        peer.stop()


def require_case(condition: bool, name: str, result: subprocess.CompletedProcess) -> None:
    if condition:
        print(f"[OK] {name}")
        return
    raise RuntimeError(f"{name} failed\nstdout:\n{result.stdout}\nstderr:\n{result.stderr}")


def main() -> int:
    binary = find_binary()
    if binary is None:
        print("MAVSDK Phase A smoke binary is missing")
        return 2

    valid = run_peer_case(binary, peer_id=1, expected_id=1, command="status")
    require_case(valid.returncode == 0 and has_required_output("status", valid.stdout, "1"), "valid peer", valid)

    wrong = run_peer_case(binary, peer_id=2, expected_id=1, command="connect")
    require_case(wrong.returncode != 0 and "different system ID" in wrong.stderr, "wrong peer", wrong)

    no_peer = run_binary(binary, "connect", find_free_udp_port(), system_id=1)
    require_case(no_peer.returncode != 0 and "timed out" in no_peer.stderr, "no peer", no_peer)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
