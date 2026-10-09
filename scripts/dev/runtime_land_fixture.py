# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Qualify bounded runtime LAND against a controllable one-hertz MAVLink peer."""

from __future__ import annotations

import json
import os
import socket
import subprocess
import sys
import time
from pathlib import Path

from mavsdk_peer import ACCEPTED, COMMAND_NAV_LAND, DENIED, MODE_LAND, VehiclePeer, mavlink
from runtime_fixture_support import (
    FIXTURE_CREDENTIALS,
    authority_fields,
    find_cli,
    find_runtime,
    free_port,
    read_logs,
    request,
    require,
    send_request,
    start_runtime,
    stop_runtime,
    wait_for_listener,
)
from runtime_qualification_support import wait_for_vehicle


class LandPeer(VehiclePeer):
    """Keep aircraft airborne while controlling ACK, mode and heartbeat timing."""

    def __init__(
        self,
        port: int,
        *,
        ack_delay: float = 0,
        mode_delay: float | None = 0,
        ack_result: int | None = ACCEPTED,
        change_identity: bool = False,
    ) -> None:
        super().__init__(port, 1, ACCEPTED, initial_mode=4, initial_armed=True, initial_relative_altitude_m=5)
        self._land_ack_result = ack_result
        self.ack_delay, self.mode_delay, self.change_identity = ack_delay, mode_delay, change_identity
        self.first_land_at: float | None = None
        self.land_ack_at: float | None = None
        self._mode_at: float | None = None
        self._last_heartbeat_at = 0.0
        self.heartbeats: list[tuple[float, int]] = []

    def _apply(self, kind: str, command: int, message) -> None:
        if command != COMMAND_NAV_LAND:
            super()._apply(kind, command, message)
            return
        if self.first_land_at is None:
            self.first_land_at = time.monotonic()

    def _acknowledge(self, command: int) -> None:
        if command != COMMAND_NAV_LAND:
            super()._acknowledge(command)

    def _send_telemetry(self) -> None:
        now = time.monotonic()
        if self.first_land_at is not None:
            if self.change_identity:
                self._vehicle_type = mavlink.MAV_TYPE_FIXED_WING
            if (
                self.land_ack_at is None
                and self._land_ack_result is not None
                and now >= self.first_land_at + self.ack_delay
            ):
                self._send(self._mavlink.command_ack_encode(COMMAND_NAV_LAND, self._land_ack_result))
                self.land_ack_at = now
                if self._land_ack_result == ACCEPTED and self.mode_delay is not None:
                    self._mode_at = now + self.mode_delay
            if self._mode_at is not None and now >= self._mode_at:
                self._custom_mode = MODE_LAND
        super()._send_telemetry()

    def _send(self, message) -> None:
        if message.get_type() == "HEARTBEAT":
            now = time.monotonic()
            if now - self._last_heartbeat_at < 1.0:
                return
            self._last_heartbeat_at = now
            self.heartbeats.append((now, int(message.custom_mode)))
        super()._send(message)

    def land_count(self) -> int:
        return sum(command[1] == COMMAND_NAV_LAND for command in self.commands())


def bound_land(port: int, identifier: str) -> dict:
    hello = request(port, identifier + "-hello", "hello")
    return {
        "protocol": "nomad-core",
        "version": 1,
        "client_id": "runtime-smoke",
        "id": identifier,
        "type": "land",
        "credential": FIXTURE_CREDENTIALS["runtime-smoke"],
        **authority_fields(hello, "runtime-smoke"),
    }


def audit_land(directory: Path) -> dict:
    records = [json.loads(line) for path in directory.glob("*.jsonl") for line in path.read_bytes().splitlines()]
    outcomes = [
        record for record in records if record.get("operation") == "land" and record["event"] == "mutation_outcome"
    ]
    require(len(outcomes) == 1, "one LAND operation has exactly one durable outcome")
    return outcomes[0]


def exercise_cli(port: int, peer: LandPeer) -> tuple[str, bool, float]:
    environment = dict(
        os.environ,
        NOMAD_RUNTIME_IPC_PORT=str(port),
        NOMAD_CLIENT_ID="nomad-cli",
        NOMAD_CLIENT_CREDENTIAL=FIXTURE_CREDENTIALS["nomad-cli"],
    )
    admitted = subprocess.run([str(find_cli()), "admit"], env=environment, capture_output=True, text=True, timeout=8)
    require(admitted.returncode == 0, "installed CLI explicitly admits its own source")
    started = time.monotonic()
    result = subprocess.run([str(find_cli()), "land"], env=environment, capture_output=True, text=True, timeout=8)
    elapsed = time.monotonic() - started
    require(result.returncode == 0, f"installed CLI LAND succeeds: {result.stderr}")
    require(
        result.stdout.strip() == "LAND mode observed; touchdown not verified",
        "CLI distinguishes engagement from touchdown",
    )
    require(peer._armed and peer._relative_altitude_m == 5, "peer remains airborne after engagement success")
    return "success", True, elapsed


def exercise_request(
    port: int, peer: LandPeer, name: str, expected: str, acknowledged: bool
) -> tuple[str, bool, float]:
    hello = request(port, name + "-admit-context", "hello")
    admitted = request(port, name + "-admit", "admit_authority", **authority_fields(hello, "runtime-smoke"))
    require(admitted["ok"], "runtime LAND source requires explicit authenticated admission")
    mutation = bound_land(port, name)
    started = time.monotonic()
    response = send_request(port, mutation)
    elapsed = time.monotonic() - started
    require(response["outcome"] == expected, f"{name}: truthful {expected} outcome")
    require(response["command_result"]["acknowledged"] == acknowledged, f"{name}: independent ACK evidence retained")
    count = peer.land_count()
    require(send_request(port, mutation) == response, f"{name}: cached result remains identical")
    time.sleep(0.3)
    require(peer.land_count() == count, f"{name}: cached request never starts another LAND transmission")
    return response["outcome"], response["command_result"]["acknowledged"], elapsed


def run_case(name: str, expected: str, acknowledged: bool, *, cli: bool = False, **options) -> dict:
    udp, ipc = free_port(socket.SOCK_DGRAM), free_port(socket.SOCK_STREAM)
    peer = LandPeer(udp, **options)
    peer.start()
    process, stdout, stderr = start_runtime(find_runtime(), udp, ipc)
    try:
        wait_for_listener(ipc)
        wait_for_vehicle(ipc)
        outcome, ack, elapsed = (
            exercise_cli(ipc, peer) if cli else exercise_request(ipc, peer, name, expected, acknowledged)
        )
        record = audit_land(Path(process._nomad_test_storage.name) / "audit")
        require(
            record["result"] == outcome and record["acknowledged"] == ack, f"{name}: durable evidence matches response"
        )
        require(elapsed <= 3.5, f"{name}: three-second operation plus bounded IPC/process margin ({elapsed:.3f}s)")
        if expected == "unknown" and ack:
            require(elapsed >= 2.85, f"{name}: missing engagement consumes the single observation budget")
        if expected == "success":
            require(
                peer.land_ack_at is not None
                and any(at > peer.land_ack_at and mode == MODE_LAND for at, mode in peer.heartbeats),
                f"{name}: real post-ACK LAND heartbeat required",
            )
        return {
            "case": name,
            "outcome": outcome,
            "acknowledged": ack,
            "elapsed_ms": round(elapsed * 1000, 3),
            "wire_attempts": peer.land_count(),
            "heartbeat_period_seconds": 1,
        }
    except Exception:
        print(read_logs(stdout, stderr), file=sys.stderr)
        raise
    finally:
        stop_runtime(process)
        stdout.close()
        stderr.close()
        process._nomad_test_storage.cleanup()
        peer.stop()


def main() -> int:
    cases = [
        run_case("installed-cli-one-hertz", "success", True, cli=True),
        run_case("late-ack-within-budget", "success", True, ack_delay=1.4),
        run_case("late-mode-outside-budget", "unknown", True, ack_delay=1.8, mode_delay=1.8),
        run_case("accepted-without-mode", "unknown", True, mode_delay=None),
        run_case("negative-ack", "failed", True, ack_result=DENIED),
        run_case("missing-ack", "unknown", False, ack_result=None),
        run_case("identity-change-stops-retries", "interrupted", False, ack_result=None, change_identity=True),
    ]
    print(json.dumps({"result": "passed", "scope": "runtime-land-one-hertz-peer", "cases": cases}, sort_keys=True))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as error:
        print(f"runtime LAND qualification failed: {error}", file=sys.stderr)
        raise SystemExit(1)
