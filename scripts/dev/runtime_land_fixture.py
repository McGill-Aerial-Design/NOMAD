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
    REQUEST_TIMING_FIELDS,
    authority_fields,
    find_cli,
    find_runtime,
    free_port,
    request,
    require,
    send_request,
    start_runtime,
    stop_runtime,
    wait_for_listener,
)
from runtime_qualification_support import admit, wait_for_vehicle

LAND_CASES = (
    "installed-cli-one-hertz",
    "late-ack-within-budget",
    "late-mode-outside-budget",
    "accepted-without-mode",
    "negative-ack",
    "missing-ack",
    "identity-change-stops-retries",
)


def clock_anchor() -> dict:
    before = time.monotonic_ns()
    wall = time.time_ns() // 1_000_000
    return {"before_ns": before, "unix_ms": wall, "after_ns": time.monotonic_ns()}


def clock_correlation(timing: dict) -> str:
    start, end = timing.get("wall_start"), timing.get("wall_end")
    if not valid_clock_anchor(start) or not valid_clock_anchor(end):
        return "unavailable"
    wall_delta = end["unix_ms"] - start["unix_ms"]
    lower = (end["before_ns"] - start["after_ns"]) / 1_000_000
    upper = (end["after_ns"] - start["before_ns"]) / 1_000_000
    # Integer wall milliseconds can differ by one tick at each sample.
    return "consistent_at_ms_resolution" if lower - 2 <= wall_delta <= upper + 2 else "clock_shift_or_sampling_mismatch"


def valid_clock_anchor(value) -> bool:
    return (
        isinstance(value, dict)
        and set(value) == {"before_ns", "unix_ms", "after_ns"}
        and all(type(item) is int and 0 <= item <= 2**63 - 1 for item in value.values())
        and value["before_ns"] <= value["after_ns"]
    )


def land_audit_timing(directory: Path) -> dict:
    """Project synthetic LAND records; their wall times precede the native write."""
    records, status = [], "missing"
    try:
        for path in directory.glob("*.jsonl"):
            status = "read" if status == "missing" else status
            for line in path.read_bytes().splitlines():
                try:
                    record = json.loads(line)
                except (ValueError, RecursionError):
                    status = "partial"
                    continue
                if not isinstance(record, dict) or record.get("operation") != "land":
                    continue
                event, result, ack = record.get("event"), record.get("result"), record.get("acknowledged")
                ordinal, wall = record.get("ordinal"), record.get("time_ms")
                if (
                    event not in ("mutation_intent", "mutation_outcome", "request_rejected")
                    or result not in ("pending", "success", "failed", "unknown", "interrupted", "rejected")
                    or type(ordinal) is not int
                    or not 1 <= ordinal <= 2**63 - 1
                    or type(wall) is not int
                    or not 0 <= wall <= 2**63 - 1
                    or (ack is not None and type(ack) is not bool)
                    or len(records) >= 8
                ):
                    status = "partial"
                    continue
                records.append(
                    {
                        "event": event,
                        "result": result,
                        "acknowledged": ack,
                        "ordinal": ordinal,
                        "prewrite_unix_ms": wall,
                    }
                )
    except OSError:
        status = "unavailable"
    return {"read_status": status, "records": records}


def emit_land_timing(name: str, peer: LandPeer, directory: Path, timing: dict, passed: bool) -> None:
    """Emit fixed synthetic facts before teardown without exporting requests or raw journals."""
    origin = timing.get("started_ns")

    def offset(value):
        return (value - origin) // 1000 if type(value) is int and type(origin) is int else None

    def peer_offset(value):
        return offset(int(value * 1_000_000_000)) if value is not None else None

    def anchor(value):
        if not valid_clock_anchor(value):
            return None
        return {
            "before_us": offset(value["before_ns"]),
            "unix_ms": value["unix_ms"],
            "after_us": offset(value["after_ns"]),
        }

    after_land = [
        (at, mode) for at, mode in peer.heartbeats if peer.first_land_at is not None and at > peer.first_land_at
    ]
    after_ack = [(at, mode) for at, mode in after_land if peer.land_ack_at is not None and at > peer.land_ack_at]
    audit = land_audit_timing(directory)
    client = {key.removesuffix("_ns"): offset(timing.get("client", {}).get(key)) for key in REQUEST_TIMING_FIELDS}
    observed = all(value is not None for key, value in client.items() if key != "error")
    observed &= (
        audit["read_status"] == "read"
        and bool(audit["records"])
        and clock_correlation(timing) == "consistent_at_ms_resolution"
    )
    print(
        json.dumps(
            {
                "event": "runtime_land_stage_timing",
                "case": name if name in LAND_CASES else "unrecognized",
                "checks": "passed" if passed else "failed",
                "elapsed_us": offset(timing.get("returned_ns")),
                "measurement": "observed_boundaries" if observed else "partial",
                "client_us": client,
                "clock_anchors": {key: anchor(timing.get(key)) for key in ("wall_start", "wall_end")},
                "clock_correlation": clock_correlation(timing),
                "peer_us": {
                    "first_land_decoded": peer_offset(peer.first_land_at),
                    "ack_loop_entry": peer_offset(peer.land_ack_at),
                    "ack_send_begin": offset(peer.land_ack_send_begin_ns),
                    "ack_send_return": offset(peer.land_ack_send_return_ns),
                    "first_heartbeat_attempt_after_land": peer_offset(after_land[0][0]) if after_land else None,
                    "first_heartbeat_attempt_after_ack": peer_offset(after_ack[0][0]) if after_ack else None,
                    "first_land_mode_heartbeat_attempt_after_ack": next(
                        (peer_offset(at) for at, mode in after_ack if mode == MODE_LAND), None
                    ),
                },
                "wire_attempts": peer.land_count(),
                "audit": audit,
                "audit_read_us": offset(time.monotonic_ns()),
                "limits": "Python monotonic offsets; journal wall time is pre-write, not durable completion; peer send attempts are not receipt; peer snapshot may include post-response heartbeats",
            },
            sort_keys=True,
            allow_nan=False,
        ),
        flush=True,
    )


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
        self.land_ack_send_begin_ns: int | None = None
        self.land_ack_send_return_ns: int | None = None
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
                message = self._mavlink.command_ack_encode(COMMAND_NAV_LAND, self._land_ack_result)
                self.land_ack_send_begin_ns = time.monotonic_ns()
                self._send(message)
                self.land_ack_send_return_ns = time.monotonic_ns()
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


def exercise_cli(port: int, peer: LandPeer, timing: dict) -> tuple[str, bool, float]:
    environment = dict(
        os.environ,
        NOMAD_RUNTIME_IPC_PORT=str(port),
        NOMAD_CLIENT_ID="nomad-cli",
        NOMAD_CLIENT_CREDENTIAL=FIXTURE_CREDENTIALS["nomad-cli"],
    )
    admitted = subprocess.run([str(find_cli()), "admit"], env=environment, capture_output=True, text=True, timeout=8)
    require(admitted.returncode == 0, "installed CLI explicitly admits its own source")
    timing["wall_start"] = clock_anchor()
    started = time.monotonic()
    timing["started_ns"] = int(started * 1_000_000_000)
    try:
        result = subprocess.run([str(find_cli()), "land"], env=environment, capture_output=True, text=True, timeout=8)
    finally:
        returned = time.monotonic()
        timing["returned_ns"], timing["wall_end"] = int(returned * 1_000_000_000), clock_anchor()
    elapsed = returned - started
    require(result.returncode == 0, "installed CLI LAND succeeds")
    require(
        result.stdout.strip() == "LAND mode observed; touchdown not verified",
        "CLI distinguishes engagement from touchdown",
    )
    require(peer._armed and peer._relative_altitude_m == 5, "peer remains airborne after engagement success")
    return "success", True, elapsed


def exercise_request(
    port: int, peer: LandPeer, name: str, expected: str, acknowledged: bool, timing: dict
) -> tuple[str, bool, float]:
    admit(port)
    mutation = bound_land(port, name)
    timing["wall_start"] = clock_anchor()
    timing["client"] = {}
    started = time.monotonic()
    timing["started_ns"] = int(started * 1_000_000_000)
    try:
        response = send_request(port, mutation, timing=timing["client"])
    finally:
        returned = time.monotonic()
        timing["returned_ns"], timing["wall_end"] = int(returned * 1_000_000_000), clock_anchor()
    elapsed = returned - started
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
    timing, passed = {}, False
    try:
        wait_for_listener(ipc)
        wait_for_vehicle(ipc)
        outcome, ack, elapsed = (
            exercise_cli(ipc, peer, timing)
            if cli
            else exercise_request(ipc, peer, name, expected, acknowledged, timing)
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
        passed = True
        return {
            "case": name,
            "outcome": outcome,
            "acknowledged": ack,
            "elapsed_ms": round(elapsed * 1000, 3),
            "wire_attempts": peer.land_count(),
            "heartbeat_period_seconds": 1,
        }
    finally:
        try:
            emit_land_timing(name, peer, Path(process._nomad_test_storage.name) / "audit", timing, passed)
        except Exception:
            # A diagnostic failure must never replace the case result or prevent owned cleanup.
            try:
                print('{"event":"runtime_land_stage_timing","measurement":"unavailable"}', file=sys.stderr, flush=True)
            except Exception:
                pass
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
