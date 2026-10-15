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


def project_native_timing(record: object) -> dict | None:
    if not isinstance(record, dict) or set(record) != {"event", "capacity", "overflow", "records"}:
        return None
    if (
        record["event"] != "runtime_land_native_timing"
        or type(record["capacity"]) is not int
        or record["capacity"] != 8
    ):
        return None
    if type(record["overflow"]) is not bool or not isinstance(record["records"], list) or len(record["records"]) > 8:
        return None
    rows = []
    keys = {
        "journal_event",
        "ordinal",
        "lock_acquired_us",
        "before_native_us",
        "after_native_us",
        "append_finish_us",
        "native_success",
        "append_success",
    }
    for row in record["records"]:
        if not isinstance(row, dict) or set(row) != keys:
            return None
        if row["journal_event"] not in ("mutation_intent", "mutation_outcome", "request_rejected"):
            return None
        for key in ("ordinal", "lock_acquired_us", "before_native_us", "after_native_us", "append_finish_us"):
            value = row[key]
            if value is None and key in ("before_native_us", "after_native_us"):
                continue
            if type(value) is not int or not (1 if key == "ordinal" else 0) <= value <= 2**63 - 1:
                return None
        before, after = row["before_native_us"], row["after_native_us"]
        if type(row["append_success"]) is not bool or (
            row["native_success"] is not None and type(row["native_success"]) is not bool
        ):
            return None
        if row["lock_acquired_us"] > row["append_finish_us"]:
            return None
        if before is not None and not row["lock_acquired_us"] <= before <= row["append_finish_us"]:
            return None
        if after is not None and (before is None or not before <= after <= row["append_finish_us"]):
            return None
        if (after is None) != (row["native_success"] is None):
            return None
        if row["append_success"] and row["native_success"] is not True:
            return None
        if rows and row["ordinal"] <= rows[-1]["ordinal"]:
            return None
        rows.append({key: row[key] for key in keys})
    return {"capacity": 8, "overflow": record["overflow"], "records": rows}


def unique_native_fields(pairs):
    record = {}
    for key, value in pairs:
        if key in record:
            raise ValueError("duplicate native diagnostic field")
        record[key] = value
    return record


def read_native_timing(stderr, exit_code: int | None, expected: list[dict] | None = None) -> dict:
    stderr.seek(0)
    candidate, count, rejected = None, 0, 0
    for line in stderr:
        if not line.lstrip().startswith(b"{"):
            continue
        try:
            record = json.loads(line, object_pairs_hook=unique_native_fields) if len(line) <= 8192 else None
        except (ValueError, RecursionError):
            record = None
        if not isinstance(record, dict) or record.get("event") != "runtime_land_native_timing":
            rejected += int(b'"runtime_land_native_timing"' in line)
            continue
        count += 1
        candidate = project_native_timing(record)
        rejected += int(candidate is None)
    if count != 1 or candidate is None:
        return {
            "status": "partial" if count or rejected else "unavailable",
            "records": [],
            "overflow": None,
            "candidate_count": count,
            "rejected_candidates": rejected,
        }
    matches = expected is not None and [(row["ordinal"], row["journal_event"]) for row in candidate["records"]] == [
        (row["ordinal"], row["event"]) for row in expected
    ]
    complete = (
        type(exit_code) is int
        and exit_code == 0
        and rejected == 0
        and not candidate["overflow"]
        and [row["journal_event"] for row in candidate["records"]] == ["mutation_intent", "mutation_outcome"]
        and all(row["append_success"] and row["after_native_us"] is not None for row in candidate["records"])
        and matches
    )
    return {
        "status": "complete" if complete else "partial",
        **candidate,
        "candidate_count": count,
        "rejected_candidates": rejected,
        "journal_match": matches,
    }


def emit_native_timing(name: str, stderr, exit_code: int | None, directory: Path) -> tuple[dict, dict]:
    audit = land_audit_timing(directory)
    facts = read_native_timing(stderr, exit_code, audit["records"] if audit["read_status"] == "read" else None)
    print(
        json.dumps(
            {
                "event": "runtime_land_native_stage_timing",
                "case": name if name in LAND_CASES else "unrecognized",
                "runtime_exit_code": exit_code,
                **facts,
                "limits": "Steady offsets from append entry; native call includes write/flush and descheduling; append_finish is an in-function pre-unlock snapshot; no shared Python epoch",
            },
            sort_keys=True,
            allow_nan=False,
        ),
        flush=True,
    )
    return facts, audit


def land_budget_accounting(timing: dict, native: dict, audit: dict) -> dict:
    """Remove only the matched synchronous native calls contained in the first response."""
    started, returned = timing.get("started_ns"), timing.get("returned_ns")
    require(
        timing.get("first_response_complete") is True
        and type(started) is int
        and type(returned) is int
        and 0 <= started <= returned,
        "LAND budget has a completed first-response interval",
    )
    require(isinstance(native, dict) and isinstance(audit, dict), "LAND budget has native and durable facts")
    projected = project_native_timing(
        {
            "event": "runtime_land_native_timing",
            "capacity": native.get("capacity"),
            "overflow": native.get("overflow"),
            "records": native.get("records"),
        }
    )
    require(
        projected is not None
        and native.get("status") == "complete"
        and type(native.get("candidate_count")) is int
        and native["candidate_count"] == 1
        and type(native.get("rejected_candidates")) is int
        and native["rejected_candidates"] == 0
        and native.get("journal_match") is True
        and projected["overflow"] is False
        and audit.get("read_status") == "read",
        "LAND budget requires complete native measurement and durable association",
    )
    rows, durable = projected["records"], audit["records"]
    require(
        [row["journal_event"] for row in rows] == ["mutation_intent", "mutation_outcome"]
        and [(row["ordinal"], row["journal_event"]) for row in rows]
        == [(row["ordinal"], row["event"]) for row in durable]
        and all(
            row["append_success"] is True
            and row["native_success"] is True
            and row["before_native_us"] is not None
            and row["after_native_us"] is not None
            for row in rows
        ),
        "LAND budget uses exactly the successful first-request intent and outcome",
    )
    # Intent precedes Vehicle's deadline; outcome is synchronous before the first response.
    # Cached replay adds neither LAND entry, so no later request interval is subtracted.
    wall_us = (returned - started) // 1000
    native_us = sum(row["after_native_us"] - row["before_native_us"] for row in rows)
    require(
        native_us <= wall_us and sum(row["append_finish_us"] for row in rows) <= wall_us,
        "LAND native intervals fit inside the first-response wall interval",
    )
    return {"native_audit_us": native_us, "non_native_elapsed_us": wall_us - native_us}


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
    """Emit final fixed facts before storage cleanup without exporting requests or raw journals."""
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
                "native_audit_us": timing.get("native_audit_us"),
                "non_native_elapsed_us": timing.get("non_native_elapsed_us"),
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
    timing["first_response_complete"] = True
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
    timing["first_response_complete"] = True
    count = peer.land_count()
    require(send_request(port, mutation) == response, f"{name}: cached result remains identical")
    time.sleep(0.3)
    require(peer.land_count() == count, f"{name}: cached request never starts another LAND transmission")
    return response["outcome"], response["command_result"]["acknowledged"], elapsed


def run_case(name: str, expected: str, acknowledged: bool, *, cli: bool = False, **options) -> dict:
    udp, ipc = free_port(socket.SOCK_DGRAM), free_port(socket.SOCK_STREAM)
    peer = LandPeer(udp, **options)
    peer.start()
    process, stdout, stderr = start_runtime(find_runtime(), udp, ipc, audit_timing=True)
    directory = Path(process._nomad_test_storage.name) / "audit"
    timing, passed, result, case_error = {}, False, None, None
    try:
        wait_for_listener(ipc)
        wait_for_vehicle(ipc)
        outcome, ack, elapsed = (
            exercise_cli(ipc, peer, timing)
            if cli
            else exercise_request(ipc, peer, name, expected, acknowledged, timing)
        )
        record = audit_land(directory)
        require(
            record["result"] == outcome and record["acknowledged"] == ack, f"{name}: durable evidence matches response"
        )
        if expected == "success":
            require(
                peer.land_ack_at is not None
                and any(at > peer.land_ack_at and mode == MODE_LAND for at, mode in peer.heartbeats),
                f"{name}: real post-ACK LAND heartbeat required",
            )
        result = {
            "case": name,
            "outcome": outcome,
            "acknowledged": ack,
            "elapsed_ms": round(elapsed * 1000, 3),
            "wire_attempts": peer.land_count(),
            "heartbeat_period_seconds": 1,
        }
    except BaseException as error:
        case_error = error
    try:
        stop_runtime(process)
    except Exception as error:
        if case_error is None:
            case_error = error
    exited = process.poll() is not None
    try:
        if not exited:
            raise AssertionError("LAND native measurement requires confirmed runtime exit")
        native, audit = emit_native_timing(name, stderr, process.returncode, directory)
    except Exception as error:
        if case_error is None:
            case_error = error
        try:
            print('{"event":"runtime_land_native_stage_timing","status":"unavailable"}', file=sys.stderr, flush=True)
        except Exception:
            pass
    if case_error is None:
        try:
            timing.update(land_budget_accounting(timing, native, audit))
            residual_us = timing["non_native_elapsed_us"]
            require(
                residual_us <= 3_500_000,
                f"{name}: three-second operation plus bounded IPC/process margin ({residual_us / 1_000_000:.3f}s non-native)",
            )
            if expected == "unknown" and result["acknowledged"]:
                require(residual_us >= 2_850_000, f"{name}: missing engagement consumes the single observation budget")
            result.update(native_audit_ms=timing["native_audit_us"] / 1000, non_native_elapsed_ms=residual_us / 1000)
            passed = True
        except Exception as error:
            case_error = error
    try:
        emit_land_timing(name, peer, directory, timing, passed)
    except Exception as error:
        if case_error is None:
            case_error = error
        try:
            print(
                '{"event":"runtime_land_stage_timing","checks":"failed","measurement":"unavailable"}',
                file=sys.stderr,
                flush=True,
            )
        except Exception:
            pass
    for cleanup in (stdout.close, stderr.close, process._nomad_test_storage.cleanup if exited else None, peer.stop):
        if cleanup is None:
            continue
        try:
            cleanup()
        except Exception as error:
            if case_error is None:
                case_error = error
    if case_error is not None:
        raise case_error
    return result


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
    except Exception:
        print("runtime LAND qualification failed", file=sys.stderr)
        raise SystemExit(1)
