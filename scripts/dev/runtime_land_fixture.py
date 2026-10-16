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


def land_audit_timing(directory: Path) -> dict:
    """Project only fixed LAND event, ordinal and outcome facts from the durable journal."""
    records, status = [], "missing"
    try:
        for path in directory.glob("*.jsonl"):
            status = "read" if status == "missing" else status
            for line in path.read_bytes().splitlines():
                try:
                    record = json.loads(line, object_pairs_hook=unique_native_fields)
                except (ValueError, RecursionError):
                    status = "partial"
                    continue
                if not isinstance(record, dict) or record.get("operation") != "land":
                    continue
                event, result, ack = record.get("event"), record.get("result"), record.get("acknowledged")
                ordinal = record.get("ordinal")
                if (
                    event not in ("mutation_intent", "mutation_outcome", "request_rejected")
                    or result not in ("pending", "success", "failed", "unknown", "interrupted", "rejected")
                    or type(ordinal) is not int
                    or not 1 <= ordinal <= 2**63 - 1
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
                    }
                )
    except OSError:
        status = "unavailable"
    return {"read_status": status, "records": records}


def emit_land_timing(name: str, timing: dict, passed: bool) -> None:
    """Retain raw and accounted latency even when a case fails before cleanup."""
    started, returned = timing.get("started_ns"), timing.get("returned_ns")
    print(
        json.dumps(
            {
                "event": "runtime_land_budget",
                "case": name if name in LAND_CASES else "unrecognized",
                "checks": "passed" if passed else "failed",
                "elapsed_us": (returned - started) // 1000 if type(started) is int and type(returned) is int else None,
                "native_audit_us": timing.get("native_audit_us"),
                "non_native_elapsed_us": timing.get("non_native_elapsed_us"),
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


def exercise_cli(port: int, peer: LandPeer, timing: dict) -> tuple[str, bool, float]:
    environment = dict(
        os.environ,
        NOMAD_RUNTIME_IPC_PORT=str(port),
        NOMAD_CLIENT_ID="nomad-cli",
        NOMAD_CLIENT_CREDENTIAL=FIXTURE_CREDENTIALS["nomad-cli"],
    )
    admitted = subprocess.run([str(find_cli()), "admit"], env=environment, capture_output=True, text=True, timeout=8)
    require(admitted.returncode == 0, "installed CLI explicitly admits its own source")
    timing["started_ns"] = time.monotonic_ns()
    result = subprocess.run([str(find_cli()), "land"], env=environment, capture_output=True, text=True, timeout=8)
    timing["returned_ns"] = time.monotonic_ns()
    elapsed = (timing["returned_ns"] - timing["started_ns"]) / 1_000_000_000
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
    timing["started_ns"] = time.monotonic_ns()
    response = send_request(port, mutation)
    timing["returned_ns"] = time.monotonic_ns()
    elapsed = (timing["returned_ns"] - timing["started_ns"]) / 1_000_000_000
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
        emit_land_timing(name, timing, passed)
    except Exception as error:
        if case_error is None:
            case_error = error
        try:
            print(
                '{"event":"runtime_land_budget","checks":"failed","measurement":"unavailable"}',
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
