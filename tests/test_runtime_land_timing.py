# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Qualify LAND timing observations without changing request, outcome or deadline policy."""

import importlib
import io
import json
import socket
import threading
import time
from contextlib import contextmanager
from pathlib import Path
from types import SimpleNamespace

import pytest


@pytest.fixture
def modules(monkeypatch):
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[1] / "scripts/dev"))
    return importlib.import_module("runtime_fixture_support"), importlib.import_module("runtime_land_fixture")


@contextmanager
def response_server(handler):
    errors = []
    with socket.socket() as listener:
        listener.bind(("127.0.0.1", 0))
        listener.listen()
        listener.settimeout(2)

        def serve():
            try:
                with listener.accept()[0] as connection:
                    connection.settimeout(2)
                    request = bytearray()
                    while not request.endswith(b"\n"):
                        chunk = connection.recv(1024)
                        if not chunk:
                            raise AssertionError("test client did not complete its request")
                        request.extend(chunk)
                    handler(connection, json.loads(request))
            except Exception as error:
                errors.append(error)

        worker = threading.Thread(target=serve, daemon=True)
        worker.start()
        try:
            yield listener.getsockname()[1]
        finally:
            worker.join(timeout=3)
            assert not worker.is_alive(), "owned response server must stop"
            assert not errors, errors


def test_actual_socket_records_preparation_connect_send_and_fragmented_response(modules):
    support, _ = modules
    timing, received = {}, []

    def respond(connection, request):
        received.append(request)
        connection.sendall(b'{"ok":')
        deadline = time.monotonic() + 1
        while timing["first_byte_ns"] is None and time.monotonic() < deadline:
            time.sleep(0.001)
        assert timing["first_byte_ns"] is not None
        assert timing["read_complete_ns"] is None
        connection.sendall(b"true}\n")

    message = {"id": "synthetic", "credential": "synthetic-private-value"}
    with response_server(respond) as port:
        assert support.send_request(port, message, timing=timing) == {"ok": True}
    ordered = [timing[key] for key in support.REQUEST_TIMING_FIELDS if key != "error_ns"]
    assert all(type(value) is int for value in ordered) and ordered == sorted(ordered)
    assert timing["error_ns"] is None
    assert "credential" not in received[0] and "auth_proof" in received[0]
    assert message == {"id": "synthetic", "credential": "synthetic-private-value"}
    assert "synthetic-private-value" not in json.dumps(timing)


@pytest.mark.parametrize(
    ("payload", "exception", "complete"),
    [
        (b"", ConnectionError, False),
        (b"bad\n", ValueError, True),
        (b"[]\n", ValueError, True),
        (b'{"ok":true}', ConnectionError, False),
    ],
)
def test_actual_socket_failures_preserve_observed_and_absent_boundaries(modules, payload, exception, complete):
    support, _ = modules
    timing = {"stale": "must be cleared"}
    with response_server(lambda connection, _: connection.sendall(payload)) as port:
        with pytest.raises(exception):
            support.send_request(port, {}, timing=timing)
    assert set(timing) == set(support.REQUEST_TIMING_FIELDS)
    assert timing["error_ns"] is not None and timing["finished_ns"] >= timing["error_ns"]
    assert (timing["read_complete_ns"] is not None) is complete
    assert (timing["first_byte_ns"] is not None) is bool(payload)
    assert timing["response_ready_ns"] is None


def test_oversize_request_stays_rejected_before_connect(modules, monkeypatch):
    support, _ = modules
    timing = {}
    monkeypatch.setattr(support.socket, "create_connection", lambda *_a, **_kw: pytest.fail("must not connect"))
    with pytest.raises(ValueError, match="request exceeds protocol limit"):
        support.send_request(0, {"value": "x" * 65537}, timing=timing)
    assert timing["prepare_complete_ns"] is not None and timing["connect_begin_ns"] is None
    assert timing["error_ns"] is not None


def test_actual_socket_oversize_response_and_refused_connection_retain_boundaries(modules):
    support, _ = modules
    timing = {}
    with response_server(lambda connection, _: connection.sendall(b"x" * 65537)) as port:
        with pytest.raises(ValueError, match="response exceeded the protocol limit"):
            support.send_request(port, {}, timing=timing)
    assert timing["first_byte_ns"] is not None and timing["read_complete_ns"] is None
    with socket.socket() as reserved:
        reserved.bind(("127.0.0.1", 0))
        with pytest.raises(OSError):
            support.send_request(reserved.getsockname()[1], {}, timing=timing)
    assert timing["connect_begin_ns"] is not None and timing["connect_complete_ns"] is None
    assert timing["send_begin_ns"] is None and timing["first_byte_ns"] is None and timing["error_ns"] is not None


def test_client_connection_and_read_budgets_are_preserved(modules, monkeypatch):
    support, _ = modules
    budgets = []

    class Connection:
        def __enter__(self):
            return self

        def __exit__(self, *_):
            return None

        def settimeout(self, value):
            budgets.append(("read", value))

        def sendall(self, _):
            return None

        def recv(self, size):
            assert size == 1
            raise TimeoutError("synthetic read deadline")

    def connect(_address, *, timeout):
        budgets.append(("connect", timeout))
        return Connection()

    monkeypatch.setattr(support.socket, "create_connection", connect)
    timing = {}
    with pytest.raises(TimeoutError, match="synthetic read deadline"):
        support.send_request(0, {}, timing=timing)
    assert budgets == [("connect", 2.0), ("read", 8.0)]
    assert timing["first_byte_ns"] is None and timing["read_complete_ns"] is None


def audit_record(**changes):
    return dict(
        operation="land",
        event="mutation_outcome",
        result="unknown",
        acknowledged=True,
        ordinal=12,
        time_ms=1000,
        credential="PRIVATE_SENTINEL",
        request_id="PRIVATE_SENTINEL",
        **changes,
    )


def test_audit_projection_never_exports_raw_unknown_or_identifying_fields(modules, tmp_path):
    _, land = modules
    record = audit_record()
    record.update(auth_proof="PRIVATE_SENTINEL", path="PRIVATE_SENTINEL", runtime_incarnation="PRIVATE_SENTINEL")
    (tmp_path / "synthetic.jsonl").write_text(json.dumps(record) + "\n", encoding="utf-8")
    result = land.land_audit_timing(tmp_path)
    assert result == {
        "read_status": "read",
        "records": [
            {
                "event": "mutation_outcome",
                "result": "unknown",
                "acknowledged": True,
                "ordinal": 12,
                "prewrite_unix_ms": 1000,
            }
        ],
    }
    assert "PRIVATE_SENTINEL" not in json.dumps(result)


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("event", "PRIVATE_SENTINEL"),
        ("result", "PRIVATE_SENTINEL"),
        ("ordinal", True),
        ("time_ms", "PRIVATE_SENTINEL"),
        ("acknowledged", "PRIVATE_SENTINEL"),
    ],
)
def test_invalid_audit_values_remain_partial_without_raw_values(modules, tmp_path, field, value):
    _, land = modules
    record = audit_record()
    record[field] = value
    (tmp_path / "synthetic.jsonl").write_text(json.dumps(record) + "\n", encoding="utf-8")
    result = land.land_audit_timing(tmp_path)
    assert result == {"read_status": "partial", "records": []}
    assert "PRIVATE_SENTINEL" not in json.dumps(result)


def test_missing_truncated_or_unreadable_journal_never_claims_observed_records(modules, tmp_path, monkeypatch):
    _, land = modules
    assert land.land_audit_timing(tmp_path / "absent") == {"read_status": "missing", "records": []}
    (tmp_path / "synthetic.jsonl").write_bytes(b'{"operation":"land"')
    assert land.land_audit_timing(tmp_path) == {"read_status": "partial", "records": []}
    monkeypatch.setattr(Path, "read_bytes", lambda _: (_ for _ in ()).throw(PermissionError("PRIVATE_SENTINEL")))
    assert land.land_audit_timing(tmp_path) == {"read_status": "unavailable", "records": []}


def owned_case(monkeypatch, land, tmp_path, *, elapsed=3.0, failure=None, native_us=0):
    events = []
    peer = SimpleNamespace(
        first_land_at=None,
        land_ack_at=None,
        land_ack_send_begin_ns=None,
        land_ack_send_return_ns=None,
        heartbeats=[],
        land_count=lambda: 1,
        start=lambda: events.append("peer_start"),
        stop=lambda: events.append("peer_stop"),
    )
    storage = SimpleNamespace(name=str(tmp_path), cleanup=lambda: events.append("cleanup"))
    process = SimpleNamespace(_nomad_test_storage=storage, poll=lambda: 0, returncode=0)
    monkeypatch.setattr(land, "LandPeer", lambda *_a, **_kw: peer)
    monkeypatch.setattr(land, "free_port", lambda *_: 0)
    monkeypatch.setattr(land, "find_runtime", lambda: Path("synthetic"))
    monkeypatch.setattr(
        land,
        "start_runtime",
        lambda *_a, **_kw: (process, io.BytesIO(b"PRIVATE_SENTINEL"), io.BytesIO(b"PRIVATE_SENTINEL")),
    )

    def emit_native(*_):
        assert events[-1] == "stop" and "cleanup" not in events
        events.append("native_emit")
        rows = [
            {
                "journal_event": event,
                "ordinal": ordinal,
                "lock_acquired_us": 0,
                "before_native_us": 0,
                "after_native_us": native_us if ordinal == 7 else 0,
                "append_finish_us": native_us if ordinal == 7 else 0,
                "native_success": True,
                "append_success": True,
            }
            for ordinal, event in ((6, "mutation_intent"), (7, "mutation_outcome"))
        ]
        return (
            {
                "status": "complete",
                "capacity": 8,
                "overflow": False,
                "candidate_count": 1,
                "rejected_candidates": 0,
                "journal_match": True,
                "records": rows,
            },
            {
                "read_status": "read",
                "records": [{"ordinal": row["ordinal"], "event": row["journal_event"]} for row in rows],
            },
        )

    monkeypatch.setattr(land, "emit_native_timing", emit_native)
    monkeypatch.setattr(land, "stop_runtime", lambda _: events.append("stop"))
    monkeypatch.setattr(land, "wait_for_listener", lambda _: None)
    monkeypatch.setattr(land, "wait_for_vehicle", lambda _: None)
    monkeypatch.setattr(land, "audit_land", lambda _: {"result": "unknown", "acknowledged": True})

    def exercise(_port, _peer, _name, _expected, _ack, timing):
        timing.update(
            started_ns=1_000_000, returned_ns=1_000_000 + round(elapsed * 1_000_000_000), first_response_complete=True
        )
        if failure is not None:
            raise failure
        return "unknown", True, elapsed

    monkeypatch.setattr(land, "exercise_request", exercise)
    original = land.emit_land_timing

    def emit(*args):
        assert "cleanup" not in events and "stop" in events
        events.append("emit")
        original(*args)

    monkeypatch.setattr(land, "emit_land_timing", emit)
    return events


@pytest.mark.parametrize("elapsed", [3.0, 3.501])
def test_residual_deadline_and_final_success_failure_facts(modules, monkeypatch, tmp_path, capsys, elapsed):
    _, land = modules
    events = owned_case(monkeypatch, land, tmp_path, elapsed=elapsed)
    if elapsed <= 3.5:
        assert land.run_case("accepted-without-mode", "unknown", True)["outcome"] == "unknown"
    else:
        with pytest.raises(AssertionError, match="three-second operation plus bounded IPC/process margin"):
            land.run_case("accepted-without-mode", "unknown", True)
    facts = [json.loads(line) for line in capsys.readouterr().out.splitlines() if line.startswith("{")]
    assert len(facts) == 1 and facts[0]["checks"] == ("passed" if elapsed <= 3.5 else "failed")
    assert all(value is None for value in facts[0]["client_us"].values())
    assert facts[0]["clock_correlation"] == "unavailable"
    assert events[-5:] == ["stop", "native_emit", "emit", "cleanup", "peer_stop"]


@pytest.mark.parametrize("broken_emitter", [False, True])
def test_original_failure_survives_diagnostic_failure_and_all_owned_cleanup(
    modules, monkeypatch, tmp_path, capsys, broken_emitter
):
    _, land = modules
    original = RuntimeError("synthetic original failure")
    events = owned_case(monkeypatch, land, tmp_path, failure=original)
    if broken_emitter:
        monkeypatch.setattr(land, "emit_land_timing", lambda *_: (_ for _ in ()).throw(ValueError("PRIVATE_SENTINEL")))
    with pytest.raises(RuntimeError) as caught:
        land.run_case("accepted-without-mode", "unknown", True)
    assert caught.value is original
    if broken_emitter:
        assert events[-4:] == ["stop", "native_emit", "cleanup", "peer_stop"]
    else:
        assert events[-5:] == ["stop", "native_emit", "emit", "cleanup", "peer_stop"]
    captured = capsys.readouterr()
    assert "PRIVATE_SENTINEL" not in captured.out + captured.err
    if broken_emitter:
        assert json.loads(captured.err)["measurement"] == "unavailable"
    else:
        assert json.loads(captured.out)["checks"] == "failed"


def test_wall_clock_correlation_reports_consistency_shift_and_absence(modules):
    _, land = modules
    start = {"before_ns": 0, "unix_ms": 1000, "after_ns": 1000}
    end = {"before_ns": 3_000_000_000, "unix_ms": 4000, "after_ns": 3_000_001_000}
    assert land.clock_correlation({"wall_start": start, "wall_end": end}) == "consistent_at_ms_resolution"
    end["unix_ms"] += 1000
    assert land.clock_correlation({"wall_start": start, "wall_end": end}) == "clock_shift_or_sampling_mismatch"
    assert land.clock_correlation({}) == "unavailable"


@pytest.mark.parametrize(
    "residual_us,passes", [(2_850_000, True), (3_500_000, True), (2_849_999, False), (3_500_001, False)]
)
def test_both_budget_boundaries_apply_to_same_validated_residual(
    modules, monkeypatch, tmp_path, capsys, residual_us, passes
):
    _, land = modules
    native_us = 2_757_758
    events = owned_case(monkeypatch, land, tmp_path, elapsed=(native_us + residual_us) / 1_000_000, native_us=native_us)
    if passes:
        result = land.run_case("accepted-without-mode", "unknown", True)
        assert result["non_native_elapsed_ms"] == residual_us / 1000
    else:
        with pytest.raises(AssertionError):
            land.run_case("accepted-without-mode", "unknown", True)
    facts = [json.loads(line) for line in capsys.readouterr().out.splitlines() if line.startswith("{")]
    assert len(facts) == 1 and facts[0]["checks"] == ("passed" if passes else "failed")
    assert facts[0]["native_audit_us"] == native_us and facts[0]["non_native_elapsed_us"] == residual_us
    assert events[-5:] == ["stop", "native_emit", "emit", "cleanup", "peer_stop"]


def test_measured_wall_miss_is_accounted_without_shortening_reported_wall(modules, monkeypatch, tmp_path, capsys):
    _, land = modules
    owned_case(monkeypatch, land, tmp_path, elapsed=5.790197, native_us=2_757_758)
    result = land.run_case("accepted-without-mode", "unknown", True)
    assert result["elapsed_ms"] == 5790.197
    assert result["native_audit_ms"] == 2757.758 and result["non_native_elapsed_ms"] == 3032.439
    facts = [json.loads(line) for line in capsys.readouterr().out.splitlines() if line.startswith("{")]
    assert len(facts) == 1 and facts[0]["elapsed_us"] == 5_790_197 and facts[0]["checks"] == "passed"


@pytest.mark.parametrize("failure", [False, True])
def test_native_projection_error_fails_qualification_and_preserves_original_failure(
    modules, monkeypatch, tmp_path, failure
):
    _, land = modules
    original = RuntimeError("synthetic original failure") if failure else None
    events = owned_case(monkeypatch, land, tmp_path, failure=original)

    def broken(*_):
        assert events[-1] == "stop" and "cleanup" not in events
        raise ValueError("PRIVATE_SENTINEL")

    monkeypatch.setattr(land, "emit_native_timing", broken)
    if failure:
        with pytest.raises(RuntimeError) as caught:
            land.run_case("accepted-without-mode", "unknown", True)
        assert caught.value is original
    else:
        with pytest.raises(ValueError, match="PRIVATE_SENTINEL"):
            land.run_case("accepted-without-mode", "unknown", True)
    assert events[-4:] == ["stop", "emit", "cleanup", "peer_stop"]


def test_stop_failure_preserves_original_error_and_retains_live_storage(modules, monkeypatch, tmp_path):
    _, land = modules
    original = RuntimeError("synthetic original failure")
    events = owned_case(monkeypatch, land, tmp_path, failure=original)
    process = SimpleNamespace(
        poll=lambda: None,
        returncode=None,
        _nomad_test_storage=SimpleNamespace(name=str(tmp_path), cleanup=lambda: events.append("cleanup")),
    )
    monkeypatch.setattr(land, "start_runtime", lambda *_a, **_kw: (process, io.BytesIO(), io.BytesIO()))

    def stop(_):
        events.append("stop")
        raise TimeoutError("synthetic shutdown")

    monkeypatch.setattr(land, "stop_runtime", stop)
    with pytest.raises(RuntimeError) as caught:
        land.run_case("accepted-without-mode", "unknown", True)
    assert caught.value is original and "cleanup" not in events and events[-1] == "peer_stop"


@pytest.mark.parametrize("cli", [False, True])
def test_observer_preparation_and_cached_replay_are_outside_original_elapsed_boundary(modules, monkeypatch, cli):
    _, land = modules
    clock, sends = [0.0], []
    peer = SimpleNamespace(land_count=lambda: 1, _armed=True, _relative_altitude_m=5)
    monkeypatch.setattr(land.time, "monotonic", lambda: clock[0])
    monkeypatch.setattr(land.time, "sleep", lambda _: None)

    def anchor():
        clock[0] += 0.2
        return {"before_ns": 0, "unix_ms": 0, "after_ns": 0}

    monkeypatch.setattr(land, "clock_anchor", anchor)
    if cli:
        monkeypatch.setattr(land, "find_cli", lambda: Path("synthetic"))

        def command(arguments, **_):
            if arguments[-1] == "land":
                clock[0] += 3
            return SimpleNamespace(
                returncode=0, stdout="LAND mode observed; touchdown not verified", stderr="PRIVATE_SENTINEL"
            )

        monkeypatch.setattr(land.subprocess, "run", command)
        timing = {}
        _, _, elapsed = land.exercise_cli(0, peer, timing)
    else:
        monkeypatch.setattr(land, "admit", lambda _: None)
        monkeypatch.setattr(land, "bound_land", lambda *_: {})

        def send(_port, _message, *, timing=None):
            sends.append(timing is not None)
            clock[0] += 3 if timing is not None else 0.5
            if timing is not None:
                timing["prepare_begin_ns"] = 123
            return {"outcome": "unknown", "command_result": {"acknowledged": True}}

        monkeypatch.setattr(land, "send_request", send)
        timing = {}
        _, _, elapsed = land.exercise_request(0, peer, "accepted-without-mode", "unknown", True, timing)
        assert sends == [True, False] and timing["client"] == {"prepare_begin_ns": 123}
    assert elapsed == pytest.approx(3.0)
    assert timing["returned_ns"] - timing["started_ns"] == 3_000_000_000


def test_broken_diagnostic_writer_preserves_original_exception_and_cleanup(modules, monkeypatch, tmp_path):
    _, land = modules
    original = RuntimeError("synthetic original failure")
    events = owned_case(monkeypatch, land, tmp_path, failure=original)
    monkeypatch.setattr(
        land, "print", lambda *_a, **_kw: (_ for _ in ()).throw(OSError("PRIVATE_SENTINEL")), raising=False
    )
    with pytest.raises(RuntimeError) as caught:
        land.run_case("accepted-without-mode", "unknown", True)
    assert caught.value is original and events[-5:] == ["stop", "native_emit", "emit", "cleanup", "peer_stop"]


def test_cli_internal_boundaries_are_unavailable_and_peer_offsets_share_origin(modules, tmp_path, capsys):
    _, land = modules
    peer = SimpleNamespace(
        first_land_at=2.0,
        land_ack_at=2.1,
        land_ack_send_begin_ns=2_100_001_000,
        land_ack_send_return_ns=2_100_002_000,
        heartbeats=[(2.2, 4), (3.2, land.MODE_LAND)],
        land_count=lambda: 1,
    )
    land.emit_land_timing(
        "installed-cli-one-hertz", peer, tmp_path, {"started_ns": 1_000_000_000, "returned_ns": 3_300_000_000}, True
    )
    facts = json.loads(capsys.readouterr().out)
    assert all(value is None for value in facts["client_us"].values())
    assert facts["peer_us"] == {
        "first_land_decoded": 1_000_000,
        "ack_loop_entry": 1_100_000,
        "ack_send_begin": 1_100_001,
        "ack_send_return": 1_100_002,
        "first_heartbeat_attempt_after_land": 1_200_000,
        "first_heartbeat_attempt_after_ack": 1_200_000,
        "first_land_mode_heartbeat_attempt_after_ack": 2_200_000,
    }


@pytest.mark.parametrize("value", ["PRIVATE_SENTINEL", True, float("nan"), -1])
def test_malformed_clock_values_stay_null_and_unavailable(modules, tmp_path, capsys, value):
    _, land = modules
    peer = SimpleNamespace(
        first_land_at=None,
        land_ack_at=None,
        land_ack_send_begin_ns=None,
        land_ack_send_return_ns=None,
        heartbeats=[],
        land_count=lambda: 0,
    )
    timing = {"started_ns": 0, "wall_start": {"before_ns": 0, "unix_ms": value, "after_ns": 0}}
    land.emit_land_timing("accepted-without-mode", peer, tmp_path, timing, False)
    text = capsys.readouterr().out
    facts = json.loads(text)
    assert facts["clock_anchors"]["wall_start"] is None and facts["clock_correlation"] == "unavailable"
    assert facts["measurement"] == "partial" and "PRIVATE_SENTINEL" not in text
