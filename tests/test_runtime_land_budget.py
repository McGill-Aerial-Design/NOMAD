# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""LAND accounting preserves both budget bounds, failure priority and owned cleanup."""

import importlib
import io
import json
from pathlib import Path
from types import SimpleNamespace

import pytest


@pytest.fixture
def modules(monkeypatch):
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[1] / "scripts/dev"))
    return importlib.import_module("runtime_fixture_support"), importlib.import_module("runtime_land_fixture")


def owned_case(monkeypatch, land, tmp_path, *, elapsed=3.0, failure=None, native_us=0):
    events = []
    peer = SimpleNamespace(
        land_ack_at=None,
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


@pytest.mark.parametrize("cli", [False, True])
def test_only_first_response_is_timed_and_original_functional_checks_remain(modules, monkeypatch, cli):
    _, land = modules
    clock, sends = [0], []
    peer = SimpleNamespace(land_count=lambda: 1, _armed=True, _relative_altitude_m=5)
    monkeypatch.setattr(land.time, "monotonic_ns", lambda: clock[0])
    monkeypatch.setattr(land.time, "sleep", lambda _: None)
    if cli:
        monkeypatch.setattr(land, "find_cli", lambda: Path("synthetic"))

        def command(arguments, **_):
            clock[0] += 3_000_000_000 if arguments[-1] == "land" else 500_000_000
            return SimpleNamespace(
                returncode=0, stdout="LAND mode observed; touchdown not verified", stderr="PRIVATE_SENTINEL"
            )

        monkeypatch.setattr(land.subprocess, "run", command)
        timing = {}
        assert land.exercise_cli(0, peer, timing) == ("success", True, 3.0)
    else:
        monkeypatch.setattr(land, "admit", lambda _: None)
        monkeypatch.setattr(land, "bound_land", lambda *_: {})

        def send(_port, _message):
            sends.append(True)
            clock[0] += 3_000_000_000 if len(sends) == 1 else 500_000_000
            return {"outcome": "unknown", "command_result": {"acknowledged": True}}

        monkeypatch.setattr(land, "send_request", send)
        timing = {}
        assert land.exercise_request(0, peer, "accepted-without-mode", "unknown", True, timing) == (
            "unknown",
            True,
            3.0,
        )
        assert len(sends) == 2
    assert timing["first_response_complete"] is True
    assert timing["returned_ns"] - timing["started_ns"] == 3_000_000_000
