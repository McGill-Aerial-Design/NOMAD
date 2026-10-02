# SPDX-License-Identifier: Apache-2.0
"""Fault-path checks for resource comparability, hard limits and retained evidence."""

from __future__ import annotations

import copy
import json
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts/dev"))

from resource_footprint import collect_footprint, release_binary
from runtime_resource_measurement import distribution
from verify_core_resource_budgets import apply_budgets, validate_report


def environment_metadata() -> dict:
    return {
        "nomad_sha": "a" * 40,
        "mavsdk_sha": "b" * 40,
        "os": "Linux",
        "os_release": "test",
        "architecture": "x86_64",
        "compiler": {"id": "GNU", "version": "13.3"},
        "cmake_version": "cmake version 3.28",
        "build_type": "Release",
        "cmake": {"shared": False, "plugins": ["telemetry"]},
        "binary_format": "ELF-unstripped",
        "cache_state": "cold-build-tree",
        "timestamp": "2026-10-02T00:00:00+00:00",
        "protocol": "core-resources-v1",
    }


def evidence() -> tuple[dict, dict]:
    environment = environment_metadata()
    report = {
        "schema_version": 1,
        "environment": environment,
        "metrics": {
            "runtime_binary_bytes": {"value": 100, "unit": "bytes", "budget": None, "status": "unapproved"},
            "phase_core_build_seconds": {"value": 5, "unit": "seconds", "budget": None, "status": "unapproved"},
        },
    }
    report.update(evidence_sections())
    comparable = {
        key: environment[key]
        for key in ("mavsdk_sha", "os", "architecture", "compiler", "build_type", "cmake", "binary_format", "protocol")
    }
    policy = {
        "policy_version": 1,
        "profiles": {
            "Linux-x86_64-GNU": {
                "comparable": comparable,
                "budgets": {
                    "runtime_binary_bytes": {"baseline": 90, "limit": 120, "mode": "hard"},
                    "phase_core_build_seconds": {
                        "baseline": 3,
                        "limit": 4,
                        "mode": "advisory",
                        "cache_state": "cold-build-tree",
                    },
                },
            }
        },
    }
    return report, policy


def evidence_sections() -> dict:
    return {
        "composition": {
            "static_libraries": {},
            "linkage": "static",
            "compiled_plugins": ["telemetry"],
            "compiled_plugin_count": 1,
            "runtime_link_archives": ["libmavsdk.a"],
            "cli_link_archives": ["libnomad_core.a"],
        },
        "phases": {"core_build": {"seconds": 5, "exit_code": 0}},
        "runtime": {
            "samples": [sample_evidence(cycles) for cycles in (300, 30, 30, 30, 30)],
            "ipc_distribution": distribution([1, 2, 3, 4, 5]),
            "vehicle_distribution": distribution([1, 2, 3, 4, 5]),
            "restart_ipc_distribution": distribution([1, 2, 3, 4, 5]),
            "restart_vehicle_distribution": distribution([1, 2, 3, 4, 5]),
            "memory_method": "test",
            "stabilization_seconds": 1,
            "sample_window_seconds": 1,
            "poll_seconds": 0.02,
            "boundary": "software test",
        },
    }


def sample_evidence(cycles: int) -> dict:
    return {
        "operations": {"cycles": cycles},
        "audit_counts": {
            "runtime_start": 1,
            "runtime_shutdown": 1,
            "mutation_intent": cycles,
            "mutation_outcome": cycles,
            "authority_admission": 1,
            "authority_handback": cycles - 1,
            "authority_revoke": cycles,
        },
    }


@pytest.mark.parametrize("section", ["composition", "runtime", "phases"])
def test_schema_sections_are_required(section: str) -> None:
    report, policy = evidence()
    del report[section]
    with pytest.raises(ValueError, match="resource section"):
        apply_budgets(report, policy)


def test_hard_budget_failure_overrides_claimed_pass_and_explains_numbers(capsys) -> None:
    report, policy = evidence()
    report["metrics"]["runtime_binary_bytes"].update({"value": 121, "status": "pass", "budget": 1000000})
    failures = apply_budgets(report, policy)
    assert len(failures) == 1
    assert "measured=121 bytes; budget=120" in failures[0]
    assert report["metrics"]["runtime_binary_bytes"]["status"] == "fail"
    assert "ADVISORY phase_core_build_seconds" in capsys.readouterr().out


def test_budget_boundary_passes_and_advisory_overage_does_not_fail() -> None:
    report, policy = evidence()
    report["metrics"]["runtime_binary_bytes"]["value"] = 120
    assert apply_budgets(report, policy) == []
    assert report["metrics"]["phase_core_build_seconds"]["status"] == "advisory"


def test_incremental_timing_is_retained_without_comparing_cold_baseline() -> None:
    report, policy = evidence()
    report["environment"]["cache_state"] = "incremental"
    assert apply_budgets(report, policy) == []
    timing = report["metrics"]["phase_core_build_seconds"]
    assert timing["status"] == "advisory" and "cache class" in timing["comparison"]


def test_configure_failure_still_retains_phase_record(tmp_path: Path) -> None:
    from core_resource_phases import save_phases

    save_phases(tmp_path, {"configure": {"seconds": 1, "exit_code": 1}}, "cold-build-tree")
    report = json.loads((tmp_path / "resource-phases.json").read_text(encoding="utf-8"))
    assert report["phases"]["configure"]["exit_code"] == 1
    assert report["environment"]["cmake"]["configuration_status"] == "unavailable"
    assert len(report["environment"]["nomad_sha"]) == 40


@pytest.mark.parametrize(
    "key", ["mavsdk_sha", "os", "architecture", "compiler", "build_type", "cmake", "binary_format", "protocol"]
)
def test_incomparable_environment_cannot_pass(key: str) -> None:
    report, policy = evidence()
    report["environment"][key] = "c" * 40 if key == "mavsdk_sha" else "different"
    with pytest.raises((ValueError, TypeError)):
        apply_budgets(report, policy)


@pytest.mark.parametrize("value", [-1, float("nan"), float("inf"), True, "100"])
def test_invalid_measurement_cannot_pass(value) -> None:
    report, _ = evidence()
    report["metrics"]["runtime_binary_bytes"]["value"] = value
    with pytest.raises(ValueError, match="measurement"):
        validate_report(report)


def test_missing_metadata_or_required_measurement_cannot_pass() -> None:
    report, policy = evidence()
    incomplete = copy.deepcopy(report)
    del incomplete["environment"]["timestamp"]
    with pytest.raises(ValueError, match="missing resource environment"):
        apply_budgets(incomplete, policy)
    del report["metrics"]["runtime_binary_bytes"]
    with pytest.raises(ValueError, match="missing required metric runtime_binary_bytes"):
        apply_budgets(report, policy)


def test_release_lookup_never_falls_back_to_debug(tmp_path: Path) -> None:
    debug = tmp_path / "Debug"
    debug.mkdir()
    (debug / "nomad-runtime.exe").write_bytes(b"debug")
    with pytest.raises(FileNotFoundError):
        release_binary(tmp_path, "nomad-runtime")


def test_debug_artifacts_in_stage_reject_footprint(tmp_path: Path) -> None:
    stage = tmp_path / "stage/bin"
    stage.mkdir(parents=True)
    (stage / "runtime.pdb").write_bytes(b"debug")
    with pytest.raises(ValueError, match="debug artifacts"):
        collect_footprint(tmp_path)


def test_distribution_retains_all_five_samples() -> None:
    assert distribution([5, 1, 3, 2, 4]) == {"count": 5, "min": 1, "median": 3, "p95": 5, "max": 5}


def test_changed_workload_or_failed_qualification_cannot_pass() -> None:
    report, policy = evidence()
    report["runtime"]["samples"][0]["operations"]["cycles"] = 30
    with pytest.raises(ValueError, match="operation count"):
        apply_budgets(report, policy)
    report, policy = evidence()
    report["phases"]["core_build"]["exit_code"] = 1
    with pytest.raises(ValueError, match="phases must succeed"):
        apply_budgets(report, policy)


def test_retained_phase_annotations_match_verified_metric() -> None:
    report, policy = evidence()
    assert apply_budgets(report, policy) == []
    phase = report["phases"]["core_build"]
    assert phase["budget"] == 4 and phase["mode"] == "advisory"
    assert phase["status"] == report["metrics"]["phase_core_build_seconds"]["status"] == "advisory"


def test_reviewed_policy_records_measured_baselines_and_headroom() -> None:
    policy = json.loads((ROOT / "config/core-resource-budgets.json").read_text(encoding="utf-8"))
    assert len(policy["profiles"]) == 3
    for profile in policy["profiles"].values():
        assert len(profile["baseline_evidence"]["nomad_sha"]) == 40
        for name, definition in profile["budgets"].items():
            assert definition["limit"] > definition["baseline"] >= 0, name
            assert definition["headroom"] > 0 and definition["rationale"], name
            if name.startswith("phase_"):
                assert definition["mode"] == "advisory" and definition["cache_state"], name


def test_hosted_resources_retains_separate_linux_and_windows_evidence() -> None:
    workflow = (ROOT / ".github/workflows/resources.yml").read_text(encoding="utf-8")
    assert "os: [ubuntu-latest, windows-latest]" in workflow
    assert "core-resource-metrics-${{ matrix.os }}" in workflow
    assert "if: always()" in workflow
    assert "if-no-files-found: error" in workflow
    assert "metrics.json" in workflow and "resource-phases.json" in workflow
    schema = json.loads((ROOT / "config/core-resource-metrics.schema.json").read_text(encoding="utf-8"))
    assert schema["properties"]["schema_version"]["const"] == 1
