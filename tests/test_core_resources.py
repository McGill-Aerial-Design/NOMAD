# SPDX-License-Identifier: Apache-2.0
"""Fault-path checks for resource comparability, hard limits and retained evidence."""

from __future__ import annotations

import copy
import json
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts/dev"))

from resource_footprint import collect_footprint, release_binary
from resource_metadata import source_identity
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


@pytest.mark.parametrize("finder", ["connectivity", "transport"])
def test_resource_qualifications_require_release(finder: str, tmp_path: Path, monkeypatch) -> None:
    from mavsdk_connectivity_smoke import find_binary as find_connectivity
    from mavsdk_fixture_harness import find_binary as find_transport

    name = "nomad_mavsdk_connectivity_smoke" if finder == "connectivity" else "nomad-qualification"
    monkeypatch.setenv("NOMAD_RESOURCE_BUILD_DIR", str(tmp_path))
    debug, release = tmp_path / "Debug", tmp_path / "Release"
    debug.mkdir()
    (debug / f"{name}.exe").write_bytes(b"stale debug")
    lookup = find_connectivity if finder == "connectivity" else lambda: find_transport(name)
    with pytest.raises(FileNotFoundError):
        lookup()
    release.mkdir()
    expected = release / f"{name}.exe"
    expected.write_bytes(b"release")
    assert lookup() == expected


def run_git(directory: Path, *arguments: str) -> None:
    subprocess.run(
        ["git", "-C", str(directory), "-c", "protocol.file.allow=always", *arguments],
        check=True,
        capture_output=True,
    )


def commit_fixture(directory: Path) -> None:
    run_git(directory, "add", ".")
    run_git(
        directory,
        "-c",
        "user.name=ResourceTest",
        "-c",
        "user.email=resource@example.invalid",
        "commit",
        "-m",
        "[test,fixture] Create source snapshot",
    )


def create_source_fixture(directory: Path) -> None:
    directory.mkdir()
    run_git(directory, "init", "--initial-branch=main")
    (directory / "source.txt").write_text("known source", encoding="utf-8")
    commit_fixture(directory)


def test_dirty_recursive_sources_cannot_be_attributed_to_commit(tmp_path: Path) -> None:
    root, sdk, proto = [tmp_path / name for name in ("nomad", "sdk", "proto")]
    for directory in (root, sdk, proto):
        create_source_fixture(directory)
    run_git(sdk, "submodule", "add", str(proto), "proto")
    commit_fixture(sdk)
    run_git(root, "submodule", "add", str(sdk), "third_party/MAVSDK")
    run_git(root, "submodule", "update", "--init", "--recursive")
    commit_fixture(root)
    assert len(source_identity(root)["mavsdk_sha"]) == 40
    nested = root / "third_party/MAVSDK/proto"
    (nested / "source.txt").write_text("modified source", encoding="utf-8")
    with pytest.raises(ValueError, match="clean NOMAD checkout and recursive submodules"):
        source_identity(root)


def test_observe_marker_binds_both_source_revisions(tmp_path: Path) -> None:
    from core_resources import validate_build_source

    marker = tmp_path / "resource-build-source.json"
    original = {"nomad_sha": "a" * 40, "mavsdk_sha": "b" * 40}
    marker.write_text(json.dumps(original), encoding="utf-8")
    validate_build_source(marker, original)
    with pytest.raises(ValueError, match="NOMAD/MAVSDK source marker"):
        validate_build_source(marker, {**original, "mavsdk_sha": "c" * 40})


def test_source_change_during_collection_cannot_write_report(tmp_path: Path, monkeypatch) -> None:
    import core_resources

    original = {"nomad_sha": "a" * 40, "mavsdk_sha": "b" * 40}
    identities = iter([original, {**original, "mavsdk_sha": "c" * 40}])
    (tmp_path / "resource-build-source.json").write_text(json.dumps(original), encoding="utf-8")
    monkeypatch.setattr(core_resources, "source_identity", lambda root: next(identities))
    monkeypatch.setattr(core_resources, "collect_footprint", lambda build: ({}, {}))
    monkeypatch.setattr(core_resources, "release_binary", lambda build, name: tmp_path / name)
    monkeypatch.setattr(core_resources, "collect_runtime", lambda binary: ({}, {}))
    monkeypatch.setattr(core_resources, "create_report", lambda *args: {})
    output = tmp_path / "metrics.json"
    with pytest.raises(ValueError, match="source revisions changed"):
        core_resources.measure(tmp_path, output, observe=True, qualify=False)
    assert not output.exists()


def test_foreign_mavsdk_source_cannot_claim_pinned_provenance(tmp_path: Path, monkeypatch) -> None:
    import resource_metadata

    cache = tmp_path / "CMakeCache.txt"
    cache.write_text(f"NOMAD_MAVSDK_SOURCE_DIR:PATH={tmp_path.as_posix()}/different-sdk\n", encoding="utf-8")
    monkeypatch.setattr(resource_metadata, "source_identity", lambda root: {})
    with pytest.raises(ValueError, match="pinned repository MAVSDK checkout"):
        resource_metadata.collect_metadata(tmp_path, "cold-build-tree")


def test_invalid_schema_or_dirty_report_cannot_pass() -> None:
    report, _ = evidence()
    report["schema_version"] = True
    with pytest.raises(ValueError, match="schema_version"):
        validate_report(report)
    report, _ = evidence()
    report["environment"]["source_clean"] = False
    with pytest.raises(ValueError, match="dirty resource source provenance"):
        validate_report(report)


@pytest.mark.parametrize("key,value", [("mode", "hrd"), ("limit", float("nan")), ("limit", True)])
def test_invalid_policy_cannot_silently_disable_hard_gate(key: str, value) -> None:
    report, policy = evidence()
    policy["profiles"]["Linux-x86_64-GNU"]["budgets"]["runtime_binary_bytes"][key] = value
    with pytest.raises(ValueError, match="invalid budget"):
        apply_budgets(report, policy)


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
