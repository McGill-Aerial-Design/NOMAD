# SPDX-License-Identifier: Apache-2.0
"""Validate comparable resource evidence and apply reviewed hard/advisory budgets."""

from __future__ import annotations

import argparse
import json
import math
import re
import sys
from datetime import datetime
from pathlib import Path

from resource_metadata import ROOT, write_report

POLICY = ROOT / "config/core-resource-budgets.json"
METADATA_KEYS = {
    "nomad_sha",
    "mavsdk_sha",
    "os",
    "architecture",
    "compiler",
    "cmake_version",
    "build_type",
    "cmake",
    "binary_format",
    "cache_state",
    "timestamp",
    "protocol",
    "os_release",
    "source_clean",
}


def validate_report(report: dict) -> None:
    if not isinstance(report, dict) or type(report.get("schema_version")) is not int or report["schema_version"] != 1:
        raise ValueError("unsupported resource schema_version")
    environment = report.get("environment", {})
    if not isinstance(environment, dict):
        raise ValueError("resource environment must be an object")
    missing = METADATA_KEYS - environment.keys()
    if missing:
        raise ValueError(f"missing resource environment metadata: {sorted(missing)}")
    if environment["source_clean"] is not True:
        raise ValueError("dirty resource source provenance cannot pass")
    if not isinstance(environment["cmake"], dict) or not isinstance(environment["compiler"], dict):
        raise ValueError("CMake configuration and compiler metadata must be objects")
    validate_sections(report)
    for key in ("nomad_sha", "mavsdk_sha"):
        if not re.fullmatch("[a-f0-9]{40}", environment[key]):
            raise ValueError(f"invalid {key}; full commit SHA required")
    if environment["cache_state"] not in {"cold-build-tree", "incremental", "unknown"}:
        raise ValueError("invalid cache-state label")
    if environment["build_type"] != "Release" or environment["protocol"] != "core-resources-v1":
        raise ValueError("unsupported build type or workload protocol")
    if set(environment["compiler"]) != {"id", "version"}:
        raise ValueError("compiler id/version are required")
    if not all(isinstance(value, str) and value for value in environment["compiler"].values()):
        raise ValueError("compiler id/version must be nonempty strings")
    if datetime.fromisoformat(environment["timestamp"]).tzinfo is None:
        raise ValueError("resource timestamp requires an explicit timezone")
    validate_metrics(report.get("metrics"))


def validate_metrics(metrics: dict) -> None:
    if not isinstance(metrics, dict) or not metrics:
        raise ValueError("no measured metrics")
    for name, item in metrics.items():
        if not isinstance(item, dict):
            raise ValueError(f"measurement must be an object: {name}")
        value = item.get("value")
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value < 0:
            raise ValueError(f"invalid nonnegative finite measurement: {name}")
        expected = "seconds" if name.endswith("seconds") else "bytes"
        if item.get("unit") != expected:
            raise ValueError(f"wrong measurement unit: {name}")
        if "budget" not in item or item.get("status") not in {"unapproved", "pass", "fail", "advisory"}:
            raise ValueError(f"missing budget/status: {name}")


def validate_sections(report: dict) -> None:
    schema = json.loads((ROOT / "config/core-resource-metrics.schema.json").read_text(encoding="utf-8"))
    for name in ("composition", "runtime", "phases"):
        keys = set(schema["properties"][name].get("required", []))
        section = report.get(name)
        if not isinstance(section, dict) or not keys.issubset(section):
            raise ValueError(f"missing or incomplete resource section: {name}")
    runtime = report["runtime"]
    composition = report["composition"]
    plugins = composition["compiled_plugins"]
    if plugins != report["environment"]["cmake"].get("plugins") or composition["compiled_plugin_count"] != len(plugins):
        raise ValueError("compiled plugin inventory disagrees with effective CMake configuration")
    if not isinstance(runtime["samples"], list) or len(runtime["samples"]) != 5:
        raise ValueError("resource protocol requires five runtime samples")
    validate_workload(runtime, report["phases"])
    for name in (
        "ipc_distribution",
        "vehicle_distribution",
        "restart_ipc_distribution",
        "restart_vehicle_distribution",
    ):
        distribution = runtime[name]
        if not isinstance(distribution, dict) or set(distribution) != {"count", "min", "median", "p95", "max"}:
            raise ValueError(f"incomplete {name}")
        values = [distribution[key] for key in ("min", "median", "p95", "max")]
        if distribution["count"] != 5 or any(
            isinstance(value, bool) or not isinstance(value, (int, float)) for value in values
        ):
            raise ValueError(f"invalid {name}")
        if values != sorted(values) or any(not math.isfinite(value) or value < 0 for value in values):
            raise ValueError(f"invalid {name} range")


def validate_workload(runtime: dict, phases: dict) -> None:
    for index, sample in enumerate(runtime["samples"]):
        cycles = 300 if index == 0 else 30
        operations = sample.get("operations", {}) if isinstance(sample, dict) else {}
        if not isinstance(operations, dict) or operations.get("cycles") != cycles:
            raise ValueError("incomparable resource operation count")
        audit = sample.get("audit_counts", {})
        expected = {
            "runtime_start": 1,
            "runtime_shutdown": 1,
            "mutation_intent": cycles,
            "mutation_outcome": cycles,
            "authority_admission": 1,
            "authority_handback": cycles - 1,
            "authority_revoke": cycles,
        }
        if audit != expected:
            raise ValueError("resource operation audit counts are incomplete")
    for name, expected in (("poll_seconds", 0.02), ("stabilization_seconds", 1.0), ("sample_window_seconds", 1.0)):
        if runtime.get(name) != expected:
            raise ValueError(f"incomparable runtime {name}")
    if not phases or any(not isinstance(phase, dict) or phase.get("exit_code") != 0 for phase in phases.values()):
        raise ValueError("resource qualification phases must succeed")


def select_profile(report: dict, policy: dict) -> dict:
    environment = report["environment"]
    name = f"{environment['os']}-{environment['architecture']}-{environment['compiler']['id']}"
    profiles = [
        profile
        for profile in policy["profiles"].values()
        if all(profile["comparable"].get(key) == environment[key] for key in ("os", "architecture", "compiler"))
    ]
    if len(profiles) != 1:
        raise ValueError(f"unapproved resource environment {name}; establish a reviewed baseline")
    profile = profiles[0]
    for key, value in profile["comparable"].items():
        if environment.get(key) != value:
            raise ValueError(f"incomparable {key}: measured={environment.get(key)!r}, approved={value!r}")
    return profile


def apply_budgets(report: dict, policy: dict) -> list[str]:
    validate_policy(policy)
    validate_report(report)
    profile = select_profile(report, policy)
    failures = []
    for name, definition in profile["budgets"].items():
        if name not in report["metrics"]:
            raise ValueError(f"missing required metric {name}; rerun the complete collector")
        item = report["metrics"][name]
        if name.startswith("phase_") and definition.get("cache_state") != report["environment"]["cache_state"]:
            item.update(
                {
                    "budget": definition["limit"],
                    "mode": "advisory",
                    "status": "advisory",
                    "comparison": "different build-tree cache class; retain without comparing timing",
                }
            )
            print(f"ADVISORY {name}: timing cache class differs; measured={item['value']} seconds")
            continue
        limit = definition["limit"]
        exceeded = item["value"] > limit
        item.update(
            {
                "budget": limit,
                "mode": definition["mode"],
                "baseline": definition["baseline"],
                "status": ("fail" if definition["mode"] == "hard" else "advisory") if exceeded else "pass",
            }
        )
        message = f"{item['status'].upper()} {name}: measured={item['value']} {item['unit']}; budget={limit}"
        print(message)
        if item["status"] == "fail":
            failures.append(message)
    for name, item in report["metrics"].items():
        if name not in profile["budgets"]:
            item.update({"budget": None, "mode": "advisory", "status": "advisory"})
    annotate_phases(report)
    report["policy_version"] = policy["policy_version"]
    return failures


def validate_policy(policy: dict) -> None:
    if not isinstance(policy, dict) or type(policy.get("policy_version")) is not int or policy["policy_version"] != 1:
        raise ValueError("unsupported resource policy_version")
    profiles = policy.get("profiles")
    if not isinstance(profiles, dict) or not profiles:
        raise ValueError("resource policy requires reviewed profiles")
    for profile in profiles.values():
        if not isinstance(profile, dict) or not isinstance(profile.get("comparable"), dict):
            raise ValueError("resource profile requires comparable metadata")
        definitions = profile.get("budgets")
        if not isinstance(definitions, dict) or not definitions:
            raise ValueError("resource profile requires approved budgets")
        for name, definition in definitions.items():
            if not isinstance(definition, dict) or definition.get("mode") not in {"hard", "advisory"}:
                raise ValueError(f"invalid budget mode: {name}")
            values = [definition.get(key) for key in ("baseline", "limit")]
            if any(isinstance(value, bool) or not isinstance(value, (int, float)) for value in values):
                raise ValueError(f"invalid budget value: {name}")
            if any(not math.isfinite(value) or value < 0 for value in values) or values[1] < values[0]:
                raise ValueError(f"invalid budget range: {name}")


def annotate_phases(report: dict) -> None:
    for name, phase in report["phases"].items():
        metric = report["metrics"].get(f"phase_{name}_seconds", {})
        for key in ("budget", "mode", "status", "baseline", "comparison"):
            if key in metric:
                phase[key] = metric[key]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("metrics", type=Path)
    parser.add_argument("--policy", type=Path, default=POLICY)
    arguments = parser.parse_args()
    try:
        report = json.loads(arguments.metrics.read_text(encoding="utf-8"))
        policy = json.loads(arguments.policy.read_text(encoding="utf-8"))
        failures = apply_budgets(report, policy)
        write_report(arguments.metrics, report)
        write_report(
            arguments.metrics.parent / "resource-phases.json",
            {
                "schema_version": 1,
                "environment": report["environment"],
                "phases": report["phases"],
                "metrics": {name: item for name, item in report["metrics"].items() if name.startswith("phase_")},
            },
        )
        from core_resources import summarize

        summarize(report)
        return 1 if failures else 0
    except (KeyError, TypeError, ValueError, OSError) as error:
        print(f"resource evidence rejected: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
