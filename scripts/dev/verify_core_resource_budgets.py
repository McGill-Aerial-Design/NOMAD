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
}


def validate_report(report: dict) -> None:
    if report.get("schema_version") != 1:
        raise ValueError("unsupported resource schema_version")
    environment = report.get("environment", {})
    missing = METADATA_KEYS - environment.keys()
    if missing:
        raise ValueError(f"missing resource environment metadata: {sorted(missing)}")
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
    if not report.get("metrics"):
        raise ValueError("no measured metrics")
    for name, item in report["metrics"].items():
        value = item.get("value")
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value < 0:
            raise ValueError(f"invalid nonnegative finite measurement: {name}")
        expected = "seconds" if name.endswith("seconds") else "bytes"
        if item.get("unit") != expected:
            raise ValueError(f"wrong measurement unit: {name}")
        if "budget" not in item or item.get("status") not in {"unapproved", "pass", "fail", "advisory"}:
            raise ValueError(f"missing budget/status: {name}")


def validate_sections(report: dict) -> None:
    required = {
        "composition": {"static_libraries", "linkage"},
        "runtime": {
            "samples",
            "ipc_distribution",
            "vehicle_distribution",
            "memory_method",
            "stabilization_seconds",
            "poll_seconds",
            "boundary",
        },
        "phases": set(),
    }
    for name, keys in required.items():
        section = report.get(name)
        if not isinstance(section, dict) or not keys.issubset(section):
            raise ValueError(f"missing or incomplete resource section: {name}")
    runtime = report["runtime"]
    if not isinstance(runtime["samples"], list) or len(runtime["samples"]) != 5:
        raise ValueError("resource protocol requires five runtime samples")
    for name in ("ipc_distribution", "vehicle_distribution"):
        distribution = runtime[name]
        if not isinstance(distribution, dict) or set(distribution) != {"count", "min", "median", "p95", "max"}:
            raise ValueError(f"incomplete {name}")
        values = [distribution[key] for key in ("min", "median", "p95", "max")]
        if distribution["count"] != 5 or any(not isinstance(value, (int, float)) for value in values):
            raise ValueError(f"invalid {name}")
        if values != sorted(values) or any(not math.isfinite(value) or value < 0 for value in values):
            raise ValueError(f"invalid {name} range")


def select_profile(report: dict, policy: dict) -> dict:
    environment = report["environment"]
    name = f"{environment['os']}-{environment['architecture']}-{environment['compiler']['id']}"
    if name not in policy["profiles"]:
        raise ValueError(f"unapproved resource environment {name}; establish a reviewed baseline")
    profile = policy["profiles"][name]
    for key, value in profile["comparable"].items():
        if environment.get(key) != value:
            raise ValueError(f"incomparable {key}: measured={environment.get(key)!r}, approved={value!r}")
    return profile


def apply_budgets(report: dict, policy: dict) -> list[str]:
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
    report["policy_version"] = policy["policy_version"]
    return failures


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
