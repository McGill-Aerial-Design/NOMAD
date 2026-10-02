# SPDX-License-Identifier: Apache-2.0
"""Collect production-core software resource evidence; optionally build and qualify first."""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

from core_resource_phases import build_release, package_release, qualify_release, save_phases
from resource_footprint import collect_footprint, release_binary
from resource_metadata import ROOT, collect_metadata, write_report
from runtime_resource_measurement import collect_runtime


def summarize(report: dict) -> None:
    rows = ["### Production core resources", "", "| Metric | Measured | Budget | Status |", "|---|---:|---:|---|"]
    for name, metric in report["metrics"].items():
        rows.append(f"| {name} | {metric['value']} | {metric.get('budget', 'unapproved')} | {metric['status']} |")
    summary = "\n".join(rows) + "\n"
    print(summary)
    if os.environ.get("GITHUB_STEP_SUMMARY"):
        with Path(os.environ["GITHUB_STEP_SUMMARY"]).open("a", encoding="utf-8") as stream:
            stream.write(summary)


def measure(build: Path, output: Path, observe: bool, qualify: bool) -> dict:
    cache_state = "incremental" if (build / "CMakeCache.txt").exists() else "cold-build-tree"
    durations = {}
    if not observe:
        try:
            build_release(build, durations)
            if qualify:
                qualify_release(build, durations)
            package_release(build, durations)
        finally:
            save_phases(build, durations, cache_state)
    footprint, composition = collect_footprint(build)
    runtime, detail = collect_runtime(release_binary(build, "nomad-runtime"))
    values = {**footprint, **runtime}
    values.update({f"phase_{key}_seconds": item["seconds"] for key, item in durations.items()})
    report = {
        "schema_version": 1,
        "environment": collect_metadata(build, cache_state),
        "metrics": {
            key: {
                "value": value,
                "unit": "seconds" if key.endswith("seconds") else "bytes",
                "budget": None,
                "status": "unapproved",
            }
            for key, value in values.items()
        },
        "composition": composition,
        "runtime": detail,
        "phases": durations,
    }
    write_report(output, report)
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build-dir", type=Path, default=ROOT / "build/resources")
    parser.add_argument("--output", type=Path, default=ROOT / "build/resources/metrics.json")
    parser.add_argument("--observe", action="store_true", help="measure an existing Release build/stage")
    parser.add_argument("--qualify", action="store_true", help="run all deterministic native qualifications")
    arguments = parser.parse_args()
    report = measure(arguments.build_dir.resolve(), arguments.output.resolve(), arguments.observe, arguments.qualify)
    summarize(report)
    return 0


if __name__ == "__main__":
    sys.exit(main())
