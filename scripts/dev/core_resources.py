# SPDX-License-Identifier: Apache-2.0
"""Collect production-core software resource evidence; optionally build and qualify first."""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

from core_resource_phases import build_release, package_release, qualify_release, save_phases
from resource_footprint import collect_footprint, release_binary
from resource_metadata import ROOT, collect_metadata, git_sha, write_report
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
    source_sha = git_sha(ROOT)
    marker = build / "resource-build-sha.txt"
    if observe and (not marker.is_file() or marker.read_text(encoding="utf-8").strip() != source_sha):
        raise ValueError("existing build has no matching resource-source marker; rerun the full resource build")
    cache_state = "incremental" if (build / "CMakeCache.txt").exists() else "cold-build-tree"
    durations = {}
    if not observe:
        build_and_qualify(build, qualify, durations, cache_state)
        marker.write_text(source_sha + "\n", encoding="utf-8")
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
    report["environment"]["phase_cache_state"] = cache_state if durations else "unmeasured"
    if report["environment"]["nomad_sha"] != source_sha:
        raise ValueError("NOMAD HEAD changed during collection; discard this sample and rerun")
    write_report(output, report)
    return report


def build_and_qualify(build: Path, qualify: bool, durations: dict, cache_state: str) -> None:
    try:
        build_release(build, durations)
        if qualify:
            qualify_release(build, durations)
        package_release(build, durations)
    finally:
        save_phases(build, durations, cache_state)


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
