# SPDX-License-Identifier: Apache-2.0
"""Read-only owned-session and aggregate GPU observations for the Linux cohort."""

import json
import math
import os
import subprocess
import time
from pathlib import Path
from statistics import median


def snapshot(groups):
    cpu = rss = 0
    members = []
    for path in Path("/proc").iterdir():
        if not path.name.isdecimal():
            continue
        try:
            fields = (path / "stat").read_text().rsplit(")", 1)[1].split()
            if int(fields[3]) in groups:
                members.append(int(path.name))
                cpu += (int(fields[11]) + int(fields[12])) / os.sysconf("SC_CLK_TCK")
                rss += int(fields[21]) * os.sysconf("SC_PAGE_SIZE")
        except (FileNotFoundError, ProcessLookupError, PermissionError):
            continue
    memory = {
        line.split(":")[0]: int(line.split()[1]) * 1024 for line in Path("/proc/meminfo").read_text().splitlines()
    }
    result = {
        "linux_monotonic_ns": time.monotonic_ns(),
        "utc_ns": time.time_ns(),
        "cpu_seconds": cpu,
        "group_rss_bytes": rss,
        "members": members,
        "guest_available_bytes": memory["MemAvailable"],
        "gpu_used_mib": None,
    }
    try:
        gpu = subprocess.run(
            ["/usr/lib/wsl/lib/nvidia-smi", "--query-gpu=memory.used,memory.total", "--format=csv,noheader,nounits"],
            capture_output=True,
            text=True,
            timeout=2,
            check=True,
        )
        used, total = gpu.stdout.strip().split(",")
        result.update(gpu_used_mib=int(used), gpu_total_mib=int(total))
    except (OSError, ValueError, subprocess.SubprocessError) as error:
        result["gpu_error"] = type(error).__name__
    return result


def append(path, groups):
    with Path(path).open("a") as output:
        output.write(json.dumps(snapshot(groups)) + "\n")


def summarize_endurance(samples, duration):
    samples = [
        row for row in samples if row.get("tracking_seconds") is not None and 300 <= row["tracking_seconds"] <= duration
    ]
    first = [row for row in samples if row["tracking_seconds"] <= 360]
    final = [row for row in samples if row["tracking_seconds"] >= duration - 60]
    if len(samples) < 0.95 * (duration - 300) or len(first) < 57 or len(final) < 57:
        return {"passed": False, "failure": "Insufficient post-warm-up resource samples", "samples": len(samples)}
    fields = ("private_bytes", "handle_count", "gdi_objects", "user_objects")
    if any(not math.isfinite(row[field]) or row[field] < 0 for row in samples for field in fields):
        raise ValueError("Invalid native resource observation")
    growth = {field: median(row[field] for row in final) - median(row[field] for row in first) for field in fields}
    seconds = [row["tracking_seconds"] for row in samples]
    values = [row["private_bytes"] / 1048576 for row in samples]
    mean_x, mean_y = sum(seconds) / len(seconds), sum(values) / len(values)
    slope = (
        60 * sum((x - mean_x) * (y - mean_y) for x, y in zip(seconds, values)) / sum((x - mean_x) ** 2 for x in seconds)
    )
    gaps = [b - a for a, b in zip(seconds, seconds[1:])]
    cpu = [
        (b["process_cpu_ms"] - a["process_cpu_ms"]) / 1000 / (b["tracking_seconds"] - a["tracking_seconds"])
        for a, b in zip(samples, samples[1:])
    ]
    passed = (
        max(gaps) <= 3
        and growth["private_bytes"] <= 100 * 1048576
        and slope <= 2
        and all(growth[field] <= 20 for field in fields[1:])
    )
    return {
        "passed": passed,
        "samples": len(samples),
        "warmup_seconds": 300,
        "duration_seconds": duration,
        "growth": growth,
        "private_slope_mib_per_minute": slope,
        "maximum_sample_gap_seconds": max(gaps),
        "cpu_cores_median": median(cpu),
        "cpu_cores_max": max(cpu),
        "scope": "continuous video/Area stability; no model/source cycling; pixel oracle covers initial slice",
    }


def recording_bytes(run):
    total = 0
    for folder in (run / "frames", run / "decoded"):
        if folder.is_dir():
            with os.scandir(folder) as entries:
                total += sum(
                    entry.stat(follow_symlinks=False).st_size
                    for entry in entries
                    if entry.is_file(follow_symlinks=False)
                )
    with os.scandir(run) as entries:
        total += sum(
            entry.stat(follow_symlinks=False).st_size
            for entry in entries
            if entry.is_file(follow_symlinks=False) and Path(entry.name).suffix in (".jsonl", ".csv", ".log", ".png")
        )
    return total


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--endurance", required=True, type=Path)
    parser.add_argument("duration", type=int)
    args = parser.parse_args()
    if not 600 <= args.duration <= 1800:
        raise ValueError("Unsupported endurance duration")
    rows = [json.loads(line) for line in (args.endurance / "video-health.jsonl").read_text().splitlines()]
    result = summarize_endurance(rows, args.duration)
    result["recording_bytes"] = recording_bytes(args.endurance)
    result["passed"] = result["passed"] and result["recording_bytes"] <= 1073741824
    (args.endurance / "endurance-result.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result))
    raise SystemExit(0 if result["passed"] else 1)
