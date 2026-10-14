# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Export only synthetic pulse timing facts from CTest's retained runtime-test output."""

from __future__ import annotations

import argparse
import json
import re
from collections.abc import Iterable
from pathlib import Path

AUDIT_LIMIT = "pre-native-write entries; final native write/flush/response stages are not observed"
PHASES = {
    "blocked_response",
    "case_failure_before_runtime_teardown",
    "controlled_final_audit_entry_timeout",
    "controlled_original_response_received_without_resend",
    "controlled_failure_before_runtime_teardown",
}
CLIENT_LIMITS = {
    "send_begin_us": (0, 2**63 - 1),
    "send_complete_us": (0, 2**63 - 1),
    "read_begin_us": (0, 2**63 - 1),
    "first_byte_us": (0, 2**63 - 1),
    "read_complete_us": (0, 2**63 - 1),
    "error_us": (0, 2**63 - 1),
    "error_code": (0, 2**31 - 1),
    "failed_count": (-1, 65536),
    "partial_bytes": (0, 65536),
}


def project_pulse_record(record: object, sample: int | None) -> dict | None:
    """Reject unexpected fields/types/strings; reconstruct only the known synthetic record."""
    if not isinstance(record, dict) or set(record) != {
        "event",
        "phase",
        "client",
        "commands_before_blocked",
        "audit_entries",
        "audit_limit",
    }:
        return None
    if (
        record["event"] != "pulse_response_diagnostic"
        or not isinstance(record["phase"], str)
        or record["phase"] not in PHASES
    ):
        return None
    commands = record["commands_before_blocked"]
    if record["audit_limit"] != AUDIT_LIMIT or (commands is not None and (type(commands) is not int or commands != 3)):
        return None
    client = record["client"]
    if not isinstance(client, dict) or set(client) != set(CLIENT_LIMITS):
        return None
    for name, (minimum, maximum) in CLIENT_LIMITS.items():
        value = client[name]
        if value is not None and (type(value) is not int or not minimum <= value <= maximum):
            return None
    entries = record["audit_entries"]
    if not isinstance(entries, list) or len(entries) > 12:
        return None
    for entry in entries:
        if not isinstance(entry, dict) or set(entry) != {"request", "audit_entry", "ordinal", "result", "elapsed_us"}:
            return None
        if entry["request"] not in ("pulse-confirm-two", "pulse-blocked"):
            return None
        if entry["audit_entry"] not in ("mutation_intent", "mutation_outcome", "request_rejected"):
            return None
        if entry["result"] not in ("pending", "success", "failed", "rejected", "unknown", "interrupted"):
            return None
        if type(entry["ordinal"]) is not int or not 1 <= entry["ordinal"] <= 2**63 - 1:
            return None
        if type(entry["elapsed_us"]) is not int or not 0 <= entry["elapsed_us"] <= 2**63 - 1:
            return None
    return {
        "sample": None if record["phase"].startswith("controlled_") else sample,
        "phase": record["phase"],
        "client": dict(client),
        "commands_before_blocked": record["commands_before_blocked"],
        "audit_entries": [dict(entry) for entry in entries],
    }


def extract_pulse_diagnostics(lines: Iterable[str]) -> dict:
    """Read one named CTest output block; ambiguous/rejected/truncated data stays incomplete."""
    records, markers = [], []
    sections = rejected = 0
    output_starts = 0
    target = output = ended = False
    test_status = "not_found"
    sample = None
    for raw in lines:
        line = raw.strip()
        boundary = re.fullmatch(r"\d+/\d+ Test: (.+)", line)
        if boundary:
            target = boundary[1] == "nomad_runtime_tests"
            output = False
            if target:
                sections += 1
                test_status = "unknown"
            continue
        if not target or sections != 1:
            continue
        if line == "Output:":
            output_starts += 1
            output = output_starts == 1
            if not output:
                records, markers, ended = [], [], False
        elif line == "<end of output>":
            output, ended = False, True
        elif not output and line in ("Test Passed.", "Test Failed."):
            test_status = "passed" if line == "Test Passed." else "failed"
        elif output and line.startswith("pulse_response_sample="):
            match = re.fullmatch(r"pulse_response_sample=([1-9]|1\d|20)", line)
            sample = int(match[1]) if match else None
            if sample is None or len(markers) >= 20:
                rejected += 1
            else:
                markers.append(sample)
        elif output and line.startswith("{"):
            try:
                record = json.loads(line) if len(line) <= 65536 else None
            except (ValueError, RecursionError):
                record = None
            if not isinstance(record, dict) or record.get("event") != "pulse_response_diagnostic":
                rejected += int('"pulse_response_diagnostic"' in line)
                continue
            safe = project_pulse_record(record, sample)
            if safe is None or len(records) >= 32:
                rejected += 1
            else:
                records.append(safe)
    if sections > 1:
        records, markers, test_status = [], [], "ambiguous"
    samples = [row["sample"] for row in records if row["phase"] == "blocked_response" and row["sample"] is not None]
    phases = [row["phase"] for row in records]
    complete = ended and output_starts == 1 and test_status == "passed" and rejected == 0
    complete &= markers == samples == list(range(1, 21))
    complete &= len(records) == 23 and phases.count("blocked_response") == 21
    complete &= phases.count("controlled_final_audit_entry_timeout") == 1
    complete &= phases.count("controlled_original_response_received_without_resend") == 1
    complete &= phases[-2:] == [
        "controlled_final_audit_entry_timeout",
        "controlled_original_response_received_without_resend",
    ]
    successful = [
        row
        for row in records
        if row["phase"] in ("blocked_response", "controlled_original_response_received_without_resend")
    ]
    stamps = ("send_begin_us", "send_complete_us", "read_begin_us", "first_byte_us", "read_complete_us")
    complete &= all(
        all(row["client"][key] is not None for key in stamps)
        and [row["client"][key] for key in stamps] == sorted(row["client"][key] for key in stamps)
        and all(row["client"][key] is None for key in ("error_us", "error_code", "failed_count", "partial_bytes"))
        and {"mutation_intent", "mutation_outcome", "request_rejected"}
        <= {entry["audit_entry"] for entry in row["audit_entries"] if entry["request"] == "pulse-blocked"}
        for row in successful
    )
    timeouts = [row["client"] for row in records if row["phase"] == "controlled_final_audit_entry_timeout"]
    complete &= all(
        row["error_us"] is not None
        and row["read_begin_us"] is not None
        and row["error_us"] >= row["read_begin_us"]
        and row["error_code"] in (11, 10060)
        and row["failed_count"] == -1
        and row["partial_bytes"] == 0
        and row["first_byte_us"] is None
        and row["read_complete_us"] is None
        for row in timeouts
    )
    complete &= all(row["commands_before_blocked"] == 3 for row in records)
    status = "complete" if complete else "partial" if records or sections > 1 or output_starts > 1 else "unavailable"
    return {
        "schema_version": 1,
        "status": status,
        "runtime_test_status": test_status,
        "target_sections": sections,
        "output_sections": output_starts,
        "output_ended": ended,
        "rejected_records": rejected,
        "cohort_samples": samples,
        "expected_cohort_samples": 20,
        "records": records,
        "audit_limit": AUDIT_LIMIT,
        "original_host_cause": "unresolved",
        "production_fix": False,
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        with args.input.open(encoding="utf-8", errors="replace") as source:
            result = extract_pulse_diagnostics(source)
    except OSError:
        result = extract_pulse_diagnostics([])
        result["input_status"] = "unavailable"
    else:
        result["input_status"] = "read"
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print(json.dumps({"status": result["status"], "records": len(result["records"])}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
