# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Safe measurement delivery from actual CTest-shaped retained output."""

import copy
import json
from pathlib import Path

import pytest
import yaml

from scripts.ci.export_pulse_diagnostics import extract_pulse_diagnostics, main


def diagnostic(phase="blocked_response"):
    return {
        "event": "pulse_response_diagnostic",
        "phase": phase,
        "client": {
            "send_begin_us": 10,
            "send_complete_us": 20,
            "read_begin_us": 21,
            "first_byte_us": 200,
            "read_complete_us": 300,
            "error_us": None,
            "error_code": None,
            "failed_count": None,
            "partial_bytes": None,
        },
        "commands_before_blocked": 3,
        "audit_entries": [
            {
                "request": "pulse-blocked",
                "audit_entry": event,
                "ordinal": index + 11,
                "result": "pending" if index == 0 else "rejected",
                "elapsed_us": 50 + index * 20,
            }
            for index, event in enumerate(("mutation_intent", "mutation_outcome", "request_rejected"))
        ],
        "audit_limit": "pre-native-write entries; final native write/flush/response stages are not observed",
    }


def ctest(output, result="Test Passed.", end=True, command=r"C:\private\operator\binary.exe"):
    lines = [
        "1/2 Test: other_test",
        "Output:",
        json.dumps(diagnostic()),
        "<end of output>",
        "Test Passed.",
        "2/2 Test: nomad_runtime_tests",
        f'Command: "{command}"',
        "Directory: /private/operator/root",
        "Output:",
        "----------------------------------------------------------",
        *output,
    ]
    if end:
        lines.extend(["<end of output>", "Test time = 240.00 sec", "--------------------", result])
    return lines


def complete_output():
    lines = [json.dumps(diagnostic())]
    for sample in range(1, 21):
        lines.extend([f"pulse_response_sample={sample}", json.dumps(diagnostic())])
    for phase in ("controlled_final_audit_entry_timeout", "controlled_original_response_received_without_resend"):
        record = diagnostic(phase)
        if phase.endswith("timeout"):
            record["client"].update(
                first_byte_us=None,
                read_complete_us=None,
                error_us=2000021,
                error_code=10060,
                failed_count=-1,
                partial_bytes=0,
            )
        lines.append(json.dumps(record))
    return lines


@pytest.mark.parametrize("command", [r"C:\private\operator\binary.exe", "/private/operator/binary"])
def test_complete_cohort_and_control_projection_excludes_headers_and_raw_fields(command):
    exported = extract_pulse_diagnostics(ctest(complete_output(), command=command))
    assert exported["status"] == "complete"
    assert exported["runtime_test_status"] == "passed"
    assert len(exported["records"]) == 23
    assert exported["cohort_samples"] == list(range(1, 21))
    assert exported["records"][0]["sample"] is None
    assert [row["sample"] for row in exported["records"][1:21]] == list(range(1, 21))
    assert all(row["sample"] is None for row in exported["records"][-2:])
    assert exported["records"][-2]["client"]["first_byte_us"] is None
    text = json.dumps(exported)
    assert "private" not in text and "operator" not in text and "Command" not in text
    assert "audit_limit" not in exported["records"][0]
    assert exported["original_host_cause"] == "unresolved" and exported["production_fix"] is False


@pytest.mark.parametrize(
    "mutation",
    [
        lambda row: row.update(credential="TOKEN_MUST_NOT_EXPORT"),
        lambda row: row.update(phase=r"C:\private\TOKEN_MUST_NOT_EXPORT"),
        lambda row: row.update(audit_limit="TOKEN_MUST_NOT_EXPORT"),
        lambda row: row.update(phase=[]),
        lambda row: row.update(commands_before_blocked=3.0),
        lambda row: row["client"].update(auth_proof="TOKEN_MUST_NOT_EXPORT"),
        lambda row: row["client"].update(first_byte_us=True),
        lambda row: row["client"].update(first_byte_us=float("nan")),
        lambda row: row["client"].update(first_byte_us=-1),
        lambda row: row["client"].update(first_byte_us=2**63),
        lambda row: row["client"].update(error_code="TOKEN_MUST_NOT_EXPORT"),
        lambda row: row["audit_entries"][0].update(request="TOKEN_MUST_NOT_EXPORT"),
        lambda row: row["audit_entries"][0].update(result="TOKEN_MUST_NOT_EXPORT"),
        lambda row: row["audit_entries"][0].update(path="TOKEN_MUST_NOT_EXPORT"),
        lambda row: row["audit_entries"][0].update(ordinal=True),
        lambda row: row["audit_entries"][0].update(elapsed_us=float("inf")),
        lambda row: row["client"].pop("read_begin_us"),
    ],
)
def test_unknown_fields_unsafe_strings_and_wrong_numeric_types_are_not_persisted(mutation):
    row = copy.deepcopy(diagnostic())
    mutation(row)
    exported = extract_pulse_diagnostics(ctest([json.dumps(row)]))
    assert exported["status"] != "complete"
    assert exported["rejected_records"] == 1
    assert exported["records"] == []
    assert "TOKEN_MUST_NOT_EXPORT" not in json.dumps(exported)


def test_partial_failure_preserves_valid_sample_facts_without_claiming_completion():
    row = diagnostic("case_failure_before_runtime_teardown")
    row["client"].update(
        first_byte_us=None, read_complete_us=None, error_us=2000021, error_code=10060, failed_count=-1, partial_bytes=0
    )
    exported = extract_pulse_diagnostics(
        ctest(
            ["pulse_response_sample=1", json.dumps(diagnostic()), "pulse_response_sample=2", json.dumps(row)],
            result="Test Failed.",
        )
    )
    assert exported["status"] == "partial" and exported["runtime_test_status"] == "failed"
    assert [record["sample"] for record in exported["records"]] == [1, 2]
    assert exported["records"][-1]["client"]["error_code"] == 10060
    assert exported["cohort_samples"] == [1]


def test_truncated_log_and_malformed_candidates_never_claim_complete():
    exported = extract_pulse_diagnostics(
        ctest(
            [
                json.dumps(diagnostic()),
                '{"event":"pulse_response_diagnostic","credential":"TOKEN_MUST_NOT_EXPORT"',
                "pulse_response_sample=999",
                "secret nonmatching text",
                '{"event":"not_a_diagnostic"}',
            ],
            end=False,
        )
    )
    assert exported["status"] == "partial" and exported["runtime_test_status"] == "unknown"
    assert exported["output_ended"] is False and exported["rejected_records"] == 2
    assert len(exported["records"]) == 1
    assert "TOKEN_MUST_NOT_EXPORT" not in json.dumps(exported)


def test_duplicate_runtime_sections_are_explicitly_ambiguous_and_export_no_stale_records():
    exported = extract_pulse_diagnostics(ctest(complete_output()) + ctest(complete_output()))
    assert exported["status"] == "partial" and exported["runtime_test_status"] == "ambiguous"
    assert exported["target_sections"] == 2 and exported["records"] == [] and exported["cohort_samples"] == []


def test_second_output_start_cannot_reuse_a_previous_complete_footer():
    exported = extract_pulse_diagnostics(ctest(complete_output()) + ["Output:", json.dumps(diagnostic())])
    assert exported["status"] != "complete" and exported["output_sections"] == 2
    assert exported["records"] == [] and exported["output_ended"] is False


@pytest.mark.parametrize(
    "mutation",
    [
        lambda row: row.update(client={key: None for key in row["client"]}, audit_entries=[]),
        lambda row: row["client"].update(first_byte_us=0),
        lambda row: row.update(commands_before_blocked=None),
        lambda row: row.update(audit_entries=[]),
    ],
)
def test_present_record_counts_with_missing_or_inconsistent_measurements_stay_partial(mutation):
    lines = complete_output()
    row = json.loads(lines[2])
    mutation(row)
    lines[2] = json.dumps(row)
    exported = extract_pulse_diagnostics(ctest(lines))
    assert exported["status"] == "partial" and len(exported["records"]) == 23


@pytest.mark.parametrize(
    "lines",
    [
        [],
        [json.dumps(diagnostic())],
        ["1/1 Test: other_test", "Output:", json.dumps(diagnostic()), "<end of output>", "Test Passed."],
    ],
)
def test_records_outside_the_named_test_are_unavailable(lines):
    result = extract_pulse_diagnostics(lines)
    assert result["status"] == "unavailable" and result["records"] == []


def test_missing_input_writes_one_safe_unavailable_artifact(tmp_path, capsys):
    output = tmp_path / "artifact" / "pulse.json"
    assert main(["--input", str(tmp_path / "missing.log"), "--output", str(output)]) == 0
    record = json.loads(output.read_text())
    assert record["status"] == "unavailable" and record["input_status"] == "unavailable"
    assert record["records"] == []
    assert len(list(output.parent.iterdir())) == 1
    assert "missing.log" not in capsys.readouterr().out


def test_unreadable_input_is_curated_without_exception_paths(tmp_path):
    output = tmp_path / "pulse.json"
    assert main(["--input", str(tmp_path), "--output", str(output)]) == 0
    assert json.loads(output.read_text())["input_status"] == "unavailable"


def test_cli_exports_failed_test_data_without_changing_its_recorded_failure(tmp_path):
    source = tmp_path / "LastTest.log"
    source.write_text("\n".join(ctest([json.dumps(diagnostic())], result="Test Failed.")))
    output = tmp_path / "pulse.json"
    assert main(["--input", str(source), "--output", str(output)]) == 0
    assert json.loads(output.read_text())["runtime_test_status"] == "failed"
    assert json.loads(output.read_text())["status"] == "partial"


def test_core_workflow_retains_failure_and_uploads_only_the_exported_file():
    workflow = yaml.safe_load((Path(__file__).resolve().parents[1] / ".github/workflows/test.yml").read_text())
    steps = workflow["jobs"]["core"]["steps"]
    build_index = next(index for index, step in enumerate(steps) if step.get("name") == "Build and test C++ core")
    build, exporter, upload = steps[build_index : build_index + 3]
    assert build["run"] == "pixi run test-core" and "continue-on-error" not in build
    assert exporter["if"] == "${{ always() && steps.checkout.outcome == 'success' && steps.pixi.outcome == 'success' }}"
    assert "build/core/Testing/Temporary/LastTest.log" in exporter["run"]
    assert upload["uses"] == "actions/upload-artifact@v6"
    assert upload["if"] == "${{ always() && steps.pulse-diagnostics.outcome == 'success' }}"
    assert upload["with"]["path"] == "${{ runner.temp }}/pulse-diagnostics-${{ matrix.os }}.json"
    assert upload["with"]["name"] == "pulse-diagnostics-${{ matrix.os }}"
    assert upload["with"]["if-no-files-found"] == "error"
