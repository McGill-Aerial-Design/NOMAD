# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Execute the hosted aggregate gates against successful and incomplete evidence."""

from __future__ import annotations

import shlex
import shutil
import subprocess
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture(params=[("sitl.yml", "safety-qualification-gate"), ("csharp.yml", "csharp-qualification-gate")])
def gate(request):
    filename, job_name = request.param
    workflow = yaml.safe_load((ROOT / ".github" / "workflows" / filename).read_text(encoding="utf-8"))
    step = next(step for step in workflow["jobs"][job_name]["steps"] if "run" in step)
    values = {name: "success" for name in step["env"]}
    values["REQUIRED"] = "true"
    return step["run"], values


def run_gate(script: str, values: dict[str, str]) -> subprocess.CompletedProcess[bytes]:
    bash = shutil.which("bash")
    assert bash is not None, "qualification gate regression checks require Bash"
    exports = "\n".join(f"export {name}={shlex.quote(value)}" for name, value in values.items())
    # Bytes preserve Bash's LF input when the test runs from Windows.
    source = (exports + "\n" + script).encode("utf-8")
    return subprocess.run([bash, "-s"], input=source, capture_output=True, timeout=15)


@pytest.mark.parametrize("required", ["true", "false"])
def test_gate_accepts_only_complete_required_or_explicitly_unneeded_qualification(gate, required) -> None:
    script, values = gate
    values["REQUIRED"] = required
    if required == "false":
        values.update({name: "skipped" for name in values if name not in {"REQUIRED", "SCOPE_RESULT"}})

    result = run_gate(script, values)

    assert result.returncode == 0, result.stderr


@pytest.mark.parametrize("outcome", ["failure", "cancelled", "skipped"])
def test_required_gate_rejects_each_unsuccessful_suite(gate, outcome) -> None:
    script, successful_values = gate
    for name in successful_values:
        if name in {"REQUIRED", "SCOPE_RESULT"}:
            continue
        values = dict(successful_values, **{name: outcome})

        result = run_gate(script, values)

        assert result.returncode != 0, f"gate accepted {name}={outcome}"


@pytest.mark.parametrize("required", ["", "unknown", "True", "FALSE"])
def test_gate_rejects_missing_or_malformed_scope_output(gate, required) -> None:
    script, values = gate
    values["REQUIRED"] = required

    result = run_gate(script, values)

    assert result.returncode != 0, f"gate accepted invalid scope output {required!r}"
    assert b"Invalid qualification scope output" in result.stderr


@pytest.mark.parametrize("scope_result", ["failure", "cancelled", "skipped", ""])
@pytest.mark.parametrize("required", ["true", "false"])
def test_gate_rejects_unsuccessful_scope_job_regardless_of_output(gate, scope_result, required) -> None:
    script, values = gate
    values.update(SCOPE_RESULT=scope_result, REQUIRED=required)

    result = run_gate(script, values)

    assert result.returncode != 0, f"gate accepted scope job {scope_result!r} with output {required!r}"
