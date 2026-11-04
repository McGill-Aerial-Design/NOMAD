# SPDX-License-Identifier: Apache-2.0
"""Keep private and generated files ignored even inside documentation."""

import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize(
    "path,ignored",
    [
        ("docs/example.key", True),
        ("docs/example.pem", True),
        ("docs/example.log", True),
        ("docs/example.tmp", True),
        ("docs/example.zip", True),
        ("docs/_build/generated.html", True),
        ("local/evidence/README.md", True),
        ("docs/architecture.md", False),
        ("docs/images/dashboard.png", False),
        ("scripts/build/build_plugin_windows.ps1", False),
        ("scripts/release/mission-planner-target.json", False),
    ],
)
def test_ignore_rules(path, ignored):
    result = subprocess.run(["git", "check-ignore", "--no-index", "-q", path], cwd=ROOT, check=False)
    assert result.returncode == (0 if ignored else 1), f"Unexpected ignore rule for {path}"
