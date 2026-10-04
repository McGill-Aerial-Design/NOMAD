# SPDX-License-Identifier: Apache-2.0
"""Qualification support must load without running or importing its entry points."""

import subprocess
import sys
from pathlib import Path


def test_runtime_and_release_support_share_credentials_without_entry_points():
    root = Path(__file__).resolve().parents[1]
    result = subprocess.run(
        [
            sys.executable,
            "-c",
            "import sys; sys.path.insert(0, 'scripts/dev'); "
            "import runtime_fixture_support as support; "
            "import runtime_outcome_fixture as outcome; "
            "import runtime_qualification_support as qualification; "
            "import release_process_fixture; "
            "assert support.FIXTURE_CREDENTIALS is outcome.FIXTURE_CREDENTIALS; "
            "assert support.FIXTURE_CREDENTIALS is qualification.FIXTURE_CREDENTIALS; "
            "assert not {'runtime_ipc_smoke', 'runtime_lifecycle_qualification', "
            "'release_process_qualification', 'ground_router_smoke'} & sys.modules.keys()",
        ],
        cwd=root,
        capture_output=True,
        text=True,
        timeout=10,
    )
    assert result.returncode == 0, result.stderr
