# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors

import sys
from collections.abc import Iterator
from pathlib import Path
from typing import Any

import pytest

_REPO_ROOT = Path(__file__).resolve().parent.parent
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))


@pytest.fixture(scope="module")
def ros_session() -> Iterator[dict[str, Any]]:
    """Load the ROS harness only when a ROS integration test actually runs."""
    from ros_integration_support import ros_session_values

    yield from ros_session_values()
