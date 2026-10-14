# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors

import sys
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parent.parent
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))
