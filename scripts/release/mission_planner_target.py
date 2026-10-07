# SPDX-License-Identifier: Apache-2.0
"""Reviewed Mission Planner build and deployment target."""

import json
from pathlib import Path

VERSION = json.loads(Path(__file__).with_name("mission-planner-target.json").read_text(encoding="utf-8"))["version"]
