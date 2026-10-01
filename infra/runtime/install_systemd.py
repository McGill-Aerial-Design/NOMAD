#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Explicit runtime service registration; never enable/start or touch operator state."""

from __future__ import annotations

import argparse
import re
import subprocess
from pathlib import Path


def unit_path(value: Path) -> str:
    """Quote a deliberate absolute systemd path without specifier expansion."""
    text = str(value)
    if not value.is_absolute() or any(character in text for character in '\n\r"\\$'):
        raise ValueError("use absolute paths without control, quote, backslash or dollar characters")
    return text.replace("%", "%%")


def render_unit(executable: Path, config: Path, state: Path, user: str) -> str:
    """Render package paths rather than a developer checkout path."""
    if not re.fullmatch(r"[a-z_][a-z0-9_-]{0,31}", user):
        raise ValueError("invalid service account")
    template = Path(__file__).with_name("nomad-runtime.service.in").read_text(encoding="utf-8")
    values = {"USER": user, "EXECUTABLE": unit_path(executable), "CONFIG": unit_path(config), "STATE": unit_path(state)}
    for name, value in values.items():
        template = template.replace(f"@{name}@", value)
    return template


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("render", "install", "uninstall"))
    parser.add_argument("--executable", type=Path)
    parser.add_argument("--config", type=Path)
    parser.add_argument("--state", type=Path)
    parser.add_argument("--user", default="nomad")
    arguments = parser.parse_args()
    destination = Path("/etc/systemd/system/nomad-runtime.service")
    if arguments.action == "uninstall":
        subprocess.run(["systemctl", "disable", "--now", "nomad-runtime.service"], check=True)
        destination.unlink(missing_ok=True)
    else:
        if None in (arguments.executable, arguments.config, arguments.state):
            parser.error("--executable, --config and --state are required")
        unit = render_unit(arguments.executable, arguments.config, arguments.state, arguments.user)
        if arguments.action == "render":
            print(unit, end="")
            return 0
        destination.write_text(unit, encoding="utf-8")
    subprocess.run(["systemctl", "daemon-reload"], check=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
