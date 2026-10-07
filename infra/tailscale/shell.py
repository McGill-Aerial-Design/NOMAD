# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Shell probe wrapper shared by the Tailscale and network monitors.

Every probe treats a missing binary as exit code 127 and any other failure as a
soft error, so a monitor thread never dies on a failed command.
"""

from __future__ import annotations

import logging
import subprocess

logger = logging.getLogger(__name__)

NOT_INSTALLED_EXIT_CODE = 127


def run_command(cmd: list[str], timeout: float = 10.0) -> tuple[int, str]:
    """Run a command, returning (exit_code, stdout)."""
    try:
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        return result.returncode, result.stdout
    except FileNotFoundError:
        return NOT_INSTALLED_EXIT_CODE, ""
    except Exception as e:  # noqa: BLE001 - probe failure is a soft error
        logger.debug("Command %s failed: %s", cmd[0], e)
        return 1, ""
