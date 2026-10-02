# SPDX-License-Identifier: Apache-2.0
"""Trusted test-only start hook substitutes Task Scheduler with a real child host."""

import os
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def main() -> int:
    options = (
        {"creationflags": subprocess.CREATE_NEW_PROCESS_GROUP | subprocess.DETACHED_PROCESS}
        if os.name == "nt"
        else {"start_new_session": True}
    )
    subprocess.Popen(
        [sys.executable, str(ROOT / "scripts/release/router_host.py"), *sys.argv[1:]],
        stdin=subprocess.DEVNULL,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        **options,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
