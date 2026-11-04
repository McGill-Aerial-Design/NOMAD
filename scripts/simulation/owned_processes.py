# SPDX-License-Identifier: Apache-2.0
"""Stop only a recorded run-owned Linux session, including surviving child process groups."""

import os
import signal
import subprocess
import time
from pathlib import Path


def members(session):
    found = []
    for path in Path("/proc").iterdir():
        if not path.name.isdecimal():
            continue
        try:
            fields = (path / "stat").read_text().rsplit(")", 1)[1].split()
            if int(fields[3]) == session and fields[0] != "Z":
                try:
                    environment = (path / "environ").read_bytes().split(b"\0")
                except PermissionError:
                    # Graphics drivers may make their process nondumpable; the owned session still fences descendants.
                    environment = None
                found.append((int(path.name), environment))
        except (FileNotFoundError, ProcessLookupError):
            continue
    return found


def stop(process, run_id):
    marker = f"NOMAD_SIMULATION_RUN_ID={run_id}".encode()
    for action, deadline in ((signal.SIGTERM, 5), (signal.SIGKILL, 2)):
        current = members(process.pid)
        if not current:
            break
        if any(environment is not None and marker not in environment for _, environment in current):
            raise RuntimeError("Refusing to signal a session with unowned members")
        for pid, _ in current:
            try:
                if os.getsid(pid) != process.pid:
                    raise RuntimeError("Owned session membership changed before signaling")
                os.kill(pid, action)
            except ProcessLookupError:
                pass
        end = time.monotonic() + deadline
        while members(process.pid) and time.monotonic() < end:
            time.sleep(0.02)
    try:
        process.wait(timeout=1)
    except subprocess.TimeoutExpired as error:
        raise TimeoutError("Owned group leader did not exit") from error
    if members(process.pid):
        raise TimeoutError("Owned descendants survived cleanup")
