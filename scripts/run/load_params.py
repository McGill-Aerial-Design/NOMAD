#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""
Push an ArduPilot .param file to the Cube via MAVLink before a task.

Usage:
    load_params.py <task>                # task = "task1" | "task2" | path to .param
    load_params.py task1 --device /dev/ttyACM0 --baud 115200
    load_params.py task2 --connect udp:127.0.0.1:14550

Exit codes:
    0  all params set (or already equal within tolerance)
    1  connection / protocol failure
    2  one or more params failed to set after retries

File format: ArduPilot `.param` — `NAME,VALUE` per line, comments with `#`.
"""

from __future__ import annotations

import argparse
import logging
import os
import sys
import time
from collections.abc import Iterable
from pathlib import Path

try:
    from pymavlink import mavutil
except ImportError:
    sys.stderr.write(
        "pymavlink is not installed. On the Jetson, run:\n"
        "    sudo apt install python3-pymavlink\n"
        "or in the venv:\n"
        "    pip install pymavlink\n"
    )
    sys.exit(1)

logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")
log = logging.getLogger("load_params")


def _resolve_param_dir() -> Path:
    # Canonical location for ArduPilot parameter sets. The old config/params/
    # directory held a stale subset of these files and has been removed; the
    # profiles in config/profiles/ are the single source of truth.
    env = os.environ.get("NOMAD_DIR")
    if env and (Path(env) / "config" / "profiles").is_dir():
        return Path(env) / "config" / "profiles"
    here = Path(__file__).resolve()
    for ancestor in here.parents:
        cand = ancestor / "config" / "profiles"
        if cand.is_dir():
            return cand
    return Path.home() / "NOMAD" / "config" / "profiles"


PARAM_DIR = _resolve_param_dir()
TOLERANCE = 1e-4
ACK_TIMEOUT = 5.0
RETRIES = 4


def parse_param_file(path: Path) -> dict[str, float]:
    params: dict[str, float] = {}
    with path.open() as f:
        for lineno, raw in enumerate(f, 1):
            line = raw.split("#", 1)[0].strip()
            if not line:
                continue
            # Accept "NAME,VALUE" or "NAME VALUE" or "NAME\tVALUE"
            parts = [p for p in line.replace("\t", " ").replace(",", " ").split(" ") if p]
            if len(parts) != 2:
                log.warning(f"{path.name}:{lineno}: cannot parse {raw.rstrip()!r}")
                continue
            name, value = parts
            try:
                params[name.upper()] = float(value)
            except ValueError:
                log.warning(f"{path.name}:{lineno}: non-numeric value {value!r}")
    return params


def resolve_task_file(task: str) -> Path:
    direct = Path(task)
    if direct.is_file():
        return direct
    # task1 -> task1_outdoor.params, task2 -> task2_indoor.params
    aliases = {
        "task1": "task1_outdoor.params",
        "task2": "task2_indoor.params",
    }
    candidate = PARAM_DIR / aliases.get(task, f"{task}.params")
    if not candidate.is_file():
        raise FileNotFoundError(f"Unknown task {task!r}. Expected one of task1/task2 or a path to a .param file.")
    return candidate


def connect(args) -> mavutil.mavfile:
    if args.connect:
        log.info(f"Connecting to {args.connect}")
        conn = mavutil.mavlink_connection(args.connect)
    else:
        log.info(f"Connecting to {args.device} @ {args.baud}")
        conn = mavutil.mavlink_connection(args.device, baud=args.baud)
    log.info("Waiting for heartbeat...")
    hb = conn.wait_heartbeat(timeout=10)
    if hb is None:
        raise SystemExit(1)
    log.info(f"Heartbeat from sysid={conn.target_system} compid={conn.target_component}")
    return conn


def set_one(conn, name: str, value: float) -> tuple[bool, float]:
    """Send PARAM_SET, wait for echo, verify value. Returns (ok, echoed_value)."""
    name_b = name.encode("ascii")[:16]
    for attempt in range(1, RETRIES + 1):
        conn.mav.param_set_send(
            conn.target_system,
            conn.target_component,
            name_b,
            float(value),
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
        )
        deadline = time.time() + ACK_TIMEOUT
        while time.time() < deadline:
            msg = conn.recv_match(type="PARAM_VALUE", blocking=True, timeout=ACK_TIMEOUT)
            if msg is None:
                continue
            if msg.param_id.strip("\x00") != name:
                continue
            if abs(msg.param_value - value) <= TOLERANCE:
                return True, msg.param_value
            log.warning(f"{name}: wrote {value:g} but FC reported {msg.param_value:g} (attempt {attempt}/{RETRIES})")
            break
    return False, float("nan")


def push(conn, params: dict[str, float]) -> int:
    failed = 0
    for i, (name, value) in enumerate(params.items(), 1):
        ok, got = set_one(conn, name, value)
        tag = "OK" if ok else "FAIL"
        log.info(f"[{i:>3}/{len(params)}] {tag} {name} = {value:g}" + ("" if ok else f" (got {got})"))
        if not ok:
            failed += 1
    return failed


def main(argv: Iterable[str]) -> int:
    p = argparse.ArgumentParser()
    p.add_argument("task", help="'task1', 'task2', or path to a .param file")
    src = p.add_mutually_exclusive_group()
    src.add_argument("--device", default=os.environ.get("MAVLINK_UART_DEV", "/dev/ttyACM0"))
    src.add_argument("--connect", help="mavutil URL, e.g. udp:127.0.0.1:14550")
    p.add_argument("--baud", type=int, default=int(os.environ.get("MAVLINK_UART_BAUD", "115200")))
    p.add_argument("--dry-run", action="store_true", help="parse and print, do not write")
    args = p.parse_args(list(argv))

    path = resolve_task_file(args.task)
    params = parse_param_file(path)
    log.info(f"Loaded {len(params)} params from {path}")
    if not params:
        return 1
    if args.dry_run:
        for k, v in params.items():
            print(f"{k} = {v:g}")
        return 0

    conn = connect(args)
    failed = push(conn, params)
    if failed:
        log.error(f"{failed}/{len(params)} params failed to set")
        return 2
    log.info("All params set successfully.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
