# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Read real C++ runtime STATUS through the Mission Planner client without aircraft writes."""

from __future__ import annotations

import json
import os
import secrets
import socket
import subprocess
import tempfile
import time
from pathlib import Path

from mavsdk_peer import VehiclePeer
from runtime_fixture_support import (
    FIXTURE_CREDENTIALS,
    ROOT,
    find_runtime,
    free_port,
    request,
    require,
    start_runtime,
    stop_runtime,
    wait_for_listener,
)


def verify_profile(binary: Path, probe: Path, connected: bool) -> None:
    udp_port, ipc_port = free_port(socket.SOCK_DGRAM), free_port(socket.SOCK_STREAM)
    peer = VehiclePeer(udp_port, 1) if connected else None
    if peer is not None:
        peer.start()
    process, stdout, stderr = start_runtime(binary, udp_port, ipc_port)
    try:
        wait_for_listener(ipc_port)
        expected = "Copter" if connected else "Unknown"
        deadline = time.monotonic() + 10
        while True:
            status = request(ipc_port, "status-readiness", "status")["status"]
            if status["aircraft_class"] == expected and (
                not connected or status["telemetry"]["position_valid"] and status["telemetry"]["heartbeat_fresh"]
            ):
                break
            require(time.monotonic() < deadline, "runtime STATUS did not reach the known fixture state")
            time.sleep(0.05)
        result = subprocess.run(
            [str(probe), "--runtime-status", str(ipc_port), expected],
            input=(FIXTURE_CREDENTIALS["mission-planner"] + "\n").encode(),
            capture_output=True,
            timeout=10,
            check=False,
        )
        checks = json.loads(result.stdout)
        required = {
            "authenticated_snapshot",
            "listener_ready",
            "protocol_support",
            "authority_unowned",
            "aircraft_class",
            "profile_support",
            "telemetry",
            "altitude_datums",
        }
        require(isinstance(checks, dict) and set(checks) == required, "client STATUS projection has unexpected fields")
        require(all(type(value) is bool for value in checks.values()), "client STATUS facts must be booleans")
        print(
            json.dumps(
                {"scope": "mission-planner-runtime-status", "profile": expected, "stage": "read", "checks": checks}
            ),
            flush=True,
        )
        require(result.returncode == 0 and all(checks.values()), "client rejected actual runtime STATUS facts")
        stop_runtime(process)
        require(process.returncode == 0, "runtime STATUS fixture did not exit cleanly")
        directory = Path(process._nomad_test_storage.name) / "audit"
        records = [json.loads(line) for path in directory.glob("*.jsonl") for line in path.read_bytes().splitlines()]
        require(
            records and not any(row.get("event") == "mutation_intent" for row in records),
            "read-only STATUS created a mutation intent",
        )
        print(
            json.dumps(
                {
                    "scope": "mission-planner-runtime-status",
                    "profile": expected,
                    "checks": checks,
                    "clean_exit": True,
                    "mutation_intents": 0,
                }
            ),
            flush=True,
        )
    finally:
        if process.poll() is None:
            stop_runtime(process)
        if peer is not None:
            peer.stop()
        stdout.close()
        stderr.close()
        storage = process._nomad_test_storage
        path = Path(storage.name).resolve()
        require(
            path.parent == Path(tempfile.gettempdir()).resolve() and path.name.startswith("nomad-runtime-test-"),
            "refuse cleanup outside the owned runtime fixture directory",
        )
        storage.cleanup()


def main() -> int:
    require(os.name == "nt", "Mission Planner STATUS integration requires Windows/.NET Framework")
    binary = find_runtime()
    probe = ROOT / "mission_planner/tests/coreclient/bin/NomadCoreClientTests.exe"
    require(probe.is_file(), "build test-plugin-core-client before STATUS integration")
    # Only this child fixture's environment changes; operator files/configuration are never loaded.
    for key in list(os.environ):
        if key.startswith("NOMAD_"):
            os.environ.pop(key)
    FIXTURE_CREDENTIALS["mission-planner"] = secrets.token_hex(32)
    verify_profile(binary, probe, False)
    verify_profile(binary, probe, True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
