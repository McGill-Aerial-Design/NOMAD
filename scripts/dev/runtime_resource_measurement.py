# SPDX-License-Identifier: Apache-2.0
"""Bounded Release runtime samples against the existing deterministic UDP peer."""

from __future__ import annotations

import math
import os
import socket
import statistics
import subprocess
import tempfile
import threading
import time
from pathlib import Path

import psutil
from mavsdk_peer import VehiclePeer
from runtime_ipc_smoke import authority_fields, free_port, request, stop_runtime
from runtime_lifecycle_fixture import deployment, wait_for
from runtime_lifecycle_qualification import execute_servo, status, wait_for_vehicle

STABILIZE_SECONDS = 1.0
SAMPLES = 5
CYCLES = 300
POLL_SECONDS = 0.02


def memory_bytes(pid: int) -> dict[str, int]:
    info = psutil.Process(pid).memory_info()
    values = {"resident": info.rss}
    if hasattr(info, "private"):
        values["private"] = info.private
    return values


def distribution(values: list[float]) -> dict:
    ordered = sorted(values)
    return {
        "count": len(values),
        "min": min(values),
        "median": statistics.median(values),
        "p95": ordered[math.ceil(len(values) * 0.95) - 1],
        "max": max(values),
    }


def wait_for_hello(ipc: int) -> None:
    wait_for(
        lambda: request(ipc, "resource-hello", "hello").get("ok") is True, "runtime HELLO not usable without aircraft"
    )


def wait_for_session(ipc: int) -> None:
    wait_for_vehicle(ipc)
    if not status(ipc)["vehicle_session_established"]:
        raise AssertionError("fresh heartbeat requires an established vehicle session")


def launch(binary: Path, config: Path, logs) -> tuple[subprocess.Popen, float]:
    options = {"creationflags": subprocess.CREATE_NEW_PROCESS_GROUP} if os.name == "nt" else {}
    started = time.perf_counter()
    process = subprocess.Popen([str(binary), "--config", str(config)], stdout=logs, stderr=logs, **options)
    return process, started


def sample_stabilized(pid: int) -> dict[str, int]:
    deadline = time.perf_counter() + STABILIZE_SECONDS
    samples = []
    while time.perf_counter() < deadline:
        samples.append(memory_bytes(pid))
        time.sleep(POLL_SECONDS)
    return {key: int(statistics.median(item[key] for item in samples)) for key in samples[0]}


def run_cycle(ipc: int, peer: VehiclePeer, number: int) -> None:
    request(ipc, f"resource-status-{number}", "status")
    admit_source(ipc, number)
    execute_servo(ipc, peer, number, f"resource-servo-{number}")
    hello = request(ipc, f"resource-revoke-hello-{number}", "hello")
    result = request(ipc, f"resource-revoke-{number}", "revoke_authority", **authority_fields(hello, "runtime-smoke"))
    if not result["ok"] or status(ipc)["authority_owner"] is not None:
        raise AssertionError("resource cycle must independently observe authority revocation")


def admit_source(ipc: int, number: int) -> None:
    hello = request(ipc, f"resource-admit-hello-{number}", "hello")
    kind = "admit_authority" if number == 1 else "handback_authority"
    result = request(ipc, f"resource-admit-{number}", kind, **authority_fields(hello, "runtime-smoke"))
    if not result["ok"] or status(ipc)["authority_owner"] != "runtime-smoke":
        raise AssertionError(f"resource cycle {number}: authenticated {kind} did not establish ownership")


def observe_peak(pid: int, stopped: threading.Event, readings: list[dict]) -> None:
    while not stopped.is_set():
        try:
            readings.append(memory_bytes(pid))
        except psutil.NoSuchProcess:
            return
        stopped.wait(POLL_SECONDS)


def measure_operations(process: subprocess.Popen, ipc: int, peer: VehiclePeer, cycles: int) -> dict:
    readings: list[dict] = []
    stopped = threading.Event()
    worker = threading.Thread(target=observe_peak, args=(process.pid, stopped, readings))
    worker.start()
    started = time.perf_counter()
    try:
        session = sample_stabilized(process.pid)
        admit_source(ipc, 1)
        admitted = sample_stabilized(process.pid)
        execute_servo(ipc, peer, 1, "resource-servo-1")
        hello = request(ipc, "resource-revoke-first", "hello")
        if not request(ipc, "resource-revoke-1", "revoke_authority", **authority_fields(hello, "runtime-smoke"))["ok"]:
            raise AssertionError("first resource cycle revoke failed")
        first = sample_stabilized(process.pid)
        checkpoints = [first]
        for number in range(2, cycles + 1):
            if time.perf_counter() - started > 180:
                raise TimeoutError("resource operation interval exceeded 180 seconds")
            run_cycle(ipc, peer, number)
            if number % 50 == 0:
                checkpoints.append(sample_stabilized(process.pid))
        final = sample_stabilized(process.pid)
    finally:
        stopped.set()
        worker.join(timeout=2)
    return {
        "session": session,
        "admitted": admitted,
        "command": first,
        "final": final,
        "growth_checkpoints": checkpoints,
        "cycles": cycles,
        "interval_seconds": time.perf_counter() - started,
        "peak": {key: max(item[key] for item in readings) for key in session},
        "sample_count": len(readings),
    }


def measure_sample(binary: Path, cycles: int) -> dict:
    udp, ipc = free_port(socket.SOCK_DGRAM), free_port(socket.SOCK_STREAM)
    peer = VehiclePeer(udp, 1)
    with tempfile.TemporaryDirectory(prefix="nomad-resources-") as temporary, tempfile.TemporaryFile() as logs:
        config, _ = deployment(Path(temporary), udp, ipc)
        process, started = launch(binary, config, logs)
        readings, stopped = [], threading.Event()
        worker = threading.Thread(target=observe_peak, args=(process.pid, stopped, readings))
        worker.start()
        try:
            wait_for_hello(ipc)
            ipc_seconds = time.perf_counter() - started
            if status(ipc)["vehicle_session_established"]:
                raise AssertionError("IPC-only startup must not require a vehicle")
            idle = sample_stabilized(process.pid)
            peer.start()
            wait_for_session(ipc)
            operations = measure_operations(process, ipc, peer, cycles)
            operations["peak"] = {key: max(item[key] for item in readings) for key in idle}
            return {"ipc_seconds": ipc_seconds, "idle": idle, "operations": operations}
        finally:
            stopped.set()
            worker.join(timeout=2)
            stop_runtime(process)
            peer.stop()


def measure_incarnation(binary: Path, config: Path, ipc: int, logs) -> dict:
    process, started = launch(binary, config, logs)
    try:
        wait_for_hello(ipc)
        ipc_seconds = time.perf_counter() - started
        wait_for_session(ipc)
        vehicle_seconds = time.perf_counter() - started
        state = status(ipc)
        if state["authority_owner"] is not None:
            raise AssertionError("startup/restart must not restore authority")
        return {
            "ipc_seconds": ipc_seconds,
            "vehicle_seconds": vehicle_seconds,
            "incarnation": state["runtime_incarnation"],
        }
    finally:
        stop_runtime(process)


def measure_launch_pair(binary: Path) -> dict:
    udp, ipc = free_port(socket.SOCK_DGRAM), free_port(socket.SOCK_STREAM)
    peer = VehiclePeer(udp, 1)
    peer.start()
    with tempfile.TemporaryDirectory(prefix="nomad-resources-") as temporary:
        config, _ = deployment(Path(temporary), udp, ipc)
        with tempfile.TemporaryFile() as logs:
            try:
                first = measure_incarnation(binary, config, ipc, logs)
                restart = measure_incarnation(binary, config, ipc, logs)
                if first.pop("incarnation") == restart.pop("incarnation"):
                    raise AssertionError("clean process restart must change incarnation")
            finally:
                peer.stop()
    return {"initial": first, "restart": restart}


def collect_runtime(binary: Path) -> tuple[dict, dict]:
    samples = [measure_sample(binary, CYCLES if index == 0 else 30) for index in range(SAMPLES)]
    pairs = [measure_launch_pair(binary) for _ in range(SAMPLES)]
    values = {
        "startup_ipc_max_seconds": max(item["ipc_seconds"] for item in samples),
        "startup_vehicle_max_seconds": max(item["initial"]["vehicle_seconds"] for item in pairs),
        "restart_ipc_max_seconds": max(item["restart"]["ipc_seconds"] for item in pairs),
        "restart_vehicle_max_seconds": max(item["restart"]["vehicle_seconds"] for item in pairs),
    }
    for state in ("idle", "session", "admitted", "command", "peak"):
        values[f"memory_{state}_resident_bytes"] = max(
            (item["idle"] if state == "idle" else item["operations"][state])["resident"] for item in samples
        )
    values["memory_growth_resident_bytes"] = max(
        max(0, item["operations"]["final"]["resident"] - item["operations"]["command"]["resident"]) for item in samples
    )
    detail = {
        "samples": samples,
        "vehicle_ready_launches": pairs,
        "ipc_distribution": distribution([item["ipc_seconds"] for item in samples]),
        "vehicle_distribution": distribution([item["initial"]["vehicle_seconds"] for item in pairs]),
        "restart_ipc_distribution": distribution([item["restart"]["ipc_seconds"] for item in pairs]),
        "restart_vehicle_distribution": distribution([item["restart"]["vehicle_seconds"] for item in pairs]),
        "memory_method": "psutil native Linux RSS / Windows working set; Windows private recorded separately",
        "poll_seconds": POLL_SECONDS,
        "stabilization_seconds": STABILIZE_SECONDS,
        "boundary": "software peer; clean process restart with persistent config and peer; excludes OS recovery delay",
    }
    return values, detail
