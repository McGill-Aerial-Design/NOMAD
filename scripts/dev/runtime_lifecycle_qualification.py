# SPDX-License-Identifier: Apache-2.0
"""Qualify persistent runtime clean/crash restart against a deterministic UDP peer."""

from __future__ import annotations

import json
import socket
import subprocess
import sys
import tempfile
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

from mavsdk_peer import VehiclePeer
from runtime_connection_lifetime_fixture import verify_connection_lifetime
from runtime_fixture_support import (
    find_runtime,
    free_port,
    require,
    send_request,
    stop_runtime,
)
from runtime_lifecycle_fixture import ProcessSupervisor, deployment, wait_for, write_private_json
from runtime_qualification_support import (
    admit,
    execute_servo,
    reject_old_context,
    servo,
    servo_count,
    status,
    verify_history,
    wait_for_vehicle,
)


def restart_process(
    supervisor: ProcessSupervisor, binary: Path, config: Path, directory: Path, crash: bool
) -> ProcessSupervisor:
    if crash:
        supervisor.child.kill()
        wait_for(lambda: len(supervisor.starts) == 2, "supervisor did not restart crashed runtime")
        return supervisor
    supervisor.stop()
    supervisor.log.close()
    replacement = ProcessSupervisor([str(binary), "--config", str(config)], directory)
    replacement.start()
    return replacement


def verify_link_return(ipc: int, peer: VehiclePeer, udp: int, incarnation: str) -> VehiclePeer:
    peer.stop()
    wait_for(lambda: not status(ipc)["vehicle_connected"], "link loss did not invalidate session")
    require(status(ipc)["runtime_incarnation"] == incarnation, "ordinary link loss does not restart process")
    require(status(ipc)["authority_owner"] is None, "link loss revokes authority")
    replacement = VehiclePeer(udp, 1)
    replacement.start()
    try:
        wait_for_vehicle(ipc)
        require(status(ipc)["authority_owner"] is None, "aircraft/router return does not restore authority")
        require(servo_count(replacement) == 0, "returning aircraft receives no replay")
        return replacement
    except Exception:
        replacement.stop()
        raise


def run_restart(binary: Path, directory: Path, crash: bool) -> None:
    udp, ipc = free_port(socket.SOCK_DGRAM), free_port(socket.SOCK_STREAM)
    config, _ = deployment(directory, udp, ipc)
    supervisor = ProcessSupervisor([str(binary), "--config", str(config)], directory)
    peer = VehiclePeer(udp, 1)
    incarnations = []
    supervisor.start()
    try:
        wait_for(lambda: status(ipc)["runtime_ready"], "IPC not ready before aircraft/router appears")
        require(status(ipc)["lifecycle"] == "degraded", "absent aircraft/router leaves IPC alive and degraded")
        peer.start()
        wait_for_vehicle(ipc)
        incarnations.append(status(ipc)["runtime_incarnation"])
        admit(ipc)
        old = execute_servo(ipc, peer, 1, "before-process-restart")
        original = (directory / "audit" / f"{incarnations[0]}.jsonl").read_bytes()
        supervisor = restart_process(supervisor, binary, config, directory, crash)
        wait_for(lambda: status(ipc)["runtime_incarnation"] != incarnations[0], "incarnation did not change")
        wait_for_vehicle(ipc)
        incarnations.append(status(ipc)["runtime_incarnation"])
        require(status(ipc)["authority_owner"] is None, "restart creates no authority owner")
        require(
            (directory / "audit" / f"{incarnations[0]}.jsonl").read_bytes().startswith(original),
            "prior audit history is preserved byte for byte",
        )
        reject_old_context(ipc, old, peer, 1)
        verify_duplicate(binary, config)
        admit(ipc)
        execute_servo(ipc, peer, 2, "after-process-restart")
        peer = verify_link_return(ipc, peer, udp, incarnations[-1])
    finally:
        supervisor.stop()
        peer.stop()
        supervisor.log.close()
    verify_history(directory, incarnations, clean=not crash)


def verify_unmatched_intent(binary: Path, directory: Path) -> None:
    udp, ipc = free_port(socket.SOCK_DGRAM), free_port(socket.SOCK_STREAM)
    config, _ = deployment(directory, udp, ipc)
    supervisor = ProcessSupervisor([str(binary), "--config", str(config)], directory)
    peer = VehiclePeer(udp, 1, ack_result=None)
    peer.start()
    supervisor.start()
    try:
        wait_for_vehicle(ipc)
        admit(ipc)
        old = servo(ipc, "unmatched-crash-intent")
        incarnation = status(ipc)["runtime_incarnation"]
        journal = directory / "audit" / f"{incarnation}.jsonl"
        with ThreadPoolExecutor(max_workers=1) as worker:
            pending = worker.submit(send_request, ipc, old)
            wait_for(lambda: servo_count(peer) >= 1, "peer did not observe unacknowledged send")
            records = [json.loads(line) for line in journal.read_bytes().splitlines()]
            require(any(record["event"] == "mutation_intent" for record in records), "intent is durable before crash")
            require(not any(record["event"] == "mutation_outcome" for record in records), "crash window has no outcome")
            supervisor.child.kill()
            try:
                pending.result(timeout=10)
            except (ConnectionError, OSError):
                pass
        before = servo_count(peer)
        original = journal.read_bytes()
        wait_for(lambda: len(supervisor.starts) == 2, "unmatched-intent process did not restart")
        wait_for_vehicle(ipc)
        require(status(ipc)["runtime_incarnation"] != incarnation, "unmatched-intent restart changes incarnation")
        reject_old_context(ipc, old, peer, before)
        require(journal.read_bytes() == original, "unmatched prior history stays unchanged/unknown")
    finally:
        supervisor.close()
        peer.stop()


def verify_duplicate(binary: Path, config: Path) -> None:
    result = subprocess.run([str(binary), "--config", str(config)], capture_output=True, timeout=10)
    require(result.returncode == 78, "rapid duplicate startup refuses the deployment audit lock")


def verify_failure(binary: Path, directory: Path, failure: str) -> None:
    udp, ipc = free_port(socket.SOCK_DGRAM), free_port(socket.SOCK_STREAM)
    config, settings = deployment(directory, udp, ipc)
    blocker = None
    if failure == "credentials":
        settings["NOMAD_CLIENT_CREDENTIALS_FILE"] = str(directory / "missing.json")
    elif failure == "audit":
        settings["NOMAD_AUDIT_DIRECTORY"] = str(directory / "missing-parent" / "audit")
    elif failure == "endpoint":
        settings["NOMAD_MAVLINK_ENDPOINT"] = "tcp:127.0.0.1:5760"
    elif failure == "malformed":
        settings["NOMAD_RUNTIME_IPC_PORT"] = "invalid"
    elif failure == "credential-json":
        Path(settings["NOMAD_CLIENT_CREDENTIALS_FILE"]).write_text("{}", encoding="utf-8")
    elif failure == "bind":
        blocker = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        blocker.bind(("127.0.0.1", ipc))
        blocker.listen()
    config.unlink()
    write_private_json(config, settings)
    peer = VehiclePeer(udp, 1)
    supervisor = ProcessSupervisor([str(binary), "--config", str(config)], directory)
    peer.start()
    supervisor.start()
    try:
        wait_for(lambda: len(supervisor.exits) == 1, f"{failure} did not fail startup")
        require(supervisor.exits == [78], f"{failure} is a permanent startup failure")
        supervisor.worker.join(timeout=2)
        require(
            not supervisor.worker.is_alive() and len(supervisor.starts) == 1, "permanent failure is not restart-looped"
        )
        require(not peer.commands(), "startup failure causes no vehicle mutation")
    finally:
        supervisor.close()
        peer.stop()
        if blocker:
            blocker.close()


def verify_backoff(directory: Path) -> None:
    supervisor = ProcessSupervisor([sys.executable, "-c", "raise SystemExit(1)"], directory)
    supervisor.start()
    try:
        supervisor.worker.join(timeout=5)
        require(
            not supervisor.worker.is_alive() and len(supervisor.starts) == 3,
            "fast crashes stop after bounded fixture retries",
        )
        intervals = [b - a for a, b in zip(supervisor.starts, supervisor.starts[1:], strict=False)]
        require(intervals[0] >= 0.1 and intervals[1] >= 0.2, "fixture retries respect increasing backoff")
    finally:
        supervisor.close()


def verify_starting_stop(binary: Path, directory: Path) -> None:
    udp, ipc = free_port(socket.SOCK_DGRAM), free_port(socket.SOCK_STREAM)
    config, _ = deployment(directory, udp, ipc)
    supervisor = ProcessSupervisor([str(binary), "--config", str(config)], directory)
    supervisor.start()
    try:
        wait_for(lambda: any((directory / "audit").glob("*.jsonl")), "startup journal did not appear")
        stop_runtime(supervisor.child)
        require(supervisor.child.returncode == 0, "stop observed during startup drains cleanly without a vehicle")
        histories = list((directory / "audit").glob("*.jsonl"))
        records = [json.loads(line) for line in histories[0].read_bytes().splitlines()]
        require(records[-1]["event"] == "runtime_shutdown", "startup stop records a complete shutdown boundary")
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as probe:
            probe.bind(("127.0.0.1", ipc))
        require(True, "startup stop releases IPC binding")
    finally:
        supervisor.close()


def verify_configuration_parser(binary: Path, directory: Path) -> None:
    config, settings = deployment(directory, free_port(socket.SOCK_DGRAM), free_port(socket.SOCK_STREAM))
    invalid = [
        None,
        {},
        {**settings, "NOMAD_API_KEY": 1},
        {**settings, "UNKNOWN": "value"},
        {**settings, "NOMAD_AUDIT_DIRECTORY": "relative/audit"},
    ]
    texts = [json.dumps(value) for value in invalid]
    texts.extend(['{"NOMAD_API_KEY":"","NOMAD_API_KEY":"duplicate"}', '{"unfinished":'])
    for text in texts:
        config.write_text(text, encoding="utf-8")
        result = subprocess.run([str(binary), "--config", str(config)], capture_output=True, timeout=10)
        require(
            result.returncode == 78, "malformed/duplicate/unknown/missing/relative service config fails permanently"
        )


def main() -> int:
    binary = find_runtime()
    verify_connection_lifetime(binary)
    with tempfile.TemporaryDirectory(prefix="nomad-lifecycle-") as temporary:
        root = Path(temporary)
        for crash in (False, True):
            directory = root / ("crash" if crash else "clean")
            directory.mkdir()
            run_restart(binary, directory, crash)
        for failure in ("credentials", "credential-json", "audit", "endpoint", "malformed", "bind"):
            directory = root / failure
            directory.mkdir()
            verify_failure(binary, directory, failure)
        directory = root / "unmatched-intent"
        directory.mkdir()
        verify_unmatched_intent(binary, directory)
        directory = root / "starting-stop"
        directory.mkdir()
        verify_starting_stop(binary, directory)
        directory = root / "parser"
        directory.mkdir()
        verify_configuration_parser(binary, directory)
        verify_backoff(root)
    print("software-only runtime lifecycle qualification passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
