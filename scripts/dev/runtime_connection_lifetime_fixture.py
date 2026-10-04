# SPDX-License-Identifier: Apache-2.0
"""Keep real IPC readers active across MAVSDK resource publication and retirement."""

from __future__ import annotations

import socket
import threading
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

from mavsdk_peer import COMMAND_DO_SET_SERVO, VehiclePeer
from runtime_fixture_support import (
    authority_fields,
    free_port,
    request,
    require,
    start_runtime,
    stop_runtime,
    wait_for_listener,
)
from runtime_lifecycle_fixture import wait_for


class ConcurrentReaders:
    """Propagate every reader failure and require progress from every reader."""

    def __init__(self, port: int):
        self.port = port
        self.stop = threading.Event()
        self.shutdown = threading.Event()
        self.lock = threading.Lock()
        self.counts = [0] * 4
        self.pool = ThreadPoolExecutor(max_workers=len(self.counts))
        self.workers = [self.pool.submit(self.read, index) for index in range(len(self.counts))]

    def read(self, index: int) -> None:
        sequence = 0
        while not self.stop.is_set():
            try:
                hello = request(self.port, f"lifetime-{index}-{sequence}-hello", "hello")
                response = request(self.port, f"lifetime-{index}-{sequence}-status", "status")
            except (OSError, ConnectionError):
                if self.shutdown.is_set():
                    return
                raise
            if not hello["ok"] or hello["version"] != 1 or not response["ok"]:
                raise AssertionError("concurrent HELLO/STATUS must return valid protocol responses")
            status = response["status"]
            if not isinstance(status["vehicle_connected"], bool) or not isinstance(status["identity_resolved"], bool):
                raise AssertionError("concurrent STATUS must return complete readiness fields")
            with self.lock:
                self.counts[index] += 1
            sequence += 1

    def checkpoint(self, phase: str) -> None:
        with self.lock:
            targets = [count + 20 for count in self.counts]

        def all_readers_progressed() -> bool:
            for worker in self.workers:
                if worker.done():
                    worker.result()
                    raise AssertionError("IPC reader stopped before shutdown")
            with self.lock:
                return all(count >= target for count, target in zip(self.counts, targets, strict=True))

        wait_for(all_readers_progressed, f"concurrent IPC readers stalled during {phase}")
        require(True, f"every IPC reader completed 20 HELLO/STATUS pairs during {phase}")

    def close(self) -> None:
        self.stop.set()
        try:
            for worker in self.workers:
                worker.result(timeout=12)
        finally:
            self.pool.shutdown(wait=True)


def status(port: int) -> dict:
    return request(port, "lifetime-observe", "status")["status"]


def wait_for_vehicle(port: int) -> None:
    def ready() -> bool:
        observed = status(port)
        return (
            observed["vehicle_connected"] and observed["identity_resolved"] and observed["telemetry"]["heartbeat_fresh"]
        )

    wait_for(ready, "returning vehicle did not publish a fresh resolved session")


def verify_new_session(port: int, hello: dict, context: dict[str, object]) -> None:
    """Verify a returning connection has fresh context and rejects the retired owner."""
    wait_for_vehicle(port)
    returned = request(port, "lifetime-after-return", "hello")
    require(returned["runtime_incarnation"] == hello["runtime_incarnation"], "link return preserves runtime process")
    require(returned["authority"]["vehicle_session"] > context["vehicle_session"], "link return creates a new session")
    require(status(port)["authority_owner"] is None, "new resource generation does not restore authority")
    rejected = request(
        port,
        f"lifetime-stale-{context['vehicle_session']}",
        "set_servo",
        channel=8,
        pwm_microseconds=1500,
        **context,
    )
    require(rejected["error"]["code"] == "stale_authority", "retired session cannot send after reconnect")


def retire_and_return(
    port: int, udp: int, peer: VehiclePeer, readers: ConcurrentReaders, *, handback: bool
) -> VehiclePeer:
    """Observe real link loss, then reject the retired command context after return."""
    hello = request(port, "lifetime-before-loss", "hello")
    context = authority_fields(hello, "runtime-smoke")
    operation = "handback_authority" if handback else "admit_authority"
    admitted = request(port, f"lifetime-admit-{context['vehicle_session']}", operation, **context)
    require(admitted["ok"], "lifetime fixture admits authority before link loss")
    hello = request(port, "lifetime-admitted-context", "hello")
    context = authority_fields(hello, "runtime-smoke")
    peer.stop()

    def retired() -> bool:
        observed = status(port)
        return not observed["vehicle_connected"] and not observed["mavsdk_connection_open"]

    wait_for(retired, "link loss did not retire connection")
    require(status(port)["authority_owner"] is None, "retirement revokes authority while IPC remains alive")
    readers.checkpoint("retired resources")
    replacement = VehiclePeer(udp, 1)
    replacement.start()
    try:
        verify_new_session(port, hello, context)
        readers.checkpoint("reconnected resources")
        require(
            not any(command[1] == COMMAND_DO_SET_SERVO for command in replacement.commands()),
            "retired command context delivers no servo command to the returning peer",
        )
        return replacement
    except Exception:
        replacement.stop()
        raise


def verify_connection_lifetime(binary: Path) -> None:
    """Span discovery, repeated connection generations and shutdown with active readers."""
    udp, ipc = free_port(socket.SOCK_DGRAM), free_port(socket.SOCK_STREAM)
    process, stdout, stderr = start_runtime(binary, udp, ipc)
    readers = None
    peer = None
    try:
        wait_for_listener(ipc)
        readers = ConcurrentReaders(ipc)
        require(status(ipc)["vehicle_connected"] is False, "IPC is available before initial MAVSDK discovery")
        readers.checkpoint("initial discovery without a peer")
        peer = VehiclePeer(udp, 1)
        peer.start()
        wait_for_vehicle(ipc)
        readers.checkpoint("initial published resources")
        for cycle in range(2):
            peer = retire_and_return(ipc, udp, peer, readers, handback=cycle != 0)
        readers.checkpoint("before orderly shutdown")
        readers.shutdown.set()
        stop_runtime(process)
        readers.close()
        readers = None
        require(process.returncode == 0, "runtime drains resource teardown with active HELLO/STATUS readers")
    finally:
        try:
            if readers is not None:
                readers.shutdown.set()
                readers.close()
        finally:
            try:
                stop_runtime(process)
            finally:
                if peer is not None:
                    peer.stop()
                stdout.close()
                stderr.close()
