# SPDX-License-Identifier: Apache-2.0
"""Private runtime state and outcome assertions shared by qualification scenarios."""

from __future__ import annotations

import json
from pathlib import Path

from mavsdk_peer import COMMAND_DO_SET_SERVO, VehiclePeer
from runtime_fixture_support import FIXTURE_CREDENTIALS, authority_fields, request, require, send_request
from runtime_lifecycle_fixture import wait_for


def status(port: int) -> dict:
    return request(port, "lifecycle-status", "status")["status"]


def wait_for_vehicle(port: int) -> None:
    wait_for(
        lambda: status(port)["vehicle_connected"] and status(port)["telemetry"]["heartbeat_fresh"],
        "vehicle session did not become fresh",
    )


def servo_count(peer: VehiclePeer) -> int:
    return sum(command[1] == COMMAND_DO_SET_SERVO for command in peer.commands())


def admit(port: int) -> None:
    hello = request(port, "lifecycle-hello", "hello")
    response = request(port, "lifecycle-admit", "admit_authority", **authority_fields(hello, "runtime-smoke"))
    require(response["ok"], "fresh authenticated explicit authority admission succeeds")


def servo(port: int, identifier: str) -> dict:
    hello = request(port, "lifecycle-command-context", "hello")
    return {
        "protocol": "nomad-core",
        "version": 1,
        "client_id": "runtime-smoke",
        "id": identifier,
        "type": "set_servo",
        "credential": FIXTURE_CREDENTIALS["runtime-smoke"],
        "channel": 8,
        "pwm_microseconds": 1500,
        **authority_fields(hello, "runtime-smoke"),
    }


def execute_servo(ipc: int, peer: VehiclePeer, count: int, identifier: str) -> dict:
    command = servo(ipc, identifier)
    require(send_request(ipc, command)["command_result"]["success"], "harmless fixture servo command succeeds")
    wait_for(lambda: servo_count(peer) == count, "peer did not observe the expected command count")
    return command


def reject_old_context(port: int, old: dict, peer: VehiclePeer, expected: int) -> None:
    response = send_request(port, old)
    require(response["error"]["code"] == "stale_authority", "old incarnation/generation/request context is rejected")
    fresh = servo(port, "fresh-before-admission")
    require(send_request(port, fresh)["error"]["code"] == "not_authoritative", "reconnect cannot restore ownership")
    unsigned = dict(fresh)
    unsigned.pop("credential")
    require(
        send_request(port, unsigned)["error"]["code"] == "authentication_failed", "fresh authentication is required"
    )
    require(servo_count(peer) == expected, "restart and rejected requests produce no servo command")


def verify_history(directory: Path, incarnations: list[str], clean: bool) -> None:
    histories = {path.stem: path.read_bytes() for path in (directory / "audit").glob("*.jsonl")}
    for incarnation in incarnations:
        content = histories[incarnation]
        require(content.endswith(b"\n"), "incarnation history remains complete JSON Lines")
        records = [json.loads(line) for line in content.splitlines()]
        require(records[0]["event"] == "runtime_start", "new incarnation starts a separate durable journal")
        require(
            all(record["runtime_incarnation"] == incarnation for record in records), "journal incarnation is stable"
        )
    last = [json.loads(line) for line in histories[incarnations[-1]].splitlines()]
    require(last[-1]["event"] == "runtime_shutdown", "clean stop records durable shutdown evidence")
    first = [json.loads(line) for line in histories[incarnations[0]].splitlines()]
    require((first[-1]["event"] == "runtime_shutdown") == clean, "clean/crash exit has the expected shutdown evidence")
