# SPDX-License-Identifier: Apache-2.0
"""Compare real runtime disposition, durable evidence and deterministic vehicle wire."""

from __future__ import annotations

import json
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from typing import Any

from mavsdk_authority_peer import AuthorityPeer
from mavsdk_peer import (
    ACCEPTED,
    COMMAND_DO_MOTOR_TEST,
    COMMAND_DO_MOUNT_CONFIGURE,
    COMMAND_DO_MOUNT_CONTROL,
    COMMAND_DO_SET_RELAY,
    COMMAND_DO_SET_SERVO,
    DENIED,
)
from runtime_ipc_smoke import FIXTURE_CREDENTIALS, authority_fields, request, require, send_request


def mutation(port: int, identifier: str, operation: str = "set_servo", **fields: object) -> dict[str, Any]:
    """Build one fresh authenticated mutation without sending it."""
    hello = request(port, "outcome-context", "hello")
    arguments = fields or {"channel": 8, "pwm_microseconds": 1500}
    return {
        "protocol": "nomad-core",
        "version": 1,
        "client_id": "runtime-smoke",
        "credential": FIXTURE_CREDENTIALS["runtime-smoke"],
        "id": identifier,
        "type": operation,
        **authority_fields(hello, "runtime-smoke"),
        **arguments,
    }


def command_count(peer: AuthorityPeer, command_id: int = COMMAND_DO_SET_SERVO) -> int:
    """Count authoritative wire observations for one command."""
    return sum(command[1] == command_id for command in peer.commands())


def wait_for_wire(peer: AuthorityPeer, previous: int) -> None:
    """Wait for actual delivery before triggering authority interruption."""
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        if command_count(peer) > previous:
            return
        time.sleep(0.02)
    raise TimeoutError("peer did not observe the pending servo command")


def require_journal_outcome(
    directory: Path, identifier: str, response: dict, expected: str, event: str = "mutation_outcome"
) -> None:
    """Compare normalized response classification with the committed journal result."""
    records = [json.loads(line) for path in directory.glob("*.jsonl") for line in path.read_bytes().splitlines()]
    outcomes = [record for record in records if record.get("request_id") == identifier and record["event"] == event]
    require(response.get("outcome") == expected, f"{identifier}: runtime outcome is {expected}")
    require(len(outcomes) == 1, f"{identifier}: exactly one durable operation outcome")
    require(outcomes[0]["result"] == response["outcome"], f"{identifier}: audit and response agree")
    result = response.get("command_result")
    if result is not None:
        require(outcomes[0]["acknowledged"] == result["acknowledged"], f"{identifier}: ACK evidence agrees")


def transfer_authority(port: int) -> None:
    """Move the existing smoke source back to the outcome fixture source."""
    hello = request(port, "outcome-revoke-context", "hello")
    revoked = request(port, "outcome-revoke", "revoke_authority", **authority_fields(hello, "runtime-smoke"))
    require(revoked["ok"], "outcome fixture explicitly revokes the previous source")
    hello = request(port, "outcome-handback-context", "hello")
    admitted = request(port, "outcome-handback", "handback_authority", **authority_fields(hello, "runtime-smoke"))
    require(admitted["ok"], "outcome fixture explicitly admits its authenticated source")
    require("outcome" not in admitted, "authority success retains its state-transition response contract")


def verify_successes(port: int, peer: AuthorityPeer, directory: Path) -> None:
    """Exercise every supported mutation with positive FC protocol evidence."""
    operations = (
        ("set_servo", COMMAND_DO_SET_SERVO, {"channel": 8, "pwm_microseconds": 1500}),
        ("set_relay", COMMAND_DO_SET_RELAY, {"relay_number": 0, "on": True}),
        ("motor_test", COMMAND_DO_MOTOR_TEST, {"motor_instance": 1, "pwm_microseconds": 0, "timeout_seconds": 0.1}),
        ("configure_gimbal", COMMAND_DO_MOUNT_CONFIGURE, {"mount_mode": 2}),
        ("set_gimbal_target", COMMAND_DO_MOUNT_CONTROL, {"pitch_deg": 10.0, "roll_deg": 0.0}),
    )
    for operation, command_id, fields in operations:
        before = command_count(peer, command_id)
        identifier = "outcome-success-" + operation
        response = send_request(port, mutation(port, identifier, operation, **fields))
        require_journal_outcome(directory, identifier, response, "success")
        require(response["command_result"]["success"], f"{operation}: software success remains true")
        require(response["command_result"]["acknowledged"], f"{operation}: FC acknowledgement remains explicit")
        require(command_count(peer, command_id) == before + 1, f"{operation}: one wire delivery")


def verify_rejections(port: int, peer: AuthorityPeer, directory: Path) -> None:
    """Prove invalid context, expiry and consumed sequence cannot deliver commands."""
    before = command_count(peer)
    stale = mutation(port, "outcome-stale-authority")
    stale["authority_generation"] -= 1
    expired = mutation(port, "outcome-expired")
    expired["expires_at_ms"] = 1
    invalid = mutation(port, "outcome-invalid")
    invalid["channel"] = "invalid"
    for message, code in ((stale, "stale_authority"), (expired, "expired_request"), (invalid, "invalid_request")):
        response = send_request(port, message)
        require(response["error"]["code"] == code, f"{message['id']}: expected rejection reason")
        require_journal_outcome(directory, message["id"], response, "rejected", "request_rejected")
    require(command_count(peer) == before, "stale, expired and malformed requests have zero wire deliveries")

    consumed = mutation(port, "outcome-consumed-sequence", channel=0, pwm_microseconds=1500)
    response = send_request(port, consumed)
    require_journal_outcome(directory, consumed["id"], response, "rejected")
    replay = dict(consumed, id="outcome-sequence-replay", channel=8)
    response = send_request(port, replay)
    require(response["error"]["code"] == "stale_request", "consumed sequence cannot become replayable")
    require_journal_outcome(directory, replay["id"], response, "rejected", "request_rejected")
    require(command_count(peer) == before, "invalid value and stale sequence have zero wire deliveries")


def verify_failed_ack(port: int, peer: AuthorityPeer, directory: Path) -> None:
    """A real negative FC acknowledgement means attempted and failed."""
    before = command_count(peer)
    peer.set_ack_result(DENIED)
    try:
        message = mutation(port, "outcome-negative-ack")
        response = send_request(port, message)
        require_journal_outcome(directory, message["id"], response, "failed")
        require(not response["command_result"]["success"], "negative ACK is unsuccessful")
        require(response["command_result"]["acknowledged"], "negative ACK preserves acknowledgement")
        require(command_count(peer) == before + 1, "negative ACK follows one actual vehicle delivery")
    finally:
        peer.set_ack_result(ACCEPTED)


def verify_unknown_cache(port: int, peer: AuthorityPeer, directory: Path) -> None:
    """Timeout after wire delivery remains unknown on exact cached retry."""
    before = command_count(peer)
    peer.set_ack_result(None)
    try:
        message = mutation(port, "outcome-withheld-ack")
        response = send_request(port, message)
        require_journal_outcome(directory, message["id"], response, "unknown")
        require(not response["command_result"]["acknowledged"], "withheld ACK does not fabricate acknowledgement")
        require(command_count(peer) > before, "unknown timeout has actual vehicle delivery evidence")
        after = command_count(peer)
        require(send_request(port, message) == response, "cached unknown response is preserved exactly")
        require(command_count(peer) == after, "cached unknown request produces no new wire delivery")
    finally:
        peer.set_ack_result(ACCEPTED)


def verify_interruption(port: int, peer: AuthorityPeer, directory: Path) -> None:
    """Revoke only after vehicle delivery while the command ACK is withheld."""
    before = command_count(peer)
    peer.set_ack_result(None)
    try:
        message = mutation(port, "outcome-authority-interrupted")
        with ThreadPoolExecutor(max_workers=1) as worker:
            pending = worker.submit(send_request, port, message)
            wait_for_wire(peer, before)
            hello = request(port, "outcome-interruption-context", "hello", client_id="operator")
            revoked = request(
                port,
                "outcome-interruption-revoke",
                "revoke_authority",
                client_id="operator",
                **authority_fields(hello, "operator"),
            )
            require(revoked["ok"], "authority changes after actual mutation wire delivery")
            response = pending.result(timeout=10)
        require(
            response["error"]["code"] == "authority_interrupted", "pending operation reports authority interruption"
        )
        require_journal_outcome(directory, message["id"], response, "interrupted")
        after = command_count(peer)
        retry = send_request(port, message)
        require(retry["error"]["code"] == "stale_authority", "old generation is checked before cached response")
        require(retry.get("outcome") == "rejected", "new stale-context attempt is definitely not sent")
        require(command_count(peer) == after, "interrupted request is never replayed after authority changes")
    finally:
        peer.set_ack_result(ACCEPTED)


def verify_outcomes(port: int, peer: AuthorityPeer, directory: Path) -> None:
    """Run the truthful outcome matrix without hardware or aircraft state assumptions."""
    transfer_authority(port)
    verify_successes(port, peer, directory)
    verify_rejections(port, peer, directory)
    verify_failed_ack(port, peer, directory)
    verify_unknown_cache(port, peer, directory)
    verify_interruption(port, peer, directory)
