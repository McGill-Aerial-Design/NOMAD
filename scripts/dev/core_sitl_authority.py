# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Exercise runtime authority against isolated, disarmed Copter SITL."""

from __future__ import annotations

import json
import socket
import sys
import time
from datetime import datetime, timezone
from typing import Any

from authority_sitl_observer import (
    OBSERVER_PORT,
    SERVO_FUNCTION,
    AuthorityObserver,
    ProbeError,
    summary,
    verify_isolated_simulator,
)
from authority_sitl_relay import AuthorityRelay, runtime_source_id
from mavsdk_authority_wire_fixture import SOURCE, bound_request, current_context
from pymavlink import mavutil
from runtime_fixture_support import find_runtime, free_port, request, start_runtime, stop_runtime, wait_for_listener

CHANNEL = 5
SET_SERVO = mavutil.mavlink.MAV_CMD_DO_SET_SERVO
RETRY_WINDOW = 2.0


def wait_frames(relay: AuthorityRelay, count: int, timeout: float = 8.0) -> list[tuple[float, int, int, int]]:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if len(relay.frames) >= count:
            return list(relay.frames)
        time.sleep(0.02)
    raise ProbeError("runtime_servo_wire_frame_missing")


def wait_status(
    port: int,
    previous_session: int | None = None,
    timeout: float = 25.0,
    observer: AuthorityObserver | None = None,
) -> dict[str, Any]:
    deadline, index = time.monotonic() + timeout, 0
    while time.monotonic() < deadline:
        if observer is not None:
            observer.pump(0.0)
        status = request(port, f"authority-ready-{index}", "status")["status"]
        index += 1
        session = int(status["vehicle_session"])
        if status["vehicle_connected"] and status["aircraft_class"] == "Copter" and session != previous_session:
            return status
        time.sleep(0.1)
    raise ProbeError("runtime_copter_session_not_ready")


def wait_revoked(port: int, generation: int, observer: AuthorityObserver, timeout: float = 15.0) -> dict[str, Any]:
    deadline, index = time.monotonic() + timeout, 0
    while time.monotonic() < deadline:
        observer.pump(0.0)
        status = request(port, f"authority-revoked-{index}", "status")["status"]
        index += 1
        if status["authority_owner"] is None and int(status["authority_generation"]) != generation:
            return status
        time.sleep(0.1)
    raise ProbeError("runtime_authority_loss_not_observed")


def stale_servo(port: int, request_id: str, context: dict[str, object]) -> dict[str, Any]:
    return request(
        port,
        request_id,
        "set_servo",
        command_source=SOURCE,
        sequence=1,
        expires_at_ms=int(time.time() * 1000) + 3000,
        channel=CHANNEL,
        pwm_microseconds=1400,
        **context,
    )


def require_error(response: dict[str, Any], code: str) -> None:
    if response.get("error", {}).get("code") != code:
        raise ProbeError("runtime_ipc_boundary_failed")


def command_servo(
    port: int,
    observer: AuthorityObserver,
    relay: AuthorityRelay,
    name: str,
    sequence: int,
    pwm: int,
    evidence: list[dict[str, Any]],
) -> None:
    observer.wait_vehicle()
    before, boundary = len(relay.frames), time.monotonic()
    result = bound_request(port, name, "set_servo", sequence, channel=CHANNEL, pwm_microseconds=pwm)
    if result.get("ok") is not True or result.get("command_result", {}).get("success") is not True:
        raise ProbeError("runtime_servo_command_failed")
    frames = wait_frames(relay, before + 1)
    observed = observer.wait_servo5(pwm, boundary)
    evidence.append(
        {
            "kind": "set_servo",
            "channel": CHANNEL,
            "requested_pwm": pwm,
            "observed_pwm": observed,
            "wire_attempts": len(frames) - before,
        }
    )


def startup_denied(port: int, observer: AuthorityObserver, relay: AuthorityRelay, baseline: int, test_pwm: int) -> None:
    if request(port, "authority-startup-status", "status")["status"]["authority_owner"] is not None:
        raise ProbeError("startup_owner_present")
    before, boundary = len(relay.frames), time.monotonic()
    denied = bound_request(port, "authority-startup-denied", "set_servo", 1, channel=CHANNEL, pwm_microseconds=test_pwm)
    require_error(denied, "not_authoritative")
    observer.wait_servo5(baseline, boundary)
    if len(relay.frames) != before:
        raise ProbeError("startup_mutation_reached_wire")


def revoke_retry(
    port: int, observer: AuthorityObserver, relay: AuthorityRelay, evidence: list[dict[str, Any]], retry_pwm: int
) -> dict[str, object]:
    context = current_context(port)
    before, ack_count = len(relay.frames), relay.dropped_acks
    relay.drop_acks = True
    from concurrent.futures import ThreadPoolExecutor

    with ThreadPoolExecutor(max_workers=1) as executor:
        pending = executor.submit(
            bound_request, port, "authority-retry-servo", "set_servo", 3, channel=CHANNEL, pwm_microseconds=retry_pwm
        )
        wait_retry_attempts(relay, before, ack_count)
        revoked = bound_request(port, "authority-revoke-during-retry", "revoke_authority", 4)
        if revoked.get("ok") is not True:
            raise ProbeError("runtime_revoke_failed")
        fence_count = len(relay.frames)
        response = pending.result(timeout=8.0)
    relay.drop_acks = False
    require_error(response, "authority_interrupted")
    require_error(stale_servo(port, "authority-retry-old-context", context), "stale_authority")
    require_error(
        bound_request(
            port, "authority-retry-fresh-denied", "set_servo", 1, channel=CHANNEL, pwm_microseconds=retry_pwm
        ),
        "not_authoritative",
    )
    time.sleep(RETRY_WINDOW)
    require_fence_count(fence_count, len(relay.frames))
    evidence.append(
        {
            "kind": "set_servo",
            "channel": CHANNEL,
            "requested_pwm": retry_pwm,
            "wire_attempts": fence_count - before,
            "acknowledgements_dropped": relay.dropped_acks - ack_count,
        }
    )
    return context


def require_fence_count(at_revoke: int, observed_after_window: int) -> None:
    if observed_after_window != at_revoke:
        raise ProbeError("revoked_command_retried_after_fence")


def wait_retry_attempts(relay: AuthorityRelay, frame_count: int, ack_count: int) -> None:
    deadline = time.monotonic() + 8.0
    while time.monotonic() < deadline:
        if len(relay.frames) >= frame_count + 2 and relay.dropped_acks >= ack_count + 1:
            return
        time.sleep(0.02)
    raise ProbeError("servo_ack_drop_did_not_force_retry")


def external_modes(port: int, observer: AuthorityObserver, owner_required: bool) -> list[int]:
    modes = []
    initial = request(port, "authority-external-mode-owner", "status")["status"]
    if (initial["authority_owner"] is not None) != owner_required:
        raise ProbeError("external_mode_test_wrong_authority_state")
    for mode in (4, 0):
        observer.wait_vehicle()
        boundary = time.monotonic()
        observer.connection.mav.set_mode_send(
            observer.connection.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode
        )
        observer.wait_mode(mode, boundary)
        current = request(port, f"authority-external-mode-{mode}", "status")["status"]
        if current["authority_owner"] != initial["authority_owner"]:
            raise ProbeError("external_mode_changed_runtime_owner")
        if current["authority_generation"] != initial["authority_generation"]:
            raise ProbeError("external_mode_changed_runtime_generation")
        modes.append(mode)
    return modes


def session_rollover(
    port: int,
    relay: AuthorityRelay,
    observer: AuthorityObserver,
    old_context: dict[str, object],
    baseline: int,
    test_pwm: int,
    readbacks: dict[str, Any],
    evidence: list[dict[str, Any]],
) -> None:
    old_session, old_generation = int(old_context["vehicle_session"]), int(old_context["authority_generation"])
    relay.pause()
    wait_revoked(port, old_generation, observer)
    relay.resume()
    recovered = wait_status(port, old_session, observer=observer)
    if int(recovered["vehicle_session"]) == old_session or recovered["authority_owner"] is not None:
        raise ProbeError("session_rollover_did_not_inhibit")
    verify_servo_disabled(observer, readbacks, "link_recovery")
    before = len(relay.frames)
    require_error(stale_servo(port, "authority-old-session", old_context), "stale_authority")
    require_error(
        bound_request(
            port, "authority-recovered-fresh-denied", "set_servo", 1, channel=CHANNEL, pwm_microseconds=test_pwm
        ),
        "not_authoritative",
    )
    if len(relay.frames) != before:
        raise ProbeError("session_stale_command_reached_wire")
    if bound_request(port, "authority-session-handback", "handback_authority", 1).get("ok") is not True:
        raise ProbeError("session_handback_failed")
    command_servo(port, observer, relay, "authority-after-link", 1, test_pwm, evidence)
    command_servo(port, observer, relay, "authority-restore-link", 2, baseline, evidence)


class RuntimeProcess:
    """Own both runtime incarnations and close their captured process streams."""

    def __init__(self) -> None:
        self.udp_port = free_port(socket.SOCK_DGRAM)
        self.ipc_port = free_port(socket.SOCK_STREAM)
        self.process = None
        self.stdout = None
        self.stderr = None

    def start(self) -> None:
        self.process, self.stdout, self.stderr = start_runtime(find_runtime(), self.udp_port, self.ipc_port)
        wait_for_listener(self.ipc_port)

    def stop(self) -> None:
        try:
            if self.process is not None:
                stop_runtime(self.process)
                if self.process.returncode != 0:
                    raise ProbeError("runtime_shutdown_failed")
        finally:
            for stream in (self.stdout, self.stderr):
                if stream is not None:
                    stream.close()


def verify_servo_disabled(observer: AuthorityObserver, readbacks: dict[str, Any], checkpoint: str) -> None:
    value = observer.read_parameter(SERVO_FUNCTION)
    if value != 0.0:
        raise ProbeError("servo5_is_not_disabled_function")
    readbacks[checkpoint] = {SERVO_FUNCTION: value}


def startup_parameters(observer: AuthorityObserver, readbacks: dict[str, Any]) -> tuple[dict[str, float], int]:
    parameters = observer.read_parameters()
    if parameters[SERVO_FUNCTION] != 0.0:
        raise ProbeError("servo5_is_not_disabled_function")
    readbacks["startup"] = parameters
    baseline = observer.wait_servo5(None, time.monotonic() - 2.0)
    if baseline != 0 and not 800 <= baseline <= 2200:
        raise ProbeError("servo5_baseline_out_of_range")
    return parameters, baseline


def exercise_startup(
    port: int,
    observer: AuthorityObserver,
    relay: AuthorityRelay,
    initial_pwm: int,
    neutral_pwm: int,
    test_pwm: int,
    retry_pwm: int,
    checks: dict[str, bool],
    commands: list[dict[str, Any]],
) -> None:
    ready = wait_status(port, observer=observer)
    if ready["authority_owner"] is not None:
        raise ProbeError("startup_owner_present")
    startup_denied(port, observer, relay, initial_pwm, test_pwm)
    checks["startup_inhibited_without_wire_command"] = True
    if bound_request(port, "authority-initial-admit", "admit_authority", 1).get("ok") is not True:
        raise ProbeError("initial_admission_failed")
    command_servo(port, observer, relay, "authority-initial-servo", 1, test_pwm, commands)
    command_servo(port, observer, relay, "authority-initial-neutral", 2, neutral_pwm, commands)
    checks["admitted_servo_observed_in_fc_output"] = True
    checks["external_modes_changed_while_admitted"] = external_modes(port, observer, True) == [4, 0]
    revoke_retry(port, observer, relay, commands, retry_pwm)
    checks["dropped_ack_retry_stopped_after_revoke"] = True
    checks["external_modes_changed_while_revoked"] = external_modes(port, observer, False) == [4, 0]


def exercise_link(
    port: int,
    relay: AuthorityRelay,
    observer: AuthorityObserver,
    baseline: int,
    test_pwm: int,
    readbacks: dict[str, Any],
    checks: dict[str, bool],
    commands: list[dict[str, Any]],
) -> None:
    if bound_request(port, "authority-link-handback", "handback_authority", 1).get("ok") is not True:
        raise ProbeError("pre_link_handback_failed")
    command_servo(port, observer, relay, "authority-pre-link-restore", 1, baseline, commands)
    context = current_context(port)
    session_rollover(port, relay, observer, context, baseline, test_pwm, readbacks, commands)
    checks["link_loss_changed_session_and_required_handback"] = True


def restart_runtime(
    runtime: RuntimeProcess,
    observer: AuthorityObserver,
    relay: AuthorityRelay,
    baseline: int,
    test_pwm: int,
    readbacks: dict[str, Any],
    checks: dict[str, bool],
    commands: list[dict[str, Any]],
) -> None:
    ipc_port = runtime.ipc_port
    old_context = current_context(ipc_port)
    runtime.stop()
    runtime.start()
    restarted = wait_status(ipc_port, observer=observer)
    new_context = current_context(ipc_port)
    if new_context["runtime_incarnation"] == old_context["runtime_incarnation"]:
        raise ProbeError("runtime_incarnation_did_not_change")
    if restarted["authority_owner"] is not None:
        raise ProbeError("restart_owner_present")
    verify_servo_disabled(observer, readbacks, "restart")
    before = len(relay.frames)
    require_error(stale_servo(ipc_port, "authority-old-runtime", old_context), "stale_authority")
    denied = bound_request(
        ipc_port, "authority-restart-fresh-denied", "set_servo", 1, channel=CHANNEL, pwm_microseconds=test_pwm
    )
    require_error(denied, "not_authoritative")
    if len(relay.frames) != before:
        raise ProbeError("restart_stale_command_reached_wire")
    checks["restart_rejected_old_context_and_required_admission"] = True
    if bound_request(ipc_port, "authority-restart-admit", "admit_authority", 1).get("ok") is not True:
        raise ProbeError("restart_admission_failed")
    command_servo(ipc_port, observer, relay, "authority-after-restart", 1, test_pwm, commands)
    command_servo(ipc_port, observer, relay, "authority-final-restore", 2, baseline, commands)
    checks["explicit_admission_restored_runtime_command"] = True


def exercise_scenario(
    runtime: RuntimeProcess,
    observer: AuthorityObserver,
    relay: AuthorityRelay,
    initial_pwm: int,
    readbacks: dict[str, Any],
    checks: dict[str, bool],
    commands: list[dict[str, Any]],
) -> None:
    neutral_pwm = 1500 if initial_pwm == 0 else initial_pwm
    test_pwm = 1400 if neutral_pwm != 1400 else 1600
    retry_pwm = 1350 if neutral_pwm != 1350 else 1450
    runtime.start()
    try:
        exercise_startup(
            runtime.ipc_port, observer, relay, initial_pwm, neutral_pwm, test_pwm, retry_pwm, checks, commands
        )
    except ProbeError as error:
        raise ProbeError(error.category, "startup_admission_revoke") from error
    try:
        exercise_link(runtime.ipc_port, relay, observer, neutral_pwm, test_pwm, readbacks, checks, commands)
    except ProbeError as error:
        raise ProbeError(error.category, "link_recovery_handback") from error
    try:
        restart_runtime(runtime, observer, relay, neutral_pwm, test_pwm, readbacks, checks, commands)
    except ProbeError as error:
        raise ProbeError(error.category, "runtime_restart") from error


def run_probe() -> dict[str, Any]:
    started = datetime.now(timezone.utc).isoformat(timespec="seconds")
    checks, commands, readbacks = {}, [], {}
    failure = phase = None
    relay = observer = None
    runtime_source = initial_pwm = None
    runtime = RuntimeProcess()
    try:
        phase = "container_guard"
        verify_isolated_simulator()
        relay = AuthorityRelay(runtime.udp_port)
        connection = mavutil.mavlink_connection(
            f"udpin:0.0.0.0:{OBSERVER_PORT}", source_system=250, source_component=190
        )
        observer = AuthorityObserver(connection)
        observer.wait_vehicle()
        _parameters, initial_pwm = startup_parameters(observer, readbacks)
        readbacks["initial_output"] = {"servo5_raw": initial_pwm}
        phase = "runtime_startup"
        exercise_scenario(runtime, observer, relay, initial_pwm, readbacks, checks, commands)
        runtime_source = runtime_source_id(relay)
        checks["runtime_command_source_observed"] = True
    except ProbeError as error:
        failure, phase = error.category, error.phase or phase
    except Exception:
        failure = "probe_execution_failed"
    finally:
        failure = close_resources(runtime, observer, relay, failure, initial_pwm, commands)
    transitions = observer.transitions if observer is not None else []
    return summary(
        "failed" if failure else "passed",
        started,
        checks,
        transitions,
        commands,
        readbacks,
        runtime_source,
        failure,
        phase if failure else None,
    )


def restore_simulator_output(observer: AuthorityObserver, initial_pwm: int, commands: list[dict[str, Any]]) -> None:
    """Restore the disabled simulator output through the isolated test GCS."""
    observer.wait_vehicle()
    if observer.read_parameter(SERVO_FUNCTION) != 0.0:
        raise ProbeError("output_restoration_function_changed")
    observer.require_disarmed()
    boundary = time.monotonic()
    observer.connection.mav.command_long_send(1, 1, SET_SERVO, 0, CHANNEL, initial_pwm, 0, 0, 0, 0, 0)
    observed = observer.wait_servo5(initial_pwm, boundary)
    commands.append(
        {
            "kind": "external_simulator_output_restore",
            "channel": CHANNEL,
            "requested_pwm": initial_pwm,
            "observed_pwm": observed,
        }
    )


def close_resources(
    runtime: RuntimeProcess,
    observer: Any,
    relay: Any,
    failure: str | None,
    initial_pwm: int | None,
    commands: list[dict[str, Any]],
) -> str | None:
    try:
        runtime.stop()
    except Exception as error:
        failure = failure or getattr(error, "category", "runtime_shutdown_failed")
    if observer is not None and initial_pwm is not None:
        try:
            restore_simulator_output(observer, initial_pwm, commands)
        except Exception as error:
            failure = failure or getattr(error, "category", "output_restoration_failed")
    for resource in (observer, relay):
        if resource is None:
            continue
        try:
            resource.close()
        except Exception as error:
            failure = failure or getattr(error, "category", "probe_shutdown_failed")
    return failure


def main() -> int:
    result = run_probe()
    print(json.dumps(result, sort_keys=True), file=sys.stderr if result["result"] == "failed" else sys.stdout)
    return 1 if result["result"] == "failed" else 0


if __name__ == "__main__":
    raise SystemExit(main())
