# SPDX-License-Identifier: Apache-2.0
"""Prove delivery of a simulator RC fault while disarmed, not flight response."""

from __future__ import annotations

import json
import math
import subprocess
import sys
import threading
import time
from collections import deque
from pathlib import Path

from pymavlink import mavutil

REVISION = "dbe792162d06cab66c3475fd5556bf7a120f119e"
IMAGE = "nomad-sitl:plane-4.7.1-quadplane-tilttri"
COMPOSE_PROJECT = "nomad-quadplane-probe"
CONTAINER = "nomad-quadplane-rc-probe"
CLIENT_PORT = 14680
OBSERVER_PORT = 14681
RC_RECEIVER = mavutil.mavlink.MAV_SYS_STATUS_SENSOR_RC_RECEIVER
PARAMETERS = (
    "Q_ENABLE",
    "FLTMODE_CH",
    "RC5_OPTION",
    "THR_FAILSAFE",
    "THR_FS_VALUE",
    "RC_FS_TIMEOUT",
    "FS_SHORT_ACTN",
    "FS_LONG_TIMEOUT",
    "FS_LONG_ACTN",
    "FS_GCS_ENABL",
)
REQUIRED_MOUNTS = (
    "/nomad/quadplane-tilttri.parm",
    "/nomad/sitl-quadplane-entrypoint.sh",
)


class ProbeError(RuntimeError):
    def __init__(self, message: str, category: str = "probe_guard_or_observation_failure") -> None:
        super().__init__(message)
        self.category = category


class ObservationTimeout(ProbeError):
    pass


class ProbeCleanupError(ProbeError):
    def __init__(self, primary_error: BaseException | None, cleanup_errors: list[BaseException]) -> None:
        super().__init__("fault observation and restoration were not both proved", "restoration_failure")
        self.primary_error = primary_error
        self.cleanup_errors = cleanup_errors


def run_guard_command(arguments: list[str]) -> str:
    result = subprocess.run(arguments, capture_output=True, text=True, timeout=15, check=False)
    if result.returncode != 0:
        raise ProbeError("isolated simulator guard command failed")
    return result.stdout.strip()


def validate_container(details: dict) -> None:
    if details.get("Name", "").lstrip("/") != CONTAINER:
        raise ProbeError("probe container name does not match the dedicated instance", "wrong_sitl_topology")
    if details.get("Config", {}).get("Image") != IMAGE:
        raise ProbeError("probe simulator image is not pinned", "wrong_firmware_pin")

    labels = details.get("Config", {}).get("Labels", {})
    if labels.get("com.docker.compose.project") != COMPOSE_PROJECT:
        raise ProbeError("probe requires its dedicated Compose project", "wrong_sitl_topology")
    if labels.get("com.docker.compose.service") != "quadplane_sitl":
        raise ProbeError("probe container is not the QuadPlane SITL service", "wrong_sitl_topology")
    if not details.get("State", {}).get("Running"):
        raise ProbeError("probe simulator is not running", "simulator_process_unverified")

    settings = dict(item.split("=", 1) for item in details["Config"].get("Env", []) if "=" in item)
    expected = {
        "SITL_UDP_OUTPUT_ADDRESS": f"udp:host.docker.internal:{CLIENT_PORT}",
        "SITL_UDP_OBSERVER_ADDRESS": f"udp:host.docker.internal:{OBSERVER_PORT}",
        "SITL_PROFILE_DEFAULTS": "/nomad/quadplane-tilttri.parm",
        "SPEEDUP": "1",
    }
    if any(settings.get(key) != value for key, value in expected.items()):
        raise ProbeError("probe simulator routing/profile does not match the isolated topology", "wrong_sitl_topology")

    entrypoint = details["Config"].get("Entrypoint")
    if entrypoint != ["/bin/sh", "/nomad/sitl-quadplane-entrypoint.sh"]:
        raise ProbeError("probe simulator entrypoint is not the pinned profile entrypoint", "wrong_sitl_topology")
    mounts = {mount.get("Destination"): mount for mount in details.get("Mounts", [])}
    if any(destination not in mounts or mounts[destination].get("RW") for destination in REQUIRED_MOUNTS):
        raise ProbeError("probe simulator profile mounts are missing or writable", "wrong_sitl_topology")


def verify_isolated_simulator() -> None:
    details = json.loads(run_guard_command(["docker", "inspect", CONTAINER]))
    if not isinstance(details, list) or len(details) != 1:
        raise ProbeError("dedicated simulator container was not identified")
    validate_container(details[0])

    revision = run_guard_command(["docker", "exec", CONTAINER, "git", "-C", "/ardupilot", "rev-parse", "HEAD"])
    if revision != REVISION:
        raise ProbeError("probe simulator firmware revision is not pinned", "wrong_firmware_pin")
    command = run_guard_command(["docker", "exec", CONTAINER, "pgrep", "-af", "bin/arduplane"])
    if "bin/arduplane" not in command or "--model" not in command or "quadplane-tilttri" not in command:
        raise ProbeError("native ArduPlane simulator process was not verified", "simulator_process_unverified")


def is_ownship(message) -> bool:
    return message.get_srcSystem() == 1 and message.get_srcComponent() == 1


def get_receiver_health(message) -> bool:
    if not message.onboard_control_sensors_present & RC_RECEIVER:
        raise ProbeError("receiver health is not observable", "receiver_health_unavailable")
    return bool(message.onboard_control_sensors_health & RC_RECEIVER)


class ReceiverObserver:
    """Receive only; never emit a heartbeat, override or aircraft command."""

    def __init__(self, connection) -> None:
        self.connection = connection
        self.changed = threading.Condition()
        self.samples: deque[tuple[float, bool]] = deque(maxlen=512)
        self.heartbeat_at = 0.0
        self.error = ""
        self.stop_requested = threading.Event()
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.started_at = 0.0

    def start(self) -> None:
        self.started_at = time.monotonic()
        self.thread.start()

    def run(self) -> None:
        try:
            while not self.stop_requested.is_set():
                message = self.connection.recv_match(blocking=True, timeout=0.2)
                if message is not None and is_ownship(message):
                    self.record(message, time.monotonic())
        except Exception as error:
            safe_error = str(error) if isinstance(error, ProbeError) else "observer failed"
            with self.changed:
                self.error = safe_error
                self.changed.notify_all()

    def record(self, message, observed_at: float) -> None:
        with self.changed:
            kind = message.get_type()
            if kind == "HEARTBEAT":
                self.record_heartbeat(message, observed_at)
            elif kind == "SYS_STATUS":
                self.samples.append((observed_at, get_receiver_health(message)))
            self.changed.notify_all()

    def record_heartbeat(self, message, observed_at: float) -> None:
        if message.type != mavutil.mavlink.MAV_TYPE_FIXED_WING:
            raise ProbeError("observer aircraft is not the pinned Plane/QuadPlane type", "wrong_aircraft_identity")
        if message.autopilot != mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA:
            raise ProbeError("observer autopilot is not ArduPilot", "wrong_aircraft_identity")
        if message.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
            raise ProbeError("probe requires the aircraft to remain disarmed", "aircraft_armed")
        self.heartbeat_at = observed_at

    def wait_for_health(
        self,
        healthy: bool,
        boundary: float,
        timeout: float = 20.0,
        fail_on_change: bool = False,
        timeout_category: str = "receiver_state_transition_not_observed",
    ) -> list[tuple[float, bool]]:
        deadline = time.monotonic() + timeout
        with self.changed:
            while time.monotonic() < deadline:
                if self.error:
                    raise ProbeError(self.error)
                now = time.monotonic()
                if self.heartbeat_at and now - self.heartbeat_at > 2.0:
                    raise ProbeError("observer heartbeat became stale", "stale_observer_heartbeat")
                samples = [(stamp, value) for stamp, value in self.samples if stamp > boundary]
                if fail_on_change and any(value != healthy for _, value in samples):
                    raise ProbeError(
                        "receiver state changed during the healthy no-injection control",
                        "no_injection_control_failed",
                    )
                if self.heartbeat_at > boundary and len(samples) >= 3:
                    final = samples[-3:]
                    if all(value == healthy for _, value in final) and now - final[-1][0] <= 1.5:
                        return final
                self.changed.wait(timeout=min(0.2, max(0.0, deadline - now)))
        raise ObservationTimeout(f"receiver health={healthy} was not observed after the boundary", timeout_category)

    def close(self) -> None:
        self.stop_requested.set()
        if self.thread.ident is not None:
            self.thread.join(timeout=2)
        connection_error = False
        try:
            self.connection.close()
        except Exception:
            connection_error = True
        if self.thread.is_alive() or connection_error:
            raise ProbeError("observer shutdown did not complete cleanly")


class SimulatorParameters:
    def __init__(self, connection) -> None:
        self.connection = connection

    def drain_pending_messages(self) -> None:
        for _ in range(256):
            if self.connection.recv_match(blocking=False) is None:
                return
        raise ProbeError("control input queue did not drain before parameter access")

    def wait_for_parameter(self, name: str, boundary: float) -> float:
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            message = self.connection.recv_match(blocking=True, timeout=0.2)
            received_at = time.monotonic()
            if message is None or received_at <= boundary or not is_ownship(message):
                continue
            if message.get_type() == "HEARTBEAT":
                if message.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                    raise ProbeError("parameter controller observed an armed aircraft", "aircraft_armed")
                continue
            if message.get_type() != "PARAM_VALUE":
                continue
            parameter_id = message.param_id
            if isinstance(parameter_id, bytes):
                parameter_id = parameter_id.decode("ascii")
            if parameter_id.rstrip("\x00") != name:
                continue
            value = float(message.param_value)
            if not math.isfinite(value):
                raise ProbeError(f"non-finite simulator parameter {name}")
            return value
        raise ObservationTimeout(
            f"simulator parameter {name} was not freshly read back", "fresh_parameter_readback_missing"
        )

    def read(self, name: str) -> float:
        self.drain_pending_messages()
        boundary = time.monotonic()
        self.connection.mav.param_request_read_send(1, 1, name.encode("ascii"), -1)
        return self.wait_for_parameter(name, boundary)

    def set_rc_failure(self, value: int) -> None:
        if value not in (0, 1):
            raise ProbeError("only simulator no-pulses injection and restoration are allowed")
        self.drain_pending_messages()
        boundary = time.monotonic()
        self.connection.mav.param_set_send(1, 1, b"SIM_RC_FAIL", value, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        acknowledgement = self.wait_for_parameter("SIM_RC_FAIL", boundary)
        if acknowledgement != value or self.read("SIM_RC_FAIL") != value:
            raise ProbeError("simulator RC fault parameter did not match the request", "simulator_parameter_mismatch")


def read_probe_parameters(parameters: SimulatorParameters) -> dict[str, float]:
    values = {name: parameters.read(name) for name in PARAMETERS}
    if values["Q_ENABLE"] != 2:
        raise ProbeError("probe requires the pinned Q_ENABLE=2 profile")
    values["SIM_RC_FAIL"] = parameters.read("SIM_RC_FAIL")
    if values["SIM_RC_FAIL"] != 0:
        raise ProbeError("probe requires healthy initial simulator RC input")
    return values


def restore_rc_input(
    parameters, observer, boundary: float, timeout_category: str = "receiver_recovery_not_observed"
) -> tuple[list[tuple[float, bool]] | None, list[BaseException]]:
    errors: list[BaseException] = []
    recovery = None
    try:
        parameters.set_rc_failure(0)
    except BaseException as error:
        errors.append(error)
    try:
        recovery = observer.wait_for_health(True, boundary, timeout_category=timeout_category)
    except BaseException as error:
        errors.append(error)
    return recovery, errors


def format_health_samples(samples: list[tuple[float, bool]], origin: float) -> list[dict]:
    return [{"t_s": round(stamp - origin, 3), "receiver_healthy": healthy} for stamp, healthy in samples]


def format_probe_evidence(
    origin: float,
    profile: dict[str, float],
    baseline: list[tuple[float, bool]],
    healthy_control: list[tuple[float, bool]],
    fault_boundary: float,
    loss: list[tuple[float, bool]],
    restore_boundary: float,
    recovery: list[tuple[float, bool]],
) -> dict:
    return {
        "scope": "disarmed_simulated_rc_receiver_loss_only_not_total_c2_cut",
        "firmware_sha": REVISION,
        "parameter_readbacks": profile,
        "fault_readback": 1,
        "restored_readback": 0,
        "fault_boundary_s": round(fault_boundary - origin, 3),
        "restore_boundary_s": round(restore_boundary - origin, 3),
        "baseline_samples": format_health_samples(baseline, origin),
        "healthy_no_injection_samples": format_health_samples(healthy_control, origin),
        "unhealthy_fault_samples": format_health_samples(loss, origin),
        "healthy_recovery_samples": format_health_samples(recovery, origin),
    }


def run_receiver_probe(parameters: SimulatorParameters, observer: ReceiverObserver) -> dict:
    origin = time.monotonic()
    profile = read_probe_parameters(parameters)
    baseline = observer.wait_for_health(
        True, observer.started_at, fail_on_change=True, timeout_category="baseline_receiver_health_not_observed"
    )
    control_boundary = time.monotonic()
    healthy_control = observer.wait_for_health(
        True,
        control_boundary,
        timeout=8.0,
        fail_on_change=True,
        timeout_category="no_injection_receiver_health_not_observed",
    )

    fault_boundary = time.monotonic()
    try:
        parameters.set_rc_failure(1)
        loss = observer.wait_for_health(False, fault_boundary, timeout_category="simulated_rc_loss_not_observed")
    except BaseException as primary_error:
        _, cleanup_errors = restore_rc_input(parameters, observer, time.monotonic())
        if cleanup_errors:
            raise ProbeCleanupError(primary_error, cleanup_errors) from primary_error
        raise

    restore_boundary = time.monotonic()
    recovery, cleanup_errors = restore_rc_input(parameters, observer, restore_boundary)
    if cleanup_errors:
        raise ProbeCleanupError(None, cleanup_errors)
    if recovery is None:
        raise ProbeError("receiver recovery was not observed")
    return format_probe_evidence(
        origin, profile, baseline, healthy_control, fault_boundary, loss, restore_boundary, recovery
    )


def get_code_identity() -> dict:
    root = Path(__file__).resolve().parents[2]
    sha = run_guard_command(["git", "-C", str(root), "rev-parse", "HEAD"])
    status = run_guard_command(["git", "-C", str(root), "status", "--porcelain", "--untracked-files=normal"])
    return {"code_sha": sha, "dirty_source": bool(status)}


def failure_category(error: BaseException) -> str:
    if isinstance(error, ProbeCleanupError):
        return "probe_and_restoration_failure" if error.primary_error else "restoration_failure"
    if isinstance(error, ObservationTimeout):
        return error.category
    if isinstance(error, ProbeError):
        return error.category
    if isinstance(error, subprocess.TimeoutExpired):
        return "read_only_guard_timeout"
    return type(error).__name__


def failure_evidence(primary: BaseException | None, shutdown_errors: list[BaseException]) -> dict:
    if isinstance(primary, ProbeCleanupError):
        primary_error = primary.primary_error
        cleanup_errors = primary.cleanup_errors
    else:
        primary_error = primary
        cleanup_errors = []
    try:
        identity = get_code_identity()
    except Exception:
        identity = {"code_sha": None, "dirty_source": None}
    return {
        "result": "failed",
        "scope": "disarmed_simulated_rc_receiver_loss_only_not_total_c2_cut",
        "fault_delivery": "not_proven",
        "firmware_expected_sha": REVISION,
        **identity,
        "failure": None if primary_error is None else failure_category(primary_error),
        "restoration_failures": [failure_category(error) for error in cleanup_errors],
        "shutdown_failures": [failure_category(error) for error in shutdown_errors],
    }


def main() -> int:
    observer = None
    control = None
    evidence = None
    primary_error = None
    shutdown_errors: list[BaseException] = []
    try:
        verify_isolated_simulator()
        control = mavutil.mavlink_connection(f"udpin:0.0.0.0:{CLIENT_PORT}", source_system=250)
        passive = mavutil.mavlink_connection(f"udpin:0.0.0.0:{OBSERVER_PORT}", source_system=251)
        observer = ReceiverObserver(passive)
        observer.start()
        heartbeat = control.wait_heartbeat(timeout=10)
        if heartbeat is None or not is_ownship(heartbeat):
            raise ProbeError("parameter controller did not discover the isolated aircraft")
        evidence = run_receiver_probe(SimulatorParameters(control), observer)
        evidence.update(get_code_identity())
    except BaseException as error:
        primary_error = error
    finally:
        if observer is not None:
            try:
                observer.close()
            except BaseException as error:
                shutdown_errors.append(error)
        if control is not None:
            try:
                control.close()
            except BaseException as error:
                shutdown_errors.append(error)

    if primary_error is not None or shutdown_errors:
        print(json.dumps(failure_evidence(primary_error, shutdown_errors), sort_keys=True), file=sys.stderr)
        return 1
    print(json.dumps({"result": "passed", **evidence}, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
