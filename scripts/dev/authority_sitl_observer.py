# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Read-only guards and ownship observations for the authority SITL probe."""

from __future__ import annotations

import json
import math
import subprocess
import time
from collections import deque
from pathlib import Path
from typing import Any

from pymavlink import mavutil

REVISION = "dbe792162d06cab66c3475fd5556bf7a120f119e"
IMAGE = "nomad-sitl:copter-4.7.1"
PROJECT, CONTAINER = "nomad-authority", "nomad-authority-sitl"
RELAY_PORT, OBSERVER_PORT = 14690, 14691
SERVO_FUNCTION = "SERVO5_FUNCTION"
READ_PARAMETERS = (
    "MAV_SYSID",
    "MAV_GCS_SYSID",
    "MAV_GCS_SYSID_HI",
    "MAV_OPTIONS",
    SERVO_FUNCTION,
    "FLTMODE_CH",
    "RC_OPTIONS",
    "RC_OVERRIDE_TIME",
    "FS_GCS_ENABLE",
)


class ProbeError(RuntimeError):
    def __init__(self, category: str, phase: str = "") -> None:
        super().__init__(category)
        self.category, self.phase = category, phase


def run_guard_command(arguments: list[str]) -> str:
    result = subprocess.run(arguments, capture_output=True, text=True, timeout=15, check=False)
    if result.returncode:
        raise ProbeError("simulator_guard_failed")
    return result.stdout.strip()


def validate_container(details: dict[str, Any]) -> None:
    config = details.get("Config", {})
    if details.get("Name", "").lstrip("/") != CONTAINER:
        raise ProbeError("wrong_sitl_topology")
    if config.get("Image") != IMAGE:
        raise ProbeError("wrong_firmware_image")
    labels = config.get("Labels", {})
    if labels.get("com.docker.compose.project") != PROJECT or labels.get("com.docker.compose.service") != "sitl":
        raise ProbeError("wrong_sitl_topology")
    if not details.get("State", {}).get("Running"):
        raise ProbeError("simulator_process_unverified")
    env = dict(item.split("=", 1) for item in config.get("Env", []) if "=" in item)
    expected = {
        "SITL_UDP_OUTPUT_ADDRESS": (
            f"udp:host.docker.internal:{RELAY_PORT} --out udp:host.docker.internal:{OBSERVER_PORT}"
        ),
        "SITL_FENCE_DEFAULTS": "/nomad/sitl-fence.parm",
        "SITL_STREAM_DEFAULTS": "/nomad/sitl-streams.parm",
        "VEHICLE": "ArduCopter",
        "MODEL": "quad",
        "SPEEDUP": "1",
    }
    if any(env.get(key) != value for key, value in expected.items()):
        raise ProbeError("wrong_sitl_topology")
    if config.get("Entrypoint") != ["/bin/sh", "/nomad/sitl-entrypoint.sh"]:
        raise ProbeError("wrong_sitl_topology")
    mounts = {mount.get("Destination"): mount for mount in details.get("Mounts", [])}
    required = ("/nomad/sitl-fence.parm", "/nomad/sitl-streams.parm", "/nomad/sitl-entrypoint.sh")
    if any(path not in mounts or mounts[path].get("RW") for path in required):
        raise ProbeError("simulator_profile_not_read_only")


def verify_isolated_simulator() -> None:
    inspected = json.loads(run_guard_command(["docker", "inspect", CONTAINER]))
    if not isinstance(inspected, list) or len(inspected) != 1:
        raise ProbeError("dedicated_simulator_not_identified")
    validate_container(inspected[0])
    revision = run_guard_command(["docker", "exec", CONTAINER, "git", "-C", "/ardupilot", "rev-parse", "HEAD"])
    if revision != REVISION:
        raise ProbeError("wrong_firmware_revision")
    process = run_guard_command(["docker", "exec", CONTAINER, "pgrep", "-af", "bin/arducopter"])
    if "bin/arducopter" not in process or "--model + " not in process:
        raise ProbeError("simulator_process_unverified")


def is_ownship(message: Any) -> bool:
    return message.get_srcSystem() == 1 and message.get_srcComponent() == 1


class AuthorityObserver:
    """Synchronous receive-only ownship observer, plus read-only parameter access."""

    def __init__(self, connection: Any) -> None:
        self.connection = connection
        self.heartbeats: deque[tuple[float, int, bool]] = deque(maxlen=256)
        self.servo5: deque[tuple[float, int]] = deque(maxlen=256)
        self.transitions: list[dict[str, Any]] = []
        self.last_mode: int | None = None
        self.parameters: dict[str, tuple[float, float]] = {}

    def pump(self, timeout: float = 0.2) -> None:
        message = self.connection.recv_match(blocking=True, timeout=timeout)
        if message is not None:
            self.record(message, time.monotonic())
        for _ in range(64):
            message = self.connection.recv_match(blocking=False)
            if message is None:
                return
            self.record(message, time.monotonic())

    def record(self, message: Any, observed_at: float) -> None:
        if not is_ownship(message):
            return
        kind = message.get_type()
        if kind == "HEARTBEAT":
            self.record_heartbeat(message, observed_at)
        elif kind == "SERVO_OUTPUT_RAW" and int(message.port) == 0:
            value = getattr(message, "servo5_raw", None)
            if value is not None:
                self.servo5.append((observed_at, int(value)))
        elif kind == "PARAM_VALUE":
            self.record_parameter(message, observed_at)

    def record_heartbeat(self, message: Any, observed_at: float) -> None:
        valid_type = message.type == mavutil.mavlink.MAV_TYPE_QUADROTOR
        valid_ap = message.autopilot == mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA
        if not valid_type or not valid_ap:
            raise ProbeError("wrong_aircraft_identity")
        armed = bool(message.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        if armed:
            raise ProbeError("aircraft_armed")
        mode = int(message.custom_mode)
        if self.last_mode is not None and mode != self.last_mode:
            self.transitions.append({"kind": "mode", "custom_mode": mode})
        self.last_mode = mode
        self.heartbeats.append((observed_at, mode, armed))

    def record_parameter(self, message: Any, observed_at: float) -> None:
        name = (
            message.param_id.decode("ascii", errors="ignore")
            if isinstance(message.param_id, bytes)
            else message.param_id
        )
        name = name.rstrip("\x00")
        value = float(message.param_value)
        if name in READ_PARAMETERS and math.isfinite(value):
            self.parameters[name] = (observed_at, value)

    def require_disarmed(self) -> None:
        if not self.heartbeats or time.monotonic() - self.heartbeats[-1][0] > 2.0:
            raise ProbeError("stale_observer_heartbeat")
        if self.heartbeats[-1][2]:
            raise ProbeError("aircraft_armed")

    def wait_vehicle(self, timeout: float = 15.0) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.pump()
            if self.heartbeats and time.monotonic() - self.heartbeats[-1][0] <= 2.0:
                self.require_disarmed()
                return
        raise ProbeError("fresh_disarmed_heartbeat_missing")

    def read_parameter(self, name: str, timeout: float = 5.0) -> float:
        boundary = time.monotonic()
        self.connection.mav.param_request_read_send(1, 1, name.encode("ascii"), -1)
        deadline = boundary + timeout
        while time.monotonic() < deadline:
            self.pump()
            sample = self.parameters.get(name)
            if sample is not None and sample[0] > boundary:
                return sample[1]
        raise ProbeError("parameter_readback_missing")

    def read_parameters(self) -> dict[str, float]:
        return {name: self.read_parameter(name) for name in READ_PARAMETERS}

    def wait_servo5(self, expected: int | None, boundary: float, timeout: float = 8.0) -> int:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.pump()
            self.require_disarmed()
            samples = [sample for sample in self.servo5 if sample[0] > boundary]
            if len(samples) >= 3 and time.monotonic() - samples[-1][0] <= 1.5:
                values = [value for _stamp, value in samples[-3:]]
                if len(set(values)) == 1 and (expected is None or values[-1] == expected):
                    return values[-1]
        raise ProbeError("servo_output_observation_failed")

    def wait_mode(self, mode: int, boundary: float, timeout: float = 8.0) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.pump()
            self.require_disarmed()
            recent = [sample for sample in self.heartbeats if sample[0] > boundary]
            if len(recent) >= 2 and all(sample[1] == mode for sample in recent[-2:]):
                return
        raise ProbeError("external_mode_change_not_observed")

    def close(self) -> None:
        self.connection.close()


SCOPE = "disarmed_pinned_copter_runtime_software_authority_boundary_only"


def code_identity() -> dict[str, Any]:
    root = Path(__file__).resolve().parents[2]
    sha = subprocess.run(
        ["git", "-C", str(root), "rev-parse", "HEAD"], capture_output=True, text=True, timeout=10, check=False
    )
    dirty = subprocess.run(
        ["git", "-C", str(root), "status", "--porcelain", "--untracked-files=normal"],
        capture_output=True,
        text=True,
        timeout=10,
        check=False,
    )
    return {
        "code_sha": sha.stdout.strip() if sha.returncode == 0 else None,
        "dirty_source": bool(dirty.stdout.strip()) if dirty.returncode == 0 else None,
    }


def summary(
    result: str,
    started: str,
    checks: dict[str, bool],
    transitions: list[dict[str, Any]],
    commands: list[dict[str, Any]],
    readbacks: dict[str, Any],
    runtime_source: dict[str, int] | None,
    failure: str | None = None,
    phase: str | None = None,
) -> dict[str, Any]:
    value: dict[str, Any] = {
        "result": result,
        "scope": SCOPE,
        "started_utc": started,
        "firmware_sha": REVISION,
        **code_identity(),
        "source_ids": {
            "sitl": {"system": 1, "component": 1},
            "external_test_gcs": {"system": 250, "component": 190},
            "runtime_command": runtime_source,
        },
        "parameter_readbacks": readbacks,
        "checks": checks,
        "transitions": transitions,
        "commands": commands,
    }
    if failure:
        value["failure"] = failure
    if phase:
        value["phase"] = phase
    return value
