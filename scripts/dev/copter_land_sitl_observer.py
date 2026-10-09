# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Independent ownship observations for the guarded Copter LAND scenario."""

from __future__ import annotations

import time
from collections import deque
from typing import Any

from authority_sitl_observer import ProbeError, is_ownship
from pymavlink import mavutil

GUIDED, LAND = 4, 9
MAX_SAMPLE_AGE = 1.5
MIN_AIRBORNE_ALTITUDE = 4.0
GROUND_ALTITUDE_LIMIT = 0.35
GROUND_SAMPLE_COUNT = 5
GROUND_DWELL_SECONDS = 0.5
PREARM_CHECK = mavutil.mavlink.MAV_SYS_STATUS_PREARM_CHECK


def latest_fresh(samples: Any, boundary: float = 0.0) -> Any:
    if not samples:
        return None
    sample = samples[-1]
    age = time.monotonic() - sample[0]
    return sample if sample[0] > boundary and 0.0 <= age <= MAX_SAMPLE_AGE else None


class CopterLandObserver:
    """Receive-only flight observer; setup commands belong to the scenario."""

    def __init__(self, connection: Any) -> None:
        self.connection = connection
        self.heartbeats: deque[tuple[float, int, bool]] = deque(maxlen=512)
        self.positions: deque[tuple[float, float]] = deque(maxlen=512)
        self.gps_fixes: deque[tuple[float, int]] = deque(maxlen=64)
        self.prearm_checks: deque[tuple[float, bool]] = deque(maxlen=64)
        self.landed_states: deque[tuple[float, int]] = deque(maxlen=512)
        self.acknowledgements: deque[tuple[float, int, int]] = deque(maxlen=64)

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
        elif kind == "GLOBAL_POSITION_INT":
            self.positions.append((observed_at, float(message.relative_alt) / 1000.0))
        elif kind == "GPS_RAW_INT":
            self.gps_fixes.append((observed_at, int(message.fix_type)))
        elif kind == "SYS_STATUS":
            present = bool(message.onboard_control_sensors_present & PREARM_CHECK)
            enabled = bool(message.onboard_control_sensors_enabled & PREARM_CHECK)
            healthy = bool(message.onboard_control_sensors_health & PREARM_CHECK)
            self.prearm_checks.append((observed_at, present and enabled and healthy))
        elif kind == "EXTENDED_SYS_STATE":
            self.landed_states.append((observed_at, int(message.landed_state)))
        elif kind == "COMMAND_ACK":
            self.acknowledgements.append((observed_at, int(message.command), int(message.result)))

    def record_heartbeat(self, message: Any, observed_at: float) -> None:
        if (
            message.type != mavutil.mavlink.MAV_TYPE_QUADROTOR
            or message.autopilot != mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA
        ):
            raise ProbeError("wrong_aircraft_identity")
        armed = bool(message.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        self.heartbeats.append((observed_at, int(message.custom_mode), armed))

    def has_mode(self, mode: int, armed: bool, boundary: float = 0.0) -> bool:
        heartbeat = latest_fresh(self.heartbeats, boundary)
        return heartbeat is not None and heartbeat[1:] == (mode, armed)

    def is_disarmed_at_low_altitude(self) -> bool:
        heartbeat = latest_fresh(self.heartbeats)
        position = latest_fresh(self.positions)
        return (
            heartbeat is not None
            and not heartbeat[2]
            and position is not None
            and abs(position[1]) <= GROUND_ALTITUDE_LIMIT
        )

    def is_airborne(self, boundary: float = 0.0) -> bool:
        position = latest_fresh(self.positions, boundary)
        landed = latest_fresh(self.landed_states, boundary)
        return (
            self.has_mode(GUIDED, True, boundary)
            and position is not None
            and position[1] >= MIN_AIRBORNE_ALTITUDE
            and landed is not None
            and landed[1] == mavutil.mavlink.MAV_LANDED_STATE_IN_AIR
        )

    def is_ready_for_setup(self) -> bool:
        fix = latest_fresh(self.gps_fixes)
        # GPS can be ready before the EKF/fence position estimate permits native arming.
        prearm = latest_fresh(self.prearm_checks)
        return (
            self.is_disarmed_at_low_altitude() and fix is not None and fix[1] >= 3 and prearm is not None and prearm[1]
        )

    def ground_samples(self, boundary: float) -> list[tuple[float, float]]:
        cutoff = max(boundary, time.monotonic() - MAX_SAMPLE_AGE)
        samples: list[tuple[float, float]] = []
        for sample in self.positions:
            if sample[0] <= cutoff:
                continue
            if abs(sample[1]) > GROUND_ALTITUDE_LIMIT:
                samples.clear()
                continue
            altitudes = [value[1] for value in samples] + [sample[1]]
            if max(altitudes) - min(altitudes) > 0.15:
                samples.clear()
            samples.append(sample)
        return samples

    def has_stable_ground_state(self, boundary: float) -> bool:
        landed = latest_fresh(self.landed_states, boundary)
        samples = self.ground_samples(boundary)
        return (
            self.has_mode(LAND, False, boundary)
            and landed is not None
            and landed[1] == mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND
            and len(samples) >= GROUND_SAMPLE_COUNT
            and latest_fresh(samples, boundary) is not None
            and samples[-1][0] - samples[0][0] >= GROUND_DWELL_SECONDS
        )

    def wait_for(self, condition: Any, category: str, timeout: float) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.pump(0.1)
            if condition():
                return
        raise ProbeError(category)

    def wait_acknowledgement(self, command: int, boundary: float) -> None:
        def accepted() -> bool:
            matches = [sample for sample in self.acknowledgements if sample[0] > boundary and sample[1] == command]
            if not matches:
                return False
            if matches[-1][2] != mavutil.mavlink.MAV_RESULT_ACCEPTED:
                raise ProbeError("simulator_setup_command_rejected")
            return True

        self.wait_for(accepted, "simulator_setup_ack_missing", 10.0)

    def close(self) -> None:
        self.connection.close()
