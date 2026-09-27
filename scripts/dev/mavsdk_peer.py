# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Deterministic ArduPilot-like peer for the MAVSDK parity fixtures.

The peer is a UDP vehicle: it streams heartbeat/position/GPS/battery telemetry,
answers COMMAND_LONG/COMMAND_INT with a configurable COMMAND_ACK, and models
accepted state changes. Reposition targets advance gradually after their ACK so
the route verifier sees actual post-ACK progress.

It uses a raw socket rather than pymavlink's udpout because MAVSDK's udpin
endpoint learns its peer from the packets it receives: the peer must send first
and tolerate the ICMP resets a not-yet-bound listener produces on Windows.
"""

from __future__ import annotations

import socket
import threading
import time
from dataclasses import dataclass
from typing import Any

from mavsdk_peer_route import RouteProgress
from mavsdk_peer_transition import TransitionModel
from pymavlink.dialects.v20 import ardupilotmega as mavlink

ACCEPTED = mavlink.MAV_RESULT_ACCEPTED
DENIED = mavlink.MAV_RESULT_DENIED

# Fence transfer: ArduPilot stores a fence as MAV_MISSION_TYPE_FENCE items whose
# polygon vertices are MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION, and each
# vertex carries the polygon's vertex count in param1.
MISSION_TYPE_FENCE = mavlink.MAV_MISSION_TYPE_FENCE
FENCE_VERTEX_COMMAND = mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION
FENCE_ITEM_FRAME = mavlink.MAV_FRAME_GLOBAL_INT

# Params the peer serves. ArduPilot reports most entries as REAL32, and the
# fence cases need FENCE_ENABLE because a plan alone is not an enforced fence.
# ArduPilot reports that switch as an integer parameter, so keep the fixture's
# wire type faithful to the real SITL contract.
DEFAULT_PARAMS = {"FENCE_ENABLE": 1.0}
INTEGER_PARAMS = {"FENCE_ENABLE", "Q_ENABLE", "Q_GUIDED_MODE"}

# Command IDs the parity cases expect on the wire. They come from the dialect so
# a drift in the core shows up as a mismatch rather than a silently updated
# expectation.
COMMAND_NAV_RETURN_TO_LAUNCH = mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH
COMMAND_NAV_LAND = mavlink.MAV_CMD_NAV_LAND
COMMAND_NAV_TAKEOFF = mavlink.MAV_CMD_NAV_TAKEOFF
COMMAND_DO_SET_MODE = mavlink.MAV_CMD_DO_SET_MODE
COMMAND_DO_VTOL_TRANSITION = mavlink.MAV_CMD_DO_VTOL_TRANSITION
COMMAND_ARM_DISARM = mavlink.MAV_CMD_COMPONENT_ARM_DISARM
COMMAND_DO_REPOSITION = mavlink.MAV_CMD_DO_REPOSITION
COMMAND_DO_SET_SERVO = mavlink.MAV_CMD_DO_SET_SERVO
COMMAND_DO_SET_RELAY = mavlink.MAV_CMD_DO_SET_RELAY
COMMAND_DO_MOTOR_TEST = mavlink.MAV_CMD_DO_MOTOR_TEST
COMMAND_DO_MOUNT_CONFIGURE = mavlink.MAV_CMD_DO_MOUNT_CONFIGURE

MODE_RTL = 6
MODE_LAND = 9
GLOBAL_RELATIVE_ALT_INT_FRAME = mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT

# The identity NOMAD announces while no vehicle is latched. A heartbeat-gated
# relay opens its leg only after it sees a GCS heartbeat, so these are read from
# the dialect rather than typed here: a drift shows up as a mismatch.
GCS_VEHICLE_TYPE = mavlink.MAV_TYPE_GCS
GCS_AUTOPILOT_TYPE = mavlink.MAV_AUTOPILOT_INVALID
GCS_COMPONENT_ID = mavlink.MAV_COMP_ID_MISSIONPLANNER

# Body-frame velocity follows the vehicle's current heading; position is ignored.
BODY_NED_FRAME = mavlink.MAV_FRAME_BODY_NED

TELEMETRY_PERIOD_SECONDS = 0.2

# Home position the peer starts from; every position case must move it.
HOME_LATITUDE_DEG = 45.5017
HOME_LONGITUDE_DEG = -73.5673

# (wire form, command id, frame, params) for every command the peer saw.
CommandRecord = tuple[str, int, int | None, tuple[float, ...]]


@dataclass(frozen=True)
class SetpointRecord:
    """One SET_POSITION_TARGET_LOCAL_NED the peer decoded off the wire."""

    vx: float
    vy: float
    vz: float
    yaw_rate: float
    type_mask: int
    coordinate_frame: int
    target_system: int

    def is_zero(self) -> bool:
        return self.vx == 0.0 and self.vy == 0.0 and self.vz == 0.0 and self.yaw_rate == 0.0


@dataclass(frozen=True)
class ReceivedMessage:
    """One MAVLink message the peer received.

    Only the identity fields the link cases assert are kept; a non-heartbeat
    message leaves the vehicle type and autopilot at -1.
    """

    kind: str
    system_id: int
    component_id: int
    vehicle_type: int = -1
    autopilot: int = -1


def read_parameters(kind: str, message) -> tuple[float, ...]:
    """Read the seven MAVLink command parameters in NOMAD's canonical order."""
    if kind == "COMMAND_INT":
        # Location commands carry latitude/longitude in the scaled x/y fields.
        return (
            message.param1,
            message.param2,
            message.param3,
            message.param4,
            float(message.x),
            float(message.y),
            float(message.z),
        )
    return (
        message.param1,
        message.param2,
        message.param3,
        message.param4,
        message.param5,
        message.param6,
        message.param7,
    )


class VehiclePeer:
    """UDP ArduPilot-like peer for command and state parity tests.

    It can model relay gating, burst coalescing, and a silent bound endpoint.
    Accepted commands update simulated vehicle state for readback checks.
    """

    def __init__(
        self,
        port: int,
        system_id: int,
        ack_result: int | None = ACCEPTED,
        *,
        stream: bool = True,
        coalesce: bool = False,
        telemetry_seconds: float | None = None,
        bind: bool = False,
        params: dict[str, float] | None = None,
        vehicle_type: int = mavlink.MAV_TYPE_QUADROTOR,
        autopilot_type: int = mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
        initial_mode: int = 0,
        initial_armed: bool = False,
        vtol_state: int | None = None,
        initial_relative_altitude_m: float = 0.0,
        transition_reaches_fixed_wing: bool = True,
        transition_reaches_multicopter: bool = True,
        transition_reports_intermediate: bool = True,
    ) -> None:
        self._address, self._system_id, self._ack_result = ("127.0.0.1", port), system_id, ack_result
        self._streaming, self._coalesce = stream, coalesce
        self._telemetry_deadline = None if telemetry_seconds is None else time.monotonic() + telemetry_seconds
        self._messages: list[ReceivedMessage] = []
        self._setpoints: list[SetpointRecord] = []
        self._pending_acks: list[Any] = []
        self._params = dict(DEFAULT_PARAMS if params is None else params)
        self._vehicle_type, self._autopilot_type = vehicle_type, autopilot_type
        self._fence_polygon: list[tuple[float, float]] = []
        self._fence_arriving: list[tuple[int, float, float, float]] = []
        self._armed, self._custom_mode = initial_armed, initial_mode
        self._transition = TransitionModel.create(
            vtol_state,
            vehicle_type,
            self._params,
            transition_reaches_fixed_wing,
            transition_reaches_multicopter,
            transition_reports_intermediate,
        )
        self._latitude_deg, self._longitude_deg = HOME_LATITUDE_DEG, HOME_LONGITUDE_DEG
        self._relative_altitude_m = initial_relative_altitude_m
        self._route_progress = RouteProgress()
        self._commands, self.command_targets = [], []
        self._stop = threading.Event()
        self._socket = self._create_socket(bind)
        self._mavlink = mavlink.MAVLink(None, srcSystem=system_id, srcComponent=1)
        self._thread: threading.Thread | None = None

    def _create_socket(self, bind: bool) -> socket.socket:
        connection = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        if bind:
            connection.bind(self._address)
        return connection

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2)
        self._socket.close()

    def commands(self) -> list[CommandRecord]:
        return list(self._commands)

    def messages(self, kind: str | None = None) -> list[ReceivedMessage]:
        """Return every received message, optionally filtered by MAVLink type name."""
        return [item for item in self._messages if kind is None or item.kind == kind]

    def setpoints(self) -> list[SetpointRecord]:
        return list(self._setpoints)

    def fence_polygon(self) -> list[tuple[float, float]]:
        """The fence polygon the vehicle currently holds, as (latitude, longitude)."""
        return list(self._fence_polygon)

    def _send(self, message) -> None:
        self._send_bytes(message.pack(self._mavlink))

    def _send_bytes(self, payload: bytes) -> None:
        try:
            self._socket.sendto(payload, self._address)
        except OSError:
            # The listener may not be bound yet; the next period retries.
            pass

    def _run(self) -> None:
        self._socket.settimeout(0.05)
        last_telemetry = 0.0
        while not self._stop.is_set():
            now = time.monotonic()
            if now - last_telemetry >= TELEMETRY_PERIOD_SECONDS:
                self._send_telemetry()
                last_telemetry = now
            try:
                data, _ = self._socket.recvfrom(2048)
            except OSError:
                # A read timeout is an OSError; loop and send the next period.
                continue
            for byte in data:
                message = self._mavlink.parse_char(bytes([byte]))
                if message is not None:
                    self._handle(message)

    def _handle(self, message) -> None:
        kind = message.get_type()
        self._messages.append(
            ReceivedMessage(
                kind,
                int(message.get_srcSystem()),
                int(message.get_srcComponent()),
                int(getattr(message, "type", -1)),
                int(getattr(message, "autopilot", -1)),
            )
        )
        if kind == "SET_POSITION_TARGET_LOCAL_NED":
            self._setpoints.append(self._read_setpoint(message))
        if kind in ("PARAM_REQUEST_READ", "PARAM_REQUEST_LIST"):
            self._serve_params(message)
            return
        if kind.startswith("MISSION_"):
            self._serve_fence_transfer(kind, message)
            return
        if kind not in ("COMMAND_LONG", "COMMAND_INT"):
            return
        command = int(message.command)
        frame = getattr(message, "frame", None)
        self._commands.append((kind, command, frame, read_parameters(kind, message)))
        self.command_targets.append((command, int(message.target_system), int(message.target_component)))
        self._apply(kind, command, message)
        self._acknowledge(command)

    def _serve_params(self, message) -> None:
        """Answer PARAM_REQUEST_READ / PARAM_REQUEST_LIST from the param table."""
        if message.get_type() == "PARAM_REQUEST_LIST":
            names = sorted(self._params)
            for index, name in enumerate(names):
                self._send(self._param_value(name, index, len(names)))
            return
        name = message.param_id.strip("\x00")
        if not name and int(message.param_index) >= 0:
            names = sorted(self._params)
            index = int(message.param_index)
            name = names[index] if index < len(names) else ""
        if name in self._params:
            self._send(self._param_value(name, sorted(self._params).index(name), len(self._params)))

    def _param_value(self, name: str, index: int, count: int):
        # pymavlink's generated encoder wants the 16-byte field as bytes; it
        # decodes it back to str for the received-message accessor.
        param_type = mavlink.MAV_PARAM_TYPE_INT8 if name in INTEGER_PARAMS else mavlink.MAV_PARAM_TYPE_REAL32
        return self._mavlink.param_value_encode(
            name.encode("ascii"), float(self._params[name]), param_type, count, index
        )

    def _serve_fence_transfer(self, kind: str, message) -> None:
        """Model ArduPilot's fence mission handshake in both directions.

        Upload: the GCS sends MISSION_COUNT, the vehicle asks for each item and
        then acknowledges. Download: the GCS asks for the list, the vehicle
        answers with its count and serves the items it holds.
        """
        if int(getattr(message, "mission_type", MISSION_TYPE_FENCE)) != MISSION_TYPE_FENCE:
            return
        if kind == "MISSION_REQUEST_LIST":
            # Download initiator: answer with the count of the polygon held.
            self._send(
                self._mavlink.mission_count_encode(self._system_id, 1, len(self._fence_polygon), MISSION_TYPE_FENCE)
            )
        elif kind == "MISSION_COUNT":
            self._fence_expected = int(message.count)
            self._fence_arriving = []
            self._request_fence_item(0)
        elif kind in ("MISSION_REQUEST_INT", "MISSION_REQUEST"):
            self._send_fence_item(int(message.seq))
        elif kind == "MISSION_ITEM_INT":
            self._accept_fence_item(message)

    def _request_fence_item(self, sequence: int) -> None:
        if sequence >= self._fence_expected:
            self._fence_polygon = [(lat, lon) for _command, lat, lon, _param1 in self._fence_arriving]
            self._send(
                self._mavlink.mission_ack_encode(self._system_id, 1, mavlink.MAV_MISSION_ACCEPTED, MISSION_TYPE_FENCE)
            )
            return
        self._send(self._mavlink.mission_request_int_encode(self._system_id, 1, sequence, MISSION_TYPE_FENCE))

    def _accept_fence_item(self, message) -> None:
        self._fence_arriving.append(
            (int(message.command), float(message.x) / 1e7, float(message.y) / 1e7, float(message.param1))
        )
        self._request_fence_item(int(message.seq) + 1)

    def _send_fence_item(self, sequence: int) -> None:
        if sequence >= len(self._fence_polygon):
            return
        latitude, longitude = self._fence_polygon[sequence]
        vertex_count = float(len(self._fence_polygon))
        self._send(
            self._mavlink.mission_item_int_encode(
                1,
                1,
                sequence,
                FENCE_ITEM_FRAME,
                FENCE_VERTEX_COMMAND,
                0,
                1,
                vertex_count,
                0.0,
                0.0,
                0.0,
                int(latitude * 1e7),
                int(longitude * 1e7),
                0.0,
                MISSION_TYPE_FENCE,
            )
        )

    @staticmethod
    def _read_setpoint(message) -> SetpointRecord:
        return SetpointRecord(
            float(message.vx),
            float(message.vy),
            float(message.vz),
            float(message.yaw_rate),
            int(message.type_mask),
            int(message.coordinate_frame),
            int(message.target_system),
        )

    def _acknowledge(self, command: int) -> None:
        if self._ack_result is None:
            return
        ack = self._mavlink.command_ack_encode(command, self._ack_result)
        if self._coalesce:
            # Hold the ack so the next telemetry datagram carries it, exactly
            # as a coalescing link buries it inside a burst.
            self._pending_acks.append(ack)
            return
        self._send(ack)

    def _apply(self, kind: str, command: int, message) -> None:
        """Apply the vehicle change an accepted command asks for.

        Without this the core's state verification would time out on a peer that
        only acknowledges; the parity cases assert verification, not just the ACK.
        """
        if command == COMMAND_ARM_DISARM:
            self._armed = float(message.param1) > 0.0
        elif command == COMMAND_DO_SET_MODE:
            self._custom_mode = int(float(message.param2))
        elif command == COMMAND_DO_VTOL_TRANSITION and int(float(message.param1)) in (
            mavlink.MAV_VTOL_STATE_FW,
            mavlink.MAV_VTOL_STATE_MC,
        ):
            if self._ack_result == ACCEPTED:
                self._transition.request(int(float(message.param1)), time.monotonic())
        elif command == COMMAND_NAV_TAKEOFF:
            self._relative_altitude_m = float(message.param7)
        elif command == COMMAND_DO_REPOSITION and kind == "COMMAND_INT":
            if self._ack_result == ACCEPTED:
                self._route_progress.start((float(message.x) / 1e7, float(message.y) / 1e7, float(message.z)))
        elif command == COMMAND_NAV_RETURN_TO_LAUNCH:
            self._custom_mode = MODE_RTL
        elif command == COMMAND_NAV_LAND:
            self._custom_mode = MODE_LAND

    def _send_telemetry(self) -> None:
        if not self._streaming or self._telemetry_ended():
            return
        self._advance_route_position()
        self._transition.advance(time.monotonic())
        frames = [
            self._heartbeat_message(),
            self._position_message(),
            self._gps_message(),
            self._attitude_message(),
            self._sys_status_message(),
            self._extended_sys_state_message(),
        ]
        frames.extend(self._pending_acks)
        self._pending_acks = []
        if self._coalesce:
            self._send_bytes(b"".join(frame.pack(self._mavlink) for frame in frames))
            return
        for frame in frames:
            self._send(frame)

    def _advance_route_position(self) -> None:
        position = self._route_progress.advance(self._latitude_deg, self._longitude_deg)
        if position is None:
            return
        self._latitude_deg, self._longitude_deg, self._relative_altitude_m = position

    def _telemetry_ended(self) -> bool:
        return self._telemetry_deadline is not None and time.monotonic() >= self._telemetry_deadline

    def _heartbeat_message(self):
        base_mode = mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        if self._armed:
            base_mode |= mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        return self._mavlink.heartbeat_encode(
            self._vehicle_type,
            self._autopilot_type,
            base_mode,
            self._custom_mode,
            mavlink.MAV_STATE_ACTIVE,
        )

    def _position_message(self):
        return self._mavlink.global_position_int_encode(
            1000,
            int(self._latitude_deg * 1e7),
            int(self._longitude_deg * 1e7),
            25000,
            int(self._relative_altitude_m * 1000),
            0,
            0,
            0,
            0,
        )

    def _gps_message(self):
        return self._mavlink.gps_raw_int_encode(
            1_000_000,
            3,
            int(self._latitude_deg * 1e7),
            int(self._longitude_deg * 1e7),
            25000,
            100,
            100,
            0,
            0,
            12,
        )

    def _attitude_message(self):
        return self._mavlink.attitude_encode(1000, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    def _sys_status_message(self):
        return self._mavlink.sys_status_encode(0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 100, 12600, -1, 75, 0, 0, 0, 0, 0, 0)

    def _extended_sys_state_message(self):
        landed_state = (
            mavlink.MAV_LANDED_STATE_IN_AIR if self._relative_altitude_m > 0.5 else mavlink.MAV_LANDED_STATE_ON_GROUND
        )
        return self._mavlink.extended_sys_state_encode(self._transition.vtol_state, landed_state)
