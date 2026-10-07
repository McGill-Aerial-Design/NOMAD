# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Pinned ArduPlane identity and QLAND responses for transport tests."""

from __future__ import annotations

from mavsdk_peer import ACCEPTED, COMMAND_DO_SET_MODE, VehiclePeer
from pymavlink.dialects.v20 import ardupilotmega as mavlink


class QlandPeer(VehiclePeer):
    """VehiclePeer variant that models the pinned version and QLAND ACK."""

    def __init__(self, *args, qland_ack_result: int, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self._qland_ack_result = qland_ack_result

    def _handle(self, message) -> None:
        super()._handle(message)
        if (
            message.get_type() == "COMMAND_LONG"
            and int(message.command) == mavlink.MAV_CMD_REQUEST_MESSAGE
            and int(float(message.param1)) == mavlink.MAVLINK_MSG_ID_AUTOPILOT_VERSION
        ):
            self._send_spoofed_version(system_id=2, component_id=1)
            self._send_spoofed_version(system_id=1, component_id=2)
            self._send(encode_autopilot_version(self._mavlink))

    def _acknowledge(self, command: int) -> None:
        if command == COMMAND_DO_SET_MODE:
            self._send(self._mavlink.command_ack_encode(command, self._qland_ack_result))
            return
        super()._acknowledge(command)

    def _send_spoofed_version(self, system_id: int, component_id: int) -> None:
        encoder = mavlink.MAVLink(None, srcSystem=system_id, srcComponent=component_id)
        message = encode_autopilot_version(encoder, software_version=(4 << 24) | (6 << 16))
        self._send_bytes(message.pack(encoder))


def encode_autopilot_version(encoder, software_version: int = (4 << 24) | (7 << 16) | (1 << 8)):
    flight_custom_version = list(b"dbe79216")
    return encoder.autopilot_version_encode(
        mavlink.MAV_PROTOCOL_CAPABILITY_MISSION_INT,
        software_version,
        0,
        0,
        0,
        flight_custom_version,
        [0] * 8,
        [0] * 8,
        0,
        0,
        0,
    )


def with_qland_peer(port: int, qland_ack_result: int, action):
    peer = QlandPeer(
        port,
        1,
        ACCEPTED,
        qland_ack_result=qland_ack_result,
        params={"FENCE_ENABLE": 1.0, "Q_ENABLE": 2.0},
        vehicle_type=mavlink.MAV_TYPE_FIXED_WING,
        autopilot_type=mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
    )
    peer.start()
    try:
        return action(peer)
    finally:
        peer.stop()
