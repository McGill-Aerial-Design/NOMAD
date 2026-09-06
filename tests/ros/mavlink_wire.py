# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Structural MAVLink frame decoding shared by the ROS adapter tests.

The test responder decodes what the C++ core actually puts on the wire
(structurally, without a MAVLink parser) so the assertions are independent of
pymavlink's encode/decode layout tables. Both MAVLink v1 (0xFE, 6-byte header —
what the core sends for COMMAND_LONG and the velocity setpoint) and v2 (0xFD,
10-byte header) are handled.
"""

from __future__ import annotations

import struct

# MAVLink message ids used by the core (see src/mavlink/protocol.cpp).
COMMAND_LONG_ID = 76
SET_POSITION_TARGET_LOCAL_NED_ID = 84

# Command ids the core sends (see src/vehicle/vehicle.cpp and output.cpp).
ARM_DISARM_COMMAND = 400
LAND_COMMAND = 21
RTL_COMMAND = 20

MAV_RESULT_ACCEPTED = 0
MAV_RESULT_FAILED = 4

# The C++ encoder (encode_command_long) packs param1 at payload bytes 0..4
# and the command id (u16) at bytes 28..30; the payload is 33 bytes.
_COMMAND_LONG_COMMAND_OFFSET = 28

# The C++ encoder (encode_velocity_setpoint) puts the FRD velocity command's
# vx at payload bytes 16..20 of SET_POSITION_TARGET_LOCAL_NED.
_VX_PAYLOAD_OFFSET = 16


def decode_commands(datagram: bytes) -> list[tuple[int, float]]:
    """Return (command id, param1) for every COMMAND_LONG in a datagram."""
    commands: list[tuple[int, float]] = []
    offset = 0
    while offset + 8 <= len(datagram):
        message_id, payload, total = _next_frame(datagram, offset)
        if message_id is None:
            break
        if message_id == COMMAND_LONG_ID and len(payload) >= 30:
            command_id = struct.unpack("<H", payload[_COMMAND_LONG_COMMAND_OFFSET : _COMMAND_LONG_COMMAND_OFFSET + 2])[
                0
            ]
            param1 = struct.unpack("<f", payload[0:4])[0]
            commands.append((command_id, param1))
        offset += total
    return commands


def decode_velocity_setpoints(datagram: bytes) -> list[float]:
    """Return the vx of every velocity setpoint in a datagram."""
    vx_values: list[float] = []
    offset = 0
    while offset + 8 <= len(datagram):
        message_id, payload, total = _next_frame(datagram, offset)
        if message_id is None:
            break
        if message_id == SET_POSITION_TARGET_LOCAL_NED_ID and len(payload) >= _VX_PAYLOAD_OFFSET + 4:
            vx_values.append(struct.unpack("<f", payload[_VX_PAYLOAD_OFFSET : _VX_PAYLOAD_OFFSET + 4])[0])
        offset += total
    return vx_values


def _next_frame(datagram: bytes, offset: int) -> tuple[int | None, bytes, int]:
    """Return (message id, payload, frame length) for the frame at offset."""
    magic = datagram[offset]
    if magic == 0xFD:  # MAVLink v2: 10-byte header, 3-byte message id
        header_size, id_width = 10, 3
    elif magic == 0xFE:  # MAVLink v1: 6-byte header, 1-byte message id
        header_size, id_width = 6, 1
    else:
        return None, b"", 0
    payload_length = datagram[offset + 1]
    total = header_size + payload_length + 2
    if offset + total > len(datagram):
        return None, b"", 0
    if id_width == 3:
        message_id = datagram[offset + 7] | (datagram[offset + 8] << 8) | (datagram[offset + 9] << 16)
    else:
        message_id = datagram[offset + 5]
    payload = datagram[offset + header_size : offset + header_size + payload_length]
    return message_id, payload, total
