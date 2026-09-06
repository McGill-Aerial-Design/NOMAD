// SPDX-License-Identifier: Apache-2.0
#pragma once

// Bridge between NOMAD's Message type and the generated MAVLink C headers.
// Not a public API; do not include from public headers.
//
// The generated headers come from scripts/dev/generate_mavlink.py, which runs
// the pinned third_party/ardupilot-mavlink submodule's own mavgen — the exact
// dialect the ArduPilot firmware compiles against. Message ids, crc_extras,
// lengths, and field layouts are generated, never hand-maintained.
#include "nomad/mavlink/connection.hpp"

// The dialect headers refuse to be included directly; they expect the protocol
// defaults that each dialect's mavlink.h sets, starting with MAVLINK_H.
#define MAVLINK_H
#define MAVLINK_STX 253
#define MAVLINK_ENDIAN MAVLINK_LITTLE_ENDIAN
#define MAVLINK_ALIGNED_FIELDS 1
#define MAVLINK_CRC_EXTRA 1
#define MAVLINK_COMMAND_24BIT 1

// Generated C headers trigger MSVC noise (nameless unions, double-to-float in
// mavlink_conversions.h); silence it for these includes only.
#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable : 4201 4244 4701 4703 4706)
#endif

// ardupilotmega.h must come first: it is the only header whose
// MAVLINK_MESSAGE_CRCS table covers every message NOMAD uses (the other
// dialect tables skip FENCE_POINT/FENCE_FETCH_POINT, and each table is
// #ifndef-guarded so the first include wins).
#include "ardupilotmega/ardupilotmega.h"  // full merged CRC table, fence messages
#include "common/common.h"                // telemetry, commands, missions, FENCE_STATUS
#include "standard/standard.h"            // GLOBAL_POSITION_INT
#include "minimal/minimal.h"              // HEARTBEAT

#if defined(_MSC_VER)
#pragma warning(pop)
#endif

#include <cstdint>
#include <cstring>
#include <vector>

namespace nomad::mavlink {

inline mavlink_message_t to_generated_message(const Message &message) {
    mavlink_message_t msg{};
    msg.msgid = message.message_id;
    msg.sysid = message.system_id;
    msg.compid = message.component_id;
    msg.len = static_cast<std::uint8_t>(message.payload.size());
    std::memcpy(_MAV_PAYLOAD_NON_CONST(&msg), message.payload.data(), message.payload.size());
    return msg;
}

// The generated encoders trim trailing zero bytes (MAVLink2 channel behavior)
// and finalize then stores the checksum bytes right after the payload, so
// msg.len is the only safe length. The trimmed tail is zero by construction,
// so the payload is padded back to the message's full generated length — NOMAD
// frames messages itself and keeps full-length payloads, matching the verified
// wire format and MAVLink1 framing. Callers pass the per-message LEN constant.
inline std::vector<std::uint8_t> payload_of(const mavlink_message_t &msg, std::uint8_t full_length) {
    const auto *begin = reinterpret_cast<const std::uint8_t *>(_MAV_PAYLOAD(&msg));
    std::vector<std::uint8_t> payload(begin, begin + msg.len);
    payload.resize(full_length, 0);
    return payload;
}

} // namespace nomad::mavlink
