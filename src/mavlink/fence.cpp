// SPDX-License-Identifier: Apache-2.0
// Fence and fence-mission protocol: MAVLink wire codec plus the UDP connection
// conversations that upload and download a fence plan. Kept in one file so the
// protocol and transport codecs stay under the project source-size limit.
#include "nomad/mavlink/udp_connection.hpp"

#include "generated.hpp"
#include "nomad/mavlink/protocol.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

namespace nomad::mavlink {
namespace {

// Message identifiers used by the fence and fence-mission protocols. The
// values are the generated dialect constants, so they track the pinned
// firmware tables.
constexpr std::uint32_t kFenceFetchPointId = MAVLINK_MSG_ID_FENCE_FETCH_POINT;
constexpr std::uint32_t kFencePointId = MAVLINK_MSG_ID_FENCE_POINT;
constexpr std::uint32_t kFenceStatusId = MAVLINK_MSG_ID_FENCE_STATUS;
constexpr std::uint32_t kMissionItemIntId = MAVLINK_MSG_ID_MISSION_ITEM_INT;
constexpr std::uint32_t kMissionRequestId = MAVLINK_MSG_ID_MISSION_REQUEST;
constexpr std::uint32_t kMissionRequestIntId = MAVLINK_MSG_ID_MISSION_REQUEST_INT;
constexpr std::uint32_t kMissionRequestListId = MAVLINK_MSG_ID_MISSION_REQUEST_LIST;
constexpr std::uint32_t kMissionCountId = MAVLINK_MSG_ID_MISSION_COUNT;
constexpr std::uint32_t kMissionClearAllId = MAVLINK_MSG_ID_MISSION_CLEAR_ALL;
constexpr std::uint32_t kMissionAckId = MAVLINK_MSG_ID_MISSION_ACK;
constexpr std::uint8_t kMissionTypeFence = 1;

// GCS identity; must match the values in udp_connection.cpp.
constexpr std::uint8_t kSourceSystem = 255;
constexpr std::uint8_t kSourceComponent = 190;

} // namespace

std::vector<std::uint8_t> encode_fence_point(std::uint8_t sequence, std::uint8_t system_id, std::uint8_t component_id,
                                             std::uint8_t target_system, std::uint8_t target_component,
                                             std::uint8_t index, std::uint8_t total, const FencePoint &point) {
    mavlink_fence_point_t target{};
    target.lat = point.latitude_deg;
    target.lng = point.longitude_deg;
    target.idx = index;
    target.count = total;
    target.target_system = target_system;
    target.target_component = target_component;
    mavlink_message_t msg{};
    mavlink_msg_fence_point_encode(system_id, component_id, &msg, &target);
    return encode_message(sequence, system_id, component_id, kFencePointId,
                          payload_of(msg, MAVLINK_MSG_ID_FENCE_POINT_LEN), false)
        .value_or(std::vector<std::uint8_t>{});
}

std::optional<FencePoint> decode_fence_point(const Message &message) {
    if (message.message_id != kFencePointId || message.payload.size() < MAVLINK_MSG_ID_FENCE_POINT_MIN_LEN) {
        return std::nullopt;
    }
    const auto msg = to_generated_message(message);
    mavlink_fence_point_t point{};
    mavlink_msg_fence_point_decode(&msg, &point);
    return FencePoint{point.lat, point.lng};
}

std::vector<std::uint8_t> encode_mission_count(std::uint8_t sequence, std::uint8_t system_id,
                                                std::uint8_t component_id, std::uint8_t target_system,
                                                std::uint8_t target_component, std::uint16_t count,
                                                std::uint8_t mission_type) {
    // mission_type is a MAVLink 2 extension field and must be sent explicitly;
    // a truncated MAVLink 2 payload decodes the missing byte as zero (MISSION).
    mavlink_mission_count_t target{};
    target.count = count;
    target.target_system = target_system;
    target.target_component = target_component;
    target.mission_type = mission_type;
    mavlink_message_t msg{};
    mavlink_msg_mission_count_encode(system_id, component_id, &msg, &target);
    return encode_message(sequence, system_id, component_id, kMissionCountId,
                          payload_of(msg, MAVLINK_MSG_ID_MISSION_COUNT_LEN), true)
        .value_or(std::vector<std::uint8_t>{});
}

std::vector<std::uint8_t> encode_mission_item_int(std::uint8_t sequence, std::uint8_t system_id,
                                                  std::uint8_t component_id, std::uint8_t target_system,
                                                  std::uint8_t target_component, const FencePlanItem &item,
                                                  std::uint8_t mission_type) {
    mavlink_mission_item_int_t target{};
    target.param1 = item.param1;
    target.x = static_cast<std::int32_t>(std::llround(item.point.latitude_deg * 1.0e7));
    target.y = static_cast<std::int32_t>(std::llround(item.point.longitude_deg * 1.0e7));
    target.seq = item.sequence;
    target.command = item.command;
    target.target_system = target_system;
    target.target_component = target_component;
    target.frame = 5;  // MAV_FRAME_GLOBAL_INT
    // mission_type is a MAVLink 2 extension field and must be sent explicitly;
    // a truncated MAVLink 2 payload decodes the missing byte as zero (MISSION).
    target.mission_type = mission_type;
    mavlink_message_t msg{};
    mavlink_msg_mission_item_int_encode(system_id, component_id, &msg, &target);
    return encode_message(sequence, system_id, component_id, kMissionItemIntId,
                          payload_of(msg, MAVLINK_MSG_ID_MISSION_ITEM_INT_LEN), true)
        .value_or(std::vector<std::uint8_t>{});
}

std::vector<std::uint8_t> encode_mission_request_list(std::uint8_t sequence, std::uint8_t system_id,
                                                      std::uint8_t component_id, std::uint8_t target_system,
                                                      std::uint8_t target_component, std::uint8_t mission_type) {
    mavlink_mission_request_list_t target{};
    target.target_system = target_system;
    target.target_component = target_component;
    target.mission_type = mission_type;
    mavlink_message_t msg{};
    mavlink_msg_mission_request_list_encode(system_id, component_id, &msg, &target);
    return encode_message(sequence, system_id, component_id, kMissionRequestListId,
                          payload_of(msg, MAVLINK_MSG_ID_MISSION_REQUEST_LIST_LEN), true)
        .value_or(std::vector<std::uint8_t>{});
}

std::vector<std::uint8_t> encode_mission_request_int(std::uint8_t sequence, std::uint8_t system_id,
                                                     std::uint8_t component_id, std::uint8_t target_system,
                                                     std::uint8_t target_component, std::uint16_t index,
                                                     std::uint8_t mission_type) {
    mavlink_mission_request_int_t target{};
    target.seq = index;
    target.target_system = target_system;
    target.target_component = target_component;
    target.mission_type = mission_type;
    mavlink_message_t msg{};
    mavlink_msg_mission_request_int_encode(system_id, component_id, &msg, &target);
    return encode_message(sequence, system_id, component_id, kMissionRequestIntId,
                          payload_of(msg, MAVLINK_MSG_ID_MISSION_REQUEST_INT_LEN), true)
        .value_or(std::vector<std::uint8_t>{});
}

std::vector<std::uint8_t> encode_mission_clear_all(std::uint8_t sequence, std::uint8_t system_id,
                                                    std::uint8_t component_id, std::uint8_t target_system,
                                                    std::uint8_t target_component, std::uint8_t mission_type) {
    mavlink_mission_clear_all_t target{};
    target.target_system = target_system;
    target.target_component = target_component;
    target.mission_type = mission_type;
    mavlink_message_t msg{};
    mavlink_msg_mission_clear_all_encode(system_id, component_id, &msg, &target);
    return encode_message(sequence, system_id, component_id, kMissionClearAllId,
                          payload_of(msg, MAVLINK_MSG_ID_MISSION_CLEAR_ALL_LEN), true)
        .value_or(std::vector<std::uint8_t>{});
}

std::optional<std::uint16_t> decode_mission_sequence(const Message &message) {
    // ArduPilot answers a MISSION_COUNT with either MISSION_REQUEST_INT or
    // MISSION_REQUEST depending on the link; both carry the requested seq.
    if (message.message_id == kMissionRequestIntId &&
        message.payload.size() >= MAVLINK_MSG_ID_MISSION_REQUEST_INT_MIN_LEN) {
        const auto msg = to_generated_message(message);
        mavlink_mission_request_int_t request{};
        mavlink_msg_mission_request_int_decode(&msg, &request);
        return request.seq;
    }
    if (message.message_id == kMissionRequestId && message.payload.size() >= MAVLINK_MSG_ID_MISSION_REQUEST_MIN_LEN) {
        const auto msg = to_generated_message(message);
        mavlink_mission_request_t request{};
        mavlink_msg_mission_request_decode(&msg, &request);
        return request.seq;
    }
    return std::nullopt;
}

std::optional<std::uint16_t> decode_mission_count(const Message &message) {
    if (message.message_id != kMissionCountId || message.payload.size() < MAVLINK_MSG_ID_MISSION_COUNT_MIN_LEN) {
        return std::nullopt;
    }
    const auto msg = to_generated_message(message);
    mavlink_mission_count_t count{};
    mavlink_msg_mission_count_decode(&msg, &count);
    return count.count;
}

std::optional<std::uint8_t> decode_mission_ack(const Message &message) {
    // MISSION_ACK payload: target_system, target_component, type, mission_type.
    if (message.message_id != kMissionAckId || message.payload.size() < MAVLINK_MSG_ID_MISSION_ACK_MIN_LEN) {
        return std::nullopt;
    }
    const auto msg = to_generated_message(message);
    mavlink_mission_ack_t ack{};
    mavlink_msg_mission_ack_decode(&msg, &ack);
    return ack.type;
}

std::optional<FenceStatus> decode_fence_status(const Message &message) {
    if (message.message_id != kFenceStatusId || message.payload.size() < MAVLINK_MSG_ID_FENCE_STATUS_MIN_LEN) {
        return std::nullopt;
    }
    const auto msg = to_generated_message(message);
    mavlink_fence_status_t status{};
    mavlink_msg_fence_status_decode(&msg, &status);
    return FenceStatus{
        status.breach_status,
        status.breach_count,
        status.breach_type,
        status.breach_time,
    };
}

bool UdpMavlinkConnection::send_fence_point(const FencePoint &point, std::uint8_t index, std::uint8_t total) {
    std::lock_guard lock(receive_mutex_);
    if (!is_connected() || target_system_ == 0 || !has_peer()) {
        return false;
    }
    const auto frame = encode_fence_point(sequence_++, kSourceSystem, kSourceComponent, target_system_,
                                          target_component_, index, total, point);
    return !frame.empty() && send_frame(frame) == frame.size();
}

bool UdpMavlinkConnection::request_fence_point(std::uint8_t index) {
    std::lock_guard lock(receive_mutex_);
    if (!is_connected() || target_system_ == 0 || !has_peer()) {
        return false;
    }
    mavlink_fence_fetch_point_t target{};
    target.target_system = target_system_;
    target.target_component = target_component_;
    target.idx = index;
    mavlink_message_t msg{};
    mavlink_msg_fence_fetch_point_encode(kSourceSystem, kSourceComponent, &msg, &target);
    const auto frame = encode_message(sequence_++, kSourceSystem, kSourceComponent, kFenceFetchPointId,
                                      payload_of(msg, MAVLINK_MSG_ID_FENCE_FETCH_POINT_LEN), false);
    return frame.has_value() && send_frame(*frame) == frame->size();
}

bool UdpMavlinkConnection::upload_fence_plan(const std::vector<FencePlanItem> &items) {
    std::lock_guard lock(receive_mutex_);
    if (!is_connected() || target_system_ == 0 || !has_peer() || items.empty() || items.size() > 255) {
        return false;
    }
    const auto count = static_cast<std::uint16_t>(items.size());
    const auto count_frame = encode_mission_count(sequence_++, kSourceSystem, kSourceComponent, target_system_,
                                                  target_component_, count, kMissionTypeFence);
    if (send_frame(count_frame) != count_frame.size()) {
        return false;
    }
    std::size_t next_item = 0;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    while (next_item < items.size() && std::chrono::steady_clock::now() < deadline) {
        const auto message = receive_message_locked(std::chrono::milliseconds(1000));
        if (!message.has_value() || message->system_id != target_system_) {
            continue;
        }
        const auto requested = decode_mission_sequence(*message);
        if (requested.has_value() && *requested < items.size()) {
            const auto frame = encode_mission_item_int(sequence_++, kSourceSystem, kSourceComponent, target_system_,
                                                       target_component_, items[*requested], kMissionTypeFence);
            if (send_frame(frame) != frame.size()) {
                return false;
            }
            next_item = static_cast<std::size_t>(*requested) + 1;
            continue;
        }
        const auto acknowledgement = decode_mission_ack(*message);
        if (acknowledgement.has_value()) {
            return *acknowledgement == 0 && next_item == items.size();
        }
    }
    while (std::chrono::steady_clock::now() < deadline) {
        const auto message = receive_message_locked(std::chrono::milliseconds(500));
        if (!message.has_value() || message->system_id != target_system_) {
            continue;
        }
        const auto acknowledgement = decode_mission_ack(*message);
        if (acknowledgement.has_value()) {
            return *acknowledgement == 0;
        }
    }
    return false;
}

std::optional<std::vector<FencePlanItem>> UdpMavlinkConnection::download_fence_plan(std::chrono::milliseconds timeout) {
    std::lock_guard lock(receive_mutex_);
    if (!is_connected() || target_system_ == 0 || !has_peer()) {
        return std::nullopt;
    }
    const auto request = encode_mission_request_list(sequence_++, kSourceSystem, kSourceComponent, target_system_,
                                                     target_component_, kMissionTypeFence);
    if (send_frame(request) != request.size()) {
        return std::nullopt;
    }
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    std::optional<std::uint16_t> count;
    while (std::chrono::steady_clock::now() < deadline && !count.has_value()) {
        const auto message = receive_message_locked(std::chrono::milliseconds(500));
        if (message.has_value() && message->system_id == target_system_) {
            count = decode_mission_count(*message);
        }
    }
    if (!count.has_value() || *count > 255) {
        return std::nullopt;
    }
    std::vector<FencePlanItem> items;
    for (std::uint16_t index = 0; index < *count; ++index) {
        const auto item_request = encode_mission_request_int(sequence_++, kSourceSystem, kSourceComponent,
                                                             target_system_, target_component_, index,
                                                             kMissionTypeFence);
        if (send_frame(item_request) != item_request.size()) {
            return std::nullopt;
        }
        const auto item_deadline = std::chrono::steady_clock::now() + timeout;
        while (std::chrono::steady_clock::now() < item_deadline) {
            const auto message = receive_message_locked(std::chrono::milliseconds(500));
            if (!message.has_value() || message->system_id != target_system_ || message->message_id != kMissionItemIntId ||
                message->payload.size() < MAVLINK_MSG_ID_MISSION_ITEM_INT_LEN) {
                continue;
            }
            const auto msg = to_generated_message(*message);
            mavlink_mission_item_int_t item{};
            mavlink_msg_mission_item_int_decode(&msg, &item);
            const auto latitude = static_cast<float>(static_cast<double>(item.x) / 1e7);
            const auto longitude = static_cast<float>(static_cast<double>(item.y) / 1e7);
            items.push_back(FencePlanItem{{latitude, longitude}, index, 0});
            break;
        }
    }
    return items.size() == *count ? std::optional<std::vector<FencePlanItem>>(std::move(items)) : std::nullopt;
}

std::optional<FencePoint> UdpMavlinkConnection::wait_for_fence_point(std::chrono::milliseconds timeout) {
    std::lock_guard lock(receive_mutex_);
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
            deadline - std::chrono::steady_clock::now());
        const auto message = receive_message_locked((std::max)(std::chrono::milliseconds(1), remaining));
        if (!message.has_value() || message->system_id != target_system_) {
            continue;
        }
        const auto point = decode_fence_point(*message);
        if (point.has_value()) {
            return point;
        }
    }
    return std::nullopt;
}

} // namespace nomad::mavlink
