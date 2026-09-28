// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
//
// Phase D fence operations of the MAVSDK-backed MavlinkConnection.
//
// MAVSDK's Geofence plugin transfers the fence as MAV_MISSION_TYPE_FENCE items
// and owns that handshake, so the plan upload and the readback that verifies it
// are implemented here. The removed transport also exposed single-vertex transfer
// (request one fence item, patch one fence item); MAVSDK moves whole polygons
// and the core only ever uploads a plan and then reads the autopilot's own copy
// back, so those stay fail-closed rather than inventing a partial transfer.
//
// A fence plan is not an enforced fence: ArduPilot only acts on it once
// FENCE_ENABLE is set, which Vehicle::verify_fence_uploaded reads back through
// read_param.

#include "mavsdk_mavlink_connection.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <future>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

namespace nomad::mavlink {
namespace {

std::uint16_t fence_command(const mavsdk::Geofence::Polygon &polygon) {
    const auto command = polygon.fence_type == mavsdk::Geofence::FenceType::Exclusion
                             ? MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION
                             : MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION;
    return static_cast<std::uint16_t>(command);
}

// Flatten the autopilot's polygons back into the item order the core uploaded:
// the sequence restarts at each polygon and every vertex carries its polygon's
// vertex count in param1, which is where ArduPilot reads it from.
std::vector<FencePlanItem> to_plan_items(const mavsdk::Geofence::GeofenceData &data) {
    std::vector<FencePlanItem> items;
    for (const auto &polygon : data.polygons) {
        const auto vertex_count = static_cast<float>(polygon.points.size());
        for (std::size_t index = 0; index < polygon.points.size(); ++index) {
            const auto &point = polygon.points[index];
            items.push_back(FencePlanItem{
                FencePoint{static_cast<float>(point.latitude_deg), static_cast<float>(point.longitude_deg)},
                static_cast<std::uint16_t>(index),
                fence_command(polygon),
                vertex_count,
            });
        }
    }
    return items;
}

} // namespace

bool MavsdkMavlinkConnection::upload_fence_plan(const std::vector<FencePlanItem> &items) {
    if (!admit_send()) {
        return false;
    }
    std::shared_lock lifetime_lock(plugin_lifetime_mutex_);
    if (!is_connected_unlocked() || !geofence_ || items.empty()) {
        return false;
    }
    mavsdk::Geofence::Polygon polygon{};
    polygon.fence_type = mavsdk::Geofence::FenceType::Inclusion;
    polygon.points.reserve(items.size());
    for (const auto &item : items) {
        polygon.points.push_back(mavsdk::Geofence::Point{static_cast<double>(item.point.latitude_deg),
                                                         static_cast<double>(item.point.longitude_deg)});
    }
    mavsdk::Geofence::GeofenceData data{};
    data.polygons.push_back(std::move(polygon));
    return geofence_->upload_geofence(data) == mavsdk::Geofence::Result::Success;
}

std::optional<std::vector<FencePlanItem>>
MavsdkMavlinkConnection::download_fence_plan(std::chrono::milliseconds timeout) {
    std::shared_lock lifetime_lock(plugin_lifetime_mutex_);
    if (!is_connected_unlocked() || !geofence_) {
        return std::nullopt;
    }
    using Transfer = std::pair<mavsdk::Geofence::Result, mavsdk::Geofence::GeofenceData>;
    auto promise = std::make_shared<std::promise<Transfer>>();
    auto future = promise->get_future();
    // The callback holds its own reference: a transfer that finishes after this
    // deadline must not write into a frame that has already returned.
    geofence_->download_geofence_async([promise](mavsdk::Geofence::Result result, mavsdk::Geofence::GeofenceData data) {
        promise->set_value({result, std::move(data)});
    });
    if (future.wait_for(timeout) != std::future_status::ready) {
        return std::nullopt;
    }
    const auto [result, data] = future.get();
    if (result != mavsdk::Geofence::Result::Success) {
        return std::nullopt;
    }
    return to_plan_items(data);
}

bool MavsdkMavlinkConnection::send_fence_point(const FencePoint & /*point*/, std::uint8_t /*index*/,
                                               std::uint8_t /*total*/) {
    return false;
}

bool MavsdkMavlinkConnection::request_fence_point(std::uint8_t /*index*/) {
    return false;
}

std::optional<FencePoint> MavsdkMavlinkConnection::wait_for_fence_point(std::chrono::milliseconds /*timeout*/) {
    return std::nullopt;
}

} // namespace nomad::mavlink
