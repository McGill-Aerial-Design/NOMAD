// SPDX-License-Identifier: Apache-2.0
// Fence upload and verification: turn a configured polygon into ArduPilot fence
// mission items, then prove the autopilot actually holds them instead of
// trusting the upload acknowledgement.
//
// Split from vehicle.cpp. The MAV_CMD ids and make_command are in command_ids.hpp.
#include "nomad/vehicle/vehicle.hpp"

#include "command_ids.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <vector>

namespace nomad::vehicle {
namespace {

constexpr std::size_t kMinimumFencePoints = 3;
constexpr std::size_t kMaximumFencePoints = 255;
constexpr float kFenceCoordinateToleranceDegrees = 0.00001F;
constexpr auto kFenceReadbackTimeout = std::chrono::seconds(10);

bool is_valid_coordinate(double latitude_deg, double longitude_deg) {
    if (!std::isfinite(latitude_deg) || !std::isfinite(longitude_deg)) {
        return false;
    }
    return latitude_deg >= -90.0 && latitude_deg <= 90.0 && longitude_deg >= -180.0 && longitude_deg <= 180.0;
}

} // namespace

CommandResult Vehicle::upload_fence(const std::vector<safety::GlobalPoint> &boundary) {
    if (boundary.size() < kMinimumFencePoints || boundary.size() > kMaximumFencePoints) {
        return {false, "fence boundary must contain between 3 and 255 points"};
    }
    const auto admission = require_operation(VehicleOperation::FenceConfiguration);
    if (!admission.success) {
        return admission;
    }

    std::vector<mavlink::FencePlanItem> items;
    items.reserve(boundary.size());
    for (std::size_t index = 0; index < boundary.size(); ++index) {
        const auto &point = boundary[index];
        if (!is_valid_coordinate(point.latitude_deg, point.longitude_deg)) {
            return {false, "fence point coordinates are invalid"};
        }
        const mavlink::FencePoint fence_point{
            static_cast<float>(point.latitude_deg),
            static_cast<float>(point.longitude_deg),
        };
        // ArduPilot imports one polygon per command group and reads the group's
        // vertex count from param1 on every vertex item.
        items.push_back({fence_point, static_cast<std::uint16_t>(index), kFenceVertexCommand,
                         static_cast<float>(boundary.size())});
    }
    if (!connection_.upload_fence_plan(items)) {
        return {false, "fence plan upload failed"};
    }
    return {true, "fence uploaded"};
}

CommandResult Vehicle::verify_fence_uploaded(const std::vector<safety::GlobalPoint> &expected_boundary) {
    if (expected_boundary.size() < kMinimumFencePoints || expected_boundary.size() > kMaximumFencePoints) {
        return {false, "expected fence boundary size is invalid"};
    }
    const auto admission = require_operation(VehicleOperation::FenceConfiguration);
    if (!admission.success) {
        return admission;
    }
    // A plan on the autopilot is not a fence: ArduPilot only enforces it when
    // FENCE_ENABLE is set. Read the parameter back as authoritative state.
    const auto fence_enable = connection_.read_param("FENCE_ENABLE", kFenceReadbackTimeout);
    if (!fence_enable.has_value()) {
        return {false, "could not read FENCE_ENABLE"};
    }
    if (std::abs(*fence_enable - 1.0F) > 0.01F) {
        return {false, "fence is not enabled (FENCE_ENABLE != 1)"};
    }
    const auto downloaded = connection_.download_fence_plan(kFenceReadbackTimeout);
    if (!downloaded.has_value() || downloaded->size() != expected_boundary.size()) {
        return {false, "fence plan readback failed"};
    }
    for (std::size_t index = 0; index < expected_boundary.size(); ++index) {
        const auto expected = expected_boundary[index];
        const auto &actual = downloaded->at(index).point;
        if (std::abs(actual.latitude_deg - expected.latitude_deg) > kFenceCoordinateToleranceDegrees ||
            std::abs(actual.longitude_deg - expected.longitude_deg) > kFenceCoordinateToleranceDegrees) {
            return {false, "fence readback does not match the uploaded boundary"};
        }
    }
    return {true, "fence upload verified"};
}

} // namespace nomad::vehicle
