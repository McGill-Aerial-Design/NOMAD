// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cstdint>
#include <optional>
#include <string>

namespace nomad::safety {

inline constexpr std::uint32_t kGuidedMode = 4;

enum class RejectReason {
    none,
    link,
    armed,
    mode,
    vio,
    nonfinite,
    invalid_limits,
};

struct VelocityLimits {
    float max_velocity_xy{2.0F};
    float max_velocity_z{1.0F};
    float max_yaw_rate{1.0F};
};

struct VelocityCommand {
    float vx{};
    float vy{};
    float vz{};
    float yaw_rate{};
};

struct FlightConditions {
    bool connected{false};
    bool heartbeat_fresh{false};
    bool armed{false};
    std::uint32_t custom_mode{};
    bool vio_healthy{false};
    bool vio_fresh{false};
    float vio_confidence{};
    float min_vio_confidence{0.3F};
};

struct VelocityDecision {
    bool allowed{false};
    RejectReason reason{RejectReason::none};
    std::string message;
    std::optional<VelocityCommand> setpoint;
};

VelocityDecision evaluate_velocity(
    const VelocityLimits& limits,
    const FlightConditions& conditions,
    const VelocityCommand& command);

}  // namespace nomad::safety
