// SPDX-License-Identifier: Apache-2.0
// Bounded fixed-wing recovery to one explicit GUIDED point.

#include "nomad/vehicle/vehicle.hpp"

#include "command_ids.hpp"
#include "geo.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <optional>
#include <string>

namespace nomad::vehicle {
namespace {

using Clock = std::chrono::steady_clock;

constexpr double kArrivalRadiusMeters = 45.0;
constexpr double kRequiredProgressMeters = 10.0;
constexpr float kAltitudeToleranceMeters = 5.0F;
constexpr float kLoiterRadiusMeters = 30.0F;
constexpr auto kCommandTimeout = std::chrono::seconds(3);
constexpr auto kPollTimeout = std::chrono::milliseconds(50);

std::optional<std::string> validate_point(const RecoveryPoint &point,
                                          const safety::GlobalFencePolicy &fence_policy) {
    if (!std::isfinite(point.latitude_deg) || point.latitude_deg < -90.0 || point.latitude_deg > 90.0 ||
        !std::isfinite(point.longitude_deg) || point.longitude_deg < -180.0 || point.longitude_deg > 180.0 ||
        (point.latitude_deg == 0.0 && point.longitude_deg == 0.0)) {
        return "recovery coordinates are missing or invalid";
    }
    if (!std::isfinite(point.relative_altitude_m) || point.relative_altitude_m < 2.0F ||
        point.relative_altitude_m > 100.0F) {
        return "recovery relative altitude must be between 2 and 100 m";
    }
    const auto fence = safety::evaluate_global_position(fence_policy, {point.latitude_deg, point.longitude_deg});
    if (!fence.allowed) {
        return fence.message;
    }
    return {};
}

double distance_to_point(const telemetry::VehicleState &state, const RecoveryPoint &point) {
    return detail::distance_m(state.position.latitude_deg, state.position.longitude_deg,
                              point.latitude_deg, point.longitude_deg);
}

} // namespace

CommandResult Vehicle::fixed_wing_recovery(const RecoveryPoint &point) {
    const auto admission = require_operation(VehicleOperation::FixedWingRecovery);
    if (!admission.success) {
        return admission;
    }
    if (const auto error = validate_point(point, fence_policy_); error.has_value()) {
        return {false, "fixed-wing recovery rejected: " + *error};
    }
    if (fixed_wing_recovery_timeout_ <= std::chrono::milliseconds::zero()) {
        return {false, "fixed-wing recovery rejected: timeout must be positive"};
    }
    const auto deadline = Clock::now() + fixed_wing_recovery_timeout_;
    const auto state = connection_.get_state();
    if (const auto error = fixed_wing_route_state_error(state, state.session_id, false); error.has_value()) {
        return {false, "fixed-wing recovery rejected: " + *error};
    }
    if (distance_to_point(state, point) <= kArrivalRadiusMeters + kRequiredProgressMeters) {
        return {false, "fixed-wing recovery rejected: target is too close to prove new navigation"};
    }

    const auto guided_mode = connection_.read_param(
        "Q_GUIDED_MODE", std::min(fixed_wing_recovery_timeout_,
                                   std::chrono::duration_cast<std::chrono::milliseconds>(kCommandTimeout)));
    if (!guided_mode.has_value() || *guided_mode != 0.0F) {
        return {false, "fixed-wing recovery rejected: Q_GUIDED_MODE=0 readback is required"};
    }
    const auto ready = connection_.get_state();
    if (const auto error = fixed_wing_route_state_error(ready, state.session_id, false); error.has_value()) {
        return {false, "fixed-wing recovery rejected after parameter readback: " + *error};
    }
    if (distance_to_point(ready, point) <= kArrivalRadiusMeters + kRequiredProgressMeters) {
        return {false, "fixed-wing recovery rejected: target became too close before transmission"};
    }

    const auto command_budget = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - Clock::now());
    if (command_budget <= std::chrono::milliseconds::zero()) {
        return {false, "fixed-wing recovery timed out before transmission"};
    }
    const auto ack = connection_.send_fixed_wing_waypoint(
        {point.latitude_deg, point.longitude_deg, point.relative_altitude_m, kLoiterRadiusMeters},
        ready.session_id,
        std::min(command_budget, std::chrono::duration_cast<std::chrono::milliseconds>(kCommandTimeout)));
    if (!ack.has_value()) {
        return {false, "fixed-wing recovery received no command ACK"};
    }
    if (ack->command != kDoRepositionCommand || ack->result != kAcceptedResult) {
        return {false, "fixed-wing recovery command was rejected by ArduPilot"};
    }
    const auto ack_received_at = Clock::now();
    const auto ack_state = connection_.get_state();
    if (const auto error = fixed_wing_route_state_error(ack_state, ready.session_id, false); error.has_value()) {
        return {false, "fixed-wing recovery verification failed after ACK: " + *error};
    }
    const auto boundary = std::max(ack_received_at, ack_state.position_updated_at);
    const double ack_distance = distance_to_point(ack_state, point);
    while (Clock::now() < deadline) {
        const auto update = connection_.wait_for_state(kPollTimeout);
        const auto sample = update.value_or(connection_.get_state());
        if (const auto error = fixed_wing_route_state_error(sample, ready.session_id, false); error.has_value()) {
            return {false, "fixed-wing recovery verification failed: " + *error};
        }
        if (sample.position_updated_at <= boundary) {
            continue;
        }
        const double remaining_distance = distance_to_point(sample, point);
        const float altitude_error = std::abs(sample.position.relative_altitude_m - point.relative_altitude_m);
        if (ack_distance - remaining_distance >= kRequiredProgressMeters &&
            remaining_distance <= kArrivalRadiusMeters &&
            altitude_error <= kAltitudeToleranceMeters) {
            return {true, "fixed-wing recovery verified: recovery region reached"};
        }
    }
    return {false, "fixed-wing recovery ACK received but recovery-region verification timed out"};
}

} // namespace nomad::vehicle
