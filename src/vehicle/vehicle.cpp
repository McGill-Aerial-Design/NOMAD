// SPDX-License-Identifier: Apache-2.0
// Vehicle lifecycle, the safety-gated command verbs, their state verification,
// and the shared command/acknowledgement exchange.
//
// Velocity control and its watchdog live in vehicle_velocity.cpp; the payload
// interlock in vehicle_payload.cpp; the fence upload/readback in
// vehicle_fence.cpp. The MAV_CMD ids and make_command are in command_ids.hpp.
#include "nomad/vehicle/vehicle.hpp"

#include "command_ids.hpp"

#include <array>
#include <chrono>
#include <cmath>
#include <optional>
#include <string>
#include <thread>
#include <utility>

namespace nomad::vehicle {
namespace {

constexpr auto kCommandTimeout = std::chrono::seconds(3);
constexpr auto kStateTimeout = std::chrono::seconds(10);
constexpr auto kTakeoffStateTimeout = std::chrono::seconds(30);
constexpr auto kNavigationStateTimeout = std::chrono::seconds(60);
constexpr auto kStatePollTimeout = std::chrono::milliseconds(500);
constexpr double kLocationToleranceDegrees = 0.00002;
constexpr float kAltitudeToleranceMeters = 2.0F;

CommandResult validate_command_acknowledgement(
    const mavlink::Command &command, const char *name,
    const std::optional<mavlink::CommandAck> &acknowledgement) {
    if (!acknowledgement.has_value()) {
        return {false, std::string(name) + " timed out waiting for acknowledgement"};
    }
    if (acknowledgement->command != command.id) {
        return {false, std::string(name) + " received an acknowledgement for another command"};
    }
    if (acknowledgement->result != kAcceptedResult) {
        return {false, std::string(name) + " rejected by ArduPilot"};
    }
    return {true, std::string(name) + " accepted"};
}

} // namespace

Vehicle::Vehicle(mavlink::MavlinkConnection &connection, VehicleConfig config)
    : connection_(connection),
      watchdog_policy_(config.watchdog),
      fence_policy_(std::move(config.fence)),
      velocity_limits_(config.velocity),
      position_freshness_timeout_(config.timeouts.position_freshness),
      vtol_takeoff_state_timeout_(config.timeouts.vtol_takeoff_state),
      transition_state_timeout_(config.timeouts.transition_state),
      fixed_wing_route_timeout_(config.timeouts.fixed_wing_route),
      fixed_wing_recovery_timeout_(config.timeouts.fixed_wing_recovery),
      transition_ready_dwell_(config.timeouts.transition_ready_dwell),
      quadplane_landing_timeout_(config.timeouts.quadplane_landing) {}

Vehicle::~Vehicle() {
    {
        std::lock_guard lock(velocity_mutex_);
        shutting_down_ = true;
        if (velocity_control_active_) {
            connection_.send_velocity({});
            velocity_control_active_ = false;
        }
    }
    velocity_condition_.notify_all();
    if (watchdog_thread_.joinable()) {
        watchdog_thread_.join();
    }
}

std::optional<telemetry::VehicleState> Vehicle::wait_for_state(std::chrono::milliseconds timeout) {
    if (!connection_.is_connected()) {
        return std::nullopt;
    }
    return connection_.wait_for_state(timeout);
}

std::optional<telemetry::VehicleState> Vehicle::wait_for_telemetry(
    std::chrono::milliseconds timeout, const std::function<bool(const telemetry::VehicleState &)> &is_complete) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    std::optional<telemetry::VehicleState> freshest;
    while (std::chrono::steady_clock::now() < deadline) {
        const auto state = connection_.wait_for_state(kStatePollTimeout);
        if (!state.has_value()) {
            continue;
        }
        if (is_complete(*state)) {
            return state;
        }
        freshest = state;
    }
    return freshest;
}

CommandResult Vehicle::arm() {
    const auto admission = require_operation(VehicleOperation::Arm);
    if (!admission.success) {
        return admission;
    }
    const auto result = send_command(make_command(kArmDisarmCommand, {1, 0, 0, 0, 0, 0, 0}), "arm");
    if (!result.success) {
        return result;
    }
    return wait_for_armed_state(true, "arm");
}

CommandResult Vehicle::disarm() {
    const auto admission = require_operation(VehicleOperation::Disarm);
    if (!admission.success) {
        return admission;
    }
    const auto result = send_command(make_command(kArmDisarmCommand), "disarm");
    if (!result.success) {
        return result;
    }
    return wait_for_armed_state(false, "disarm");
}

CommandResult Vehicle::set_mode(std::uint32_t custom_mode) {
    const auto admission = require_operation(VehicleOperation::SetMode);
    if (!admission.success) {
        return admission;
    }
    return send_mode_and_verify(custom_mode, "set mode");
}

CommandResult Vehicle::set_guided_mode() {
    const auto admission = require_operation(VehicleOperation::SetGuidedMode);
    if (!admission.success) {
        return admission;
    }
    const auto mode = telemetry::guided_mode_for(connection_.get_state().identity.aircraft_class);
    if (!mode.has_value()) {
        return {false, "guided mode is unavailable for an unknown or unsupported aircraft"};
    }
    return send_mode_and_verify(*mode, "set guided mode");
}

CommandResult Vehicle::takeoff(float altitude_m) {
    if (!std::isfinite(altitude_m) || altitude_m <= 0.0F) {
        return {false, "takeoff altitude must be finite and greater than zero"};
    }
    const auto admission = require_operation(VehicleOperation::Takeoff);
    if (!admission.success) {
        return admission;
    }
    const auto result = send_command(make_command(kTakeoffCommand, {0, 0, 0, 0, 0, 0, altitude_m}), "takeoff");
    if (!result.success) {
        return result;
    }
    return wait_for_altitude(altitude_m * 0.8F, "takeoff");
}

CommandResult Vehicle::goto_location(const Location &location) {
    if (!std::isfinite(location.latitude_deg) || location.latitude_deg < -90.0 || location.latitude_deg > 90.0) {
        return {false, "latitude must be finite and between -90 and 90 degrees"};
    }
    if (!std::isfinite(location.longitude_deg) || location.longitude_deg < -180.0 || location.longitude_deg > 180.0) {
        return {false, "longitude must be finite and between -180 and 180 degrees"};
    }
    if (!std::isfinite(location.altitude_m)) {
        return {false, "altitude must be finite"};
    }
    const auto fence_decision =
        safety::evaluate_global_position(fence_policy_, {location.latitude_deg, location.longitude_deg});
    if (!fence_decision.allowed) {
        return {false, fence_decision.message};
    }
    const auto admission = require_operation(VehicleOperation::GotoLocation);
    if (!admission.success) {
        return admission;
    }
    if (!connection_.is_connected()) {
        return {false, "not connected"};
    }
    if (!connection_.goto_location_relative(location.latitude_deg, location.longitude_deg, location.altitude_m,
                                            kCommandTimeout)) {
        return {false, "goto location command not accepted"};
    }
    return wait_for_location(location);
}

CommandResult Vehicle::land() {
    const auto admission = require_operation(VehicleOperation::Land);
    if (!admission.success) {
        return admission;
    }
    const auto aircraft_class = connection_.get_state().identity.aircraft_class;
    const auto result = send_command(make_command(kLandCommand), "land");
    if (!result.success) {
        return result;
    }
    return wait_for_mode(
        [aircraft_class](std::uint32_t mode) { return telemetry::is_landing_mode(aircraft_class, mode); }, "land");
}

CommandResult Vehicle::wait_until_disarmed(std::chrono::milliseconds timeout) {
    const auto verdict = [](const telemetry::VehicleState &state) -> std::optional<CommandResult> {
        if (state.armed) {
            return std::nullopt;
        }
        return CommandResult{true, "vehicle disarmed"};
    };
    return wait_for_state_until(timeout, "timed out waiting for vehicle to disarm", verdict);
}

CommandResult Vehicle::return_to_launch() {
    const auto admission = require_operation(VehicleOperation::ReturnToLaunch);
    if (!admission.success) {
        return admission;
    }
    const auto result = send_command(make_command(kReturnToLaunchCommand), "return to launch");
    if (!result.success) {
        return result;
    }
    const auto aircraft_class = connection_.get_state().identity.aircraft_class;
    return wait_for_mode(
        [aircraft_class](std::uint32_t mode) {
            return telemetry::is_return_to_launch_mode(aircraft_class, mode);
        },
        "return to launch");
}

CommandResult Vehicle::wait_for_state_until(std::chrono::milliseconds timeout, std::string timeout_message,
                                            const StateVerdict &verdict) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        const auto state = connection_.wait_for_state(kStatePollTimeout);
        if (!state.has_value()) {
            continue;
        }
        if (const auto decision = verdict(*state); decision.has_value()) {
            return *decision;
        }
    }
    return {false, std::move(timeout_message)};
}

bool Vehicle::position_is_stale(const telemetry::VehicleState &state) const {
    return std::chrono::steady_clock::now() - state.position_updated_at > position_freshness_timeout_;
}

CommandResult Vehicle::wait_for_armed_state(bool expected, const char *name) {
    const auto verdict = [expected, name](const telemetry::VehicleState &state) -> std::optional<CommandResult> {
        if (state.armed != expected) {
            return std::nullopt;
        }
        return CommandResult{true, std::string(name) + " verified"};
    };
    return wait_for_state_until(kStateTimeout,
                                std::string(name) + " acknowledgement received but state verification timed out",
                                verdict);
}

CommandResult Vehicle::wait_for_mode(std::uint32_t expected, const char *name) {
    return wait_for_mode([expected](std::uint32_t mode) { return mode == expected; }, name);
}

CommandResult Vehicle::send_mode_and_verify(std::uint32_t custom_mode, const char *name) {
    const auto result = send_command(
        make_command(kSetModeCommand, {1, static_cast<float>(custom_mode), 0, 0, 0, 0, 0}), name);
    if (!result.success) {
        return result;
    }
    return wait_for_mode(custom_mode, name);
}

CommandResult Vehicle::wait_for_mode(const std::function<bool(std::uint32_t)> &matches, const char *name) {
    const auto verdict = [&matches, name](const telemetry::VehicleState &state) -> std::optional<CommandResult> {
        if (!matches(state.custom_mode)) {
            return std::nullopt;
        }
        return CommandResult{true, std::string(name) + " verified"};
    };
    return wait_for_state_until(kStateTimeout,
                                std::string(name) + " acknowledgement received but mode verification timed out",
                                verdict);
}

CommandResult Vehicle::require_operation(VehicleOperation operation) const {
    const auto aircraft_class = connection_.get_state().identity.aircraft_class;
    if (supports_operation(aircraft_class, operation)) {
        return {true, "operation capability verified"};
    }
    return {false, std::string(operation_name(operation)) + " is not qualified for " +
                       std::string(telemetry::aircraft_class_name(aircraft_class))};
}

CommandResult Vehicle::wait_for_altitude(float minimum_altitude_m, const char *name) {
    const auto verdict = [this, minimum_altitude_m, name](const telemetry::VehicleState &state)
        -> std::optional<CommandResult> {
        if (!state.position_valid) {
            return std::nullopt;
        }
        // Altitude is derived from the same position feed, so a stale sample
        // must fail closed even when it happens to sit above the target.
        if (position_is_stale(state)) {
            return CommandResult{false, std::string(name) + " verification failed: position feed is stale"};
        }
        if (state.position.relative_altitude_m < minimum_altitude_m) {
            return std::nullopt;
        }
        return CommandResult{true, std::string(name) + " verified"};
    };
    return wait_for_state_until(
        kTakeoffStateTimeout, std::string(name) + " acknowledgement received but altitude verification timed out",
        verdict);
}

CommandResult Vehicle::wait_for_location(const Location &location) {
    const auto verdict = [this, &location](const telemetry::VehicleState &state) -> std::optional<CommandResult> {
        if (!state.position_valid) {
            return std::nullopt;
        }
        // A fresh heartbeat does not imply a fresh position: fail closed if the
        // GLOBAL_POSITION_INT feed has gone quiet rather than trusting a stale fix.
        if (position_is_stale(state)) {
            return CommandResult{false, "goto location verification failed: position feed is stale"};
        }
        const auto latitude_error = std::abs(state.position.latitude_deg - location.latitude_deg);
        const auto longitude_error = std::abs(state.position.longitude_deg - location.longitude_deg);
        // Location altitude is above home (the REPOSITION frame is
        // GLOBAL_RELATIVE_ALT_INT), so verify against relative altitude, as
        // wait_for_altitude does for takeoff.
        const auto altitude_error = std::abs(state.position.relative_altitude_m - location.altitude_m);
        const bool arrived = latitude_error <= kLocationToleranceDegrees &&
                             longitude_error <= kLocationToleranceDegrees &&
                             altitude_error <= kAltitudeToleranceMeters;
        if (!arrived) {
            return std::nullopt;
        }
        return CommandResult{true, "goto location verified"};
    };
    return wait_for_state_until(kNavigationStateTimeout,
                                "goto location acknowledgement received but position verification timed out",
                                verdict);
}

CommandResult Vehicle::send_command(const mavlink::Command &command, const char *name) {
    if (!connection_.is_connected()) {
        return {false, "not connected"};
    }

    const auto acknowledgement = connection_.send_command(command, kCommandTimeout);
    return validate_command_acknowledgement(command, name, acknowledgement);
}

CommandResult Vehicle::send_command(const mavlink::Command &command, const char *name,
                                    std::uint64_t expected_session_id) {
    if (!connection_.is_connected()) {
        return {false, "not connected"};
    }

    const auto acknowledgement = connection_.send_command(command, expected_session_id, kCommandTimeout);
    return validate_command_acknowledgement(command, name, acknowledgement);
}

} // namespace nomad::vehicle
