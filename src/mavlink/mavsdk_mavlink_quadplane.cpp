// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
//
// Pinned QuadPlane telemetry and session-bound command support.

#include "mavsdk_mavlink_connection.hpp"

#include "mavsdk_command_ids.hpp"
#include "mavsdk_vtol_state.hpp"

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>

namespace nomad::mavlink {

void MavsdkMavlinkConnection::observe_vtol_state(mavsdk::Telemetry::VtolState state) {
    ObservationUpdate update(observation_mutex_, observation_changed_);
    state_.vtol_state = to_nomad_vtol_state(state);
    state_.vtol_state_valid = state_.vtol_state != telemetry::VtolState::Undefined;
    state_.vtol_state_updated_at = ObservationClock::now();
}

void MavsdkMavlinkConnection::observe_landed_state(mavsdk::Telemetry::LandedState state) {
    ObservationUpdate update(observation_mutex_, observation_changed_);
    state_.landed_state = static_cast<telemetry::LandedState>(state);
    state_.landed_state_valid = state != mavsdk::Telemetry::LandedState::Unknown;
    state_.landed_state_updated_at = ObservationClock::now();
}

std::optional<AutopilotVersion>
MavsdkMavlinkConnection::read_autopilot_version(std::chrono::milliseconds timeout) {
    std::shared_lock lifetime_lock(plugin_lifetime_mutex_);
    if (!is_connected_unlocked() || !passthrough_ || timeout <= std::chrono::milliseconds::zero()) {
        return std::nullopt;
    }

    struct VersionWaitState {
        std::mutex mutex;
        std::condition_variable changed;
        std::optional<AutopilotVersion> received;
    };
    const auto version_state = std::make_shared<VersionWaitState>();
    const auto expected_system_id = target_system_;
    const auto expected_component_id = target_component_;
    const auto message_handle = passthrough_->subscribe_message(
        MAVLINK_MSG_ID_AUTOPILOT_VERSION,
        [version_state, expected_system_id, expected_component_id](const mavlink_message_t &message) {
            if (message.sysid != expected_system_id || message.compid != expected_component_id) {
                return;
            }
            mavlink_autopilot_version_t wire{};
            mavlink_msg_autopilot_version_decode(&message, &wire);
            AutopilotVersion version;
            version.major = static_cast<std::uint8_t>((wire.flight_sw_version >> 24U) & 0xffU);
            version.minor = static_cast<std::uint8_t>((wire.flight_sw_version >> 16U) & 0xffU);
            version.patch = static_cast<std::uint8_t>((wire.flight_sw_version >> 8U) & 0xffU);
            for (const auto value : wire.flight_custom_version) {
                if (value == 0) {
                    break;
                }
                version.git_hash.push_back(static_cast<char>(value));
            }
            {
                std::lock_guard lock(version_state->mutex);
                version_state->received = std::move(version);
            }
            version_state->changed.notify_all();
        });

    const auto deadline = ObservationClock::now() + timeout;
    const auto request_budget = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::seconds(2));
    const auto request_timeout = (std::min)(timeout, request_budget);
    mavsdk::MavlinkPassthrough::CommandLong request{};
    request.target_sysid = target_system_;
    request.target_compid = target_component_;
    request.command = kRequestMessageCommand;
    request.param1 = static_cast<float>(MAVLINK_MSG_ID_AUTOPILOT_VERSION);
    const auto result = passthrough_->send_command_long(request, mavsdk::OperationOptions{request_timeout});
    const auto result_code = mavsdk_command_result_code(result);
    if (!result_code || *result_code != 0) {
        passthrough_->unsubscribe_message(MAVLINK_MSG_ID_AUTOPILOT_VERSION, message_handle);
        return std::nullopt;
    }

    {
        std::unique_lock lock(version_state->mutex);
        version_state->changed.wait_until(lock, deadline, [&] { return version_state->received.has_value(); });
    }
    passthrough_->unsubscribe_message(MAVLINK_MSG_ID_AUTOPILOT_VERSION, message_handle);
    std::lock_guard lock(version_state->mutex);
    return version_state->received;
}

std::optional<CommandAck> MavsdkMavlinkConnection::send_command(const Command &command,
                                                                std::uint64_t expected_session_id,
                                                                std::chrono::milliseconds timeout) {
    const auto admission = capture_transmission_admission();
    std::shared_lock lifetime_lock(plugin_lifetime_mutex_);
    if (expected_session_id == 0 || !is_connected_unlocked() || !passthrough_ ||
        timeout <= std::chrono::milliseconds::zero()) {
        return std::nullopt;
    }
    {
        std::lock_guard lock(observation_mutex_);
        if (state_.session_id != expected_session_id || !state_locked().connected) {
            return std::nullopt;
        }
    }
    const auto result = send_long(command, timeout, admission);
    if (result == mavsdk::MavlinkPassthrough::Result::CommandAdmissionCancelled) {
        return CommandAck{command.id, 0, CommandAck::Status::AdmissionCancelled};
    }
    const auto code = mavsdk_command_result_code(result);
    if (!code.has_value()) {
        return std::nullopt;
    }
    {
        std::lock_guard lock(observation_mutex_);
        if (state_.session_id != expected_session_id) {
            return std::nullopt;
        }
    }
    return CommandAck{command.id, *code};
}

} // namespace nomad::mavlink
