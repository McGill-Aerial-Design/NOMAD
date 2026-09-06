// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "nomad/mavlink/connection.hpp"

#include <chrono>
#include <cstdint>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

class FakeConnection final : public nomad::mavlink::MavlinkConnection {
  public:
    bool connect() override {
        connected = true;
        return true;
    }

    void disconnect() override {
        event_log.push_back("disconnect");
        connected = false;
    }

    bool is_connected() const override {
        return connected;
    }

    std::optional<nomad::mavlink::Heartbeat> wait_for_heartbeat(std::chrono::milliseconds) override {
        return nomad::mavlink::Heartbeat{1, 1, 0, 2, 3, 0};
    }

    std::optional<nomad::telemetry::VehicleState> wait_for_state(std::chrono::milliseconds) override {
        return get_state();
    }

    nomad::telemetry::VehicleState get_state() const override {
        std::lock_guard lock(state_mutex);
        return *state;
    }

    std::optional<nomad::mavlink::CommandAck> send_command(const nomad::mavlink::Command &command,
                                                           std::chrono::milliseconds) override {
        std::lock_guard lock(state_mutex);
        last_command = command;
        command_history.push_back(command);
        update_state_for_command(command);
        if (!command_send_results.empty()) {
            const bool result = command_send_results.front();
            command_send_results.erase(command_send_results.begin());
            if (!result) {
                return std::nullopt;
            }
        }
        if (!acknowledgement.has_value()) {
            return std::nullopt;
        }
        return nomad::mavlink::CommandAck{command.id, acknowledgement->result};
    }

    bool send_velocity(const nomad::mavlink::VelocitySetpoint &setpoint) override {
        event_log.push_back("send_velocity");
        last_velocity = setpoint;
        velocity_send_count += 1;
        if (!velocity_send_results.empty()) {
            const bool result = velocity_send_results.front();
            velocity_send_results.erase(velocity_send_results.begin());
            return result;
        }
        return true;
    }

    bool is_velocity_active() const override {
        return last_velocity.vx != 0.0F || last_velocity.vy != 0.0F || last_velocity.vz != 0.0F ||
               last_velocity.yaw_rate != 0.0F;
    }

    bool request_data_stream(std::uint8_t, std::uint16_t) override {
        return true;
    }

    bool send_fence_point(const nomad::mavlink::FencePoint &point, std::uint8_t index, std::uint8_t total) override {
        fence_points.push_back(point);
        fence_indices.push_back(index);
        fence_total = total;
        return fence_send_results.empty() || take_fence_send_result();
    }

    bool request_fence_point(std::uint8_t index) override {
        requested_fence_indices.push_back(index);
        return true;
    }

    bool upload_fence_plan(const std::vector<nomad::mavlink::FencePlanItem> &items) override {
        uploaded_fence_plan = items;
        return fence_plan_upload_result;
    }

    std::optional<std::vector<nomad::mavlink::FencePlanItem>> download_fence_plan(std::chrono::milliseconds) override {
        if (!fence_plan_download_result) {
            return std::nullopt;
        }
        return uploaded_fence_plan;
    }

    std::optional<nomad::mavlink::FencePoint> wait_for_fence_point(std::chrono::milliseconds) override {
        if (readback_index >= fence_points.size()) {
            return std::nullopt;
        }
        return fence_points[readback_index++];
    }

    std::optional<float> read_param(const std::string &param_id, std::chrono::milliseconds) override {
        const auto found = parameters.find(param_id);
        if (found == parameters.end()) {
            return std::nullopt;
        }
        return found->second;
    }

    bool connected{false};
    std::optional<nomad::telemetry::VehicleState> state{
        nomad::telemetry::VehicleState{true, true, false, 1, 1, 4},
    };
    nomad::mavlink::VelocitySetpoint last_velocity{};
    int velocity_send_count{0};
    nomad::mavlink::Command last_command{};
    std::optional<nomad::mavlink::CommandAck> acknowledgement{
        nomad::mavlink::CommandAck{0, 0},
    };
    std::vector<nomad::mavlink::Command> command_history;
    std::vector<bool> velocity_send_results;
    std::vector<bool> command_send_results;
    std::vector<nomad::mavlink::FencePoint> fence_points;
    std::vector<std::uint8_t> fence_indices;
    std::uint8_t fence_total{0};
    std::vector<std::uint8_t> requested_fence_indices;
    std::size_t readback_index{0};
    std::vector<bool> fence_send_results;
    std::vector<nomad::mavlink::FencePlanItem> uploaded_fence_plan;
    bool fence_plan_upload_result{true};
    bool fence_plan_download_result{true};
    std::map<std::string, float> parameters;
    std::vector<std::string> event_log;

    void set_connected(bool value) {
        std::lock_guard lock(state_mutex);
        state->connected = value;
        state->heartbeat_fresh = value;
    }

    void set_mode(std::uint32_t mode) {
        std::lock_guard lock(state_mutex);
        state->custom_mode = mode;
    }

  private:
    mutable std::mutex state_mutex;

    bool take_fence_send_result() {
        const bool result = fence_send_results.front();
        fence_send_results.erase(fence_send_results.begin());
        return result;
    }

    void update_state_for_command(const nomad::mavlink::Command &command) {
        if (command.id == 400) {
            state->armed = command.parameters[0] > 0.0F;
        } else if (command.id == 176) {
            state->custom_mode = static_cast<std::uint32_t>(command.parameters[1]);
        } else if (command.id == 22) {
            state->position_valid = true;
            state->position.relative_altitude_m = command.parameters[6];
        } else if (command.id == 21) {
            state->custom_mode = 9;
            state->armed = false;
        } else if (command.id == 20) {
            state->custom_mode = 6;
        } else if (command.id == 192) {
            state->position_valid = true;
            state->position.latitude_deg = command.parameters[4];
            state->position.longitude_deg = command.parameters[5];
            // REPOSITION altitude is above home; the vehicle telemetry carries
            // it in relative_altitude_m and verification compares that frame.
            state->position.relative_altitude_m = command.parameters[6];
        }
    }
};
