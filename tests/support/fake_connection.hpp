// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "nomad/mavlink/connection.hpp"

#include <chrono>
#include <atomic>
#include <cstdint>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

class FakeConnection : public nomad::mavlink::MavlinkConnection {
  public:
    FakeConnection() {
        state->identity = nomad::telemetry::identify_vehicle(nomad::telemetry::kArduPilotAutopilot,
                                                              nomad::telemetry::kQuadrotor);
    }

    struct GotoRequest {
        double latitude_deg{};
        double longitude_deg{};
        float relative_altitude_m{};
    };

    bool connect() override {
        connect_count += 1;
        connected = true;
        if (state->session_id == 0) {
            state->session_id = 1;
        }
        return true;
    }

    void disconnect() override {
        event_log.push_back("disconnect");
        connected = false;
    }

    bool is_connected() const override {
        return connected;
    }

    nomad::mavlink::ConnectFailure get_connect_failure() const override {
        return nomad::mavlink::ConnectFailure::None;
    }

    std::optional<nomad::mavlink::Heartbeat> wait_for_heartbeat(std::chrono::milliseconds) override {
        return nomad::mavlink::Heartbeat{1, 1, 0, 2, 3, 0};
    }

    std::optional<nomad::telemetry::VehicleState> wait_for_state(std::chrono::milliseconds) override {
        complete_transition_after_ack_on_state_poll();
        if (auto_stamp_fresh_fields) {
            stamp_fresh_fields();
        }
        return get_state();
    }

    nomad::telemetry::VehicleState get_state() const override {
        std::lock_guard lock(state_mutex);
        return *state;
    }

    std::optional<nomad::mavlink::CommandAck> send_command(const nomad::mavlink::Command &command,
                                                           std::chrono::milliseconds) override {
        if (!admit_send()) {
            return std::nullopt;
        }
        command_started = true;
        if (command_delay > std::chrono::milliseconds::zero()) {
            std::this_thread::sleep_for(command_delay);
        }
        if (!admit_send()) {
            return std::nullopt;
        }
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

    bool goto_location_relative(double latitude_deg, double longitude_deg, float relative_altitude_m,
                                std::chrono::milliseconds) override {
        std::lock_guard lock(state_mutex);
        last_goto = GotoRequest{latitude_deg, longitude_deg, relative_altitude_m};
        if (!acknowledgement || acknowledgement->command != 192 || acknowledgement->result != 0) {
            return false;
        }
        state->position_valid = true;
        state->position.latitude_deg = latitude_deg;
        state->position.longitude_deg = longitude_deg;
        state->position.relative_altitude_m = relative_altitude_m;
        return true;
    }

    std::optional<nomad::mavlink::CommandAck> send_fixed_wing_waypoint(
        const nomad::mavlink::FixedWingWaypointCommand &, std::uint64_t,
        std::chrono::milliseconds) override {
        return std::nullopt;
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
        fence_plan_upload_count += 1;
        uploaded_fence_plan = items;
        return fence_plan_upload_result;
    }

    std::optional<std::vector<nomad::mavlink::FencePlanItem>> download_fence_plan(std::chrono::milliseconds) override {
        fence_plan_download_count += 1;
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
        parameter_read_count += 1;
        if (change_session_after_param_read) {
            std::lock_guard lock(state_mutex);
            ++state->session_id;
        }
        const auto found = parameters.find(param_id);
        if (found == parameters.end()) {
            return std::nullopt;
        }
        return found->second;
    }

    std::optional<nomad::mavlink::AutopilotVersion>
    read_autopilot_version(std::chrono::milliseconds) override {
        ++version_read_count;
        return autopilot_version;
    }

    std::atomic_bool connected{false};
    std::atomic_int connect_count{0};
    std::atomic_bool command_started{false};
    std::chrono::milliseconds command_delay{0};
    // Most tests model a live feed, so polling refreshes valid timestamps by default.
    // Disabling this suppresses only that blanket refresh; simulated effects still
    // stamp any new samples they create.
    bool auto_stamp_fresh_fields{true};
    int version_read_count{0};
    std::optional<nomad::mavlink::AutopilotVersion> autopilot_version{
        nomad::mavlink::AutopilotVersion{4, 7, 1, "dbe79216"},
    };
    std::optional<nomad::telemetry::VehicleState> state{
        nomad::telemetry::VehicleState{true, true, false, 1, 1, 4},
    };
    nomad::mavlink::VelocitySetpoint last_velocity{};
    int velocity_send_count{0};
    nomad::mavlink::Command last_command{};
    std::optional<GotoRequest> last_goto;
    bool invalidate_vtol_after_guided_mode{false};
    bool stale_position_after_guided_mode{false};
    std::optional<nomad::mavlink::CommandAck> acknowledgement{
        nomad::mavlink::CommandAck{0, 0},
    };
    std::vector<nomad::mavlink::Command> command_history;
    std::vector<bool> velocity_send_results;
    std::vector<bool> command_send_results;
    bool disarm_on_takeoff{false};
    std::optional<float> takeoff_altitude_override;
    bool stale_position_after_arm{false};
    bool stale_heartbeat_after_arm{false};
    bool invalidate_gps_after_arm{false};
    bool complete_transition_on_command{true};
    bool complete_transition_after_ack_on_poll{false};
    bool transition_stays_intermediate{false};
    bool transition_state_unavailable_on_command{false};
    bool transition_loses_link_on_command{false};
    std::vector<nomad::mavlink::FencePoint> fence_points;
    std::vector<std::uint8_t> fence_indices;
    std::uint8_t fence_total{0};
    std::vector<std::uint8_t> requested_fence_indices;
    std::size_t readback_index{0};
    std::vector<bool> fence_send_results;
    std::vector<nomad::mavlink::FencePlanItem> uploaded_fence_plan;
    bool fence_plan_upload_result{true};
    bool fence_plan_download_result{true};
    int fence_plan_upload_count{0};
    int fence_plan_download_count{0};
    int parameter_read_count{0};
    bool change_session_after_param_read{false};
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

    void set_identity(nomad::telemetry::VehicleIdentity identity) {
        std::lock_guard lock(state_mutex);
        state->identity = identity;
    }

    std::size_t command_count() const {
        std::lock_guard lock(state_mutex);
        return command_history.size();
    }

  protected:
    mutable std::mutex state_mutex;

  private:
    bool transition_after_ack_pending{false};

  protected:
    void complete_transition_after_ack_on_state_poll() {
        std::lock_guard lock(state_mutex);
        if (!transition_after_ack_pending) {
            return;
        }
        state->vtol_state = nomad::telemetry::VtolState::FixedWing;
        state->vtol_state_valid = true;
        state->vtol_state_updated_at = std::chrono::steady_clock::now();
        transition_after_ack_pending = false;
    }

    void stamp_fresh_fields() {
        std::lock_guard lock(state_mutex);
        const auto now = std::chrono::steady_clock::now();
        if (state->position_valid) {
            state->position_updated_at = now;
        }
        if (state->velocity_updated_at != std::chrono::steady_clock::time_point{}) {
            state->velocity_updated_at = now;
        }
        if (state->battery_valid) {
            state->battery_updated_at = now;
        }
        if (state->gps_valid) {
            state->gps_updated_at = now;
        }
        if (state->attitude_valid) {
            state->attitude_updated_at = now;
        }
        if (state->vtol_state_valid) {
            state->vtol_state_updated_at = now;
        }
        if (state->landed_state_valid) {
            state->landed_state_updated_at = now;
        }
    }

  private:
    bool take_fence_send_result() {
        const bool result = fence_send_results.front();
        fence_send_results.erase(fence_send_results.begin());
        return result;
    }

    void update_state_for_command(const nomad::mavlink::Command &command) {
        if (command.id == 400) {
            state->armed = command.parameters[0] > 0.0F;
            if (state->armed && stale_position_after_arm) {
                auto_stamp_fresh_fields = false;
                state->position_updated_at = std::chrono::steady_clock::now() - std::chrono::seconds(3);
            }
            if (state->armed && stale_heartbeat_after_arm) {
                state->heartbeat_fresh = false;
                state->connected = false;
            }
            if (state->armed && invalidate_gps_after_arm) {
                state->gps_valid = false;
                state->gps.fix_type = 0;
            }
        } else if (command.id == 176) {
            state->custom_mode = static_cast<std::uint32_t>(command.parameters[1]);
            if (state->custom_mode == 15 && invalidate_vtol_after_guided_mode) {
                state->vtol_state_valid = false;
            }
            if (state->custom_mode == 15 && stale_position_after_guided_mode) {
                auto_stamp_fresh_fields = false;
                state->position_updated_at = std::chrono::steady_clock::now() - std::chrono::seconds(3);
            }
        } else if (command.id == 22) {
            state->position_valid = true;
            if (takeoff_altitude_override.has_value()) {
                state->position.relative_altitude_m = *takeoff_altitude_override;
            } else if (state->identity.aircraft_class == nomad::telemetry::AircraftClass::QuadPlane) {
                state->position.relative_altitude_m += command.parameters[6];
            } else {
                state->position.relative_altitude_m = command.parameters[6];
            }
            if (disarm_on_takeoff) {
                state->armed = false;
            }
        } else if (command.id == 3000 && command.parameters[0] == 4.0F) {
            if (transition_state_unavailable_on_command) {
                state->vtol_state_valid = false;
            } else if (transition_loses_link_on_command) {
                state->connected = false;
                state->heartbeat_fresh = false;
            } else if (transition_stays_intermediate) {
                state->vtol_state = nomad::telemetry::VtolState::TransitionToFixedWing;
                state->vtol_state_valid = true;
                state->vtol_state_updated_at = std::chrono::steady_clock::now();
            } else if (complete_transition_after_ack_on_poll) {
                transition_after_ack_pending = true;
            } else if (complete_transition_on_command) {
                state->vtol_state = nomad::telemetry::VtolState::FixedWing;
                state->vtol_state_valid = true;
                state->vtol_state_updated_at = std::chrono::steady_clock::now();
            }
        } else if (command.id == 21) {
            if (state->identity.aircraft_class == nomad::telemetry::AircraftClass::Copter) {
                state->custom_mode = 9;
                state->armed = false;
            }
        } else if (command.id == 20) {
            switch (state->identity.aircraft_class) {
            case nomad::telemetry::AircraftClass::Copter:
                state->custom_mode = 6;
                break;
            case nomad::telemetry::AircraftClass::Plane:
                state->custom_mode = 11;
                break;
            case nomad::telemetry::AircraftClass::QuadPlane:
                state->custom_mode = 21;
                break;
            case nomad::telemetry::AircraftClass::Unknown:
                break;
            }
        }
    }
};
