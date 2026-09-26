// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "fake_connection.hpp"

#include <chrono>
#include <cstdint>
#include <optional>
#include <vector>

class FixedWingWaypointFakeConnection : public FakeConnection {
  public:
    std::optional<nomad::telemetry::VehicleState> wait_for_state(std::chrono::milliseconds) override {
        complete_transition_after_ack_on_state_poll();
        complete_fixed_wing_waypoint_after_ack_on_state_poll();
        apply_fixed_wing_waypoint_sample();
        if (auto_stamp_fresh_fields) {
            stamp_fresh_fields();
        }
        return get_state();
    }

    std::optional<nomad::mavlink::CommandAck> send_fixed_wing_waypoint(
        const nomad::mavlink::FixedWingWaypointCommand &waypoint, std::uint64_t expected_session_id,
        std::chrono::milliseconds) override {
        std::lock_guard lock(state_mutex);
        change_session_before_send();
        if (!matches_expected_session(expected_session_id)) {
            return std::nullopt;
        }
        record_waypoint(waypoint);
        if (!fixed_wing_waypoint_transport_enabled || !fixed_wing_waypoint_ack.has_value()) {
            return std::nullopt;
        }
        if (fixed_wing_waypoint_ack->result != 0) {
            return fixed_wing_waypoint_ack;
        }

        const bool complete_waypoint = take_completion_result();
        apply_send_interruption_effects();
        make_selected_samples_stale();
        apply_position_before_ack();
        stage_waypoint_completion(waypoint, complete_waypoint);
        return fixed_wing_waypoint_ack;
    }

    std::vector<nomad::mavlink::FixedWingWaypointCommand> fixed_wing_waypoint_requests;
    std::optional<nomad::mavlink::CommandAck> fixed_wing_waypoint_ack{
        nomad::mavlink::CommandAck{192, 0},
    };
    std::vector<bool> fixed_wing_waypoint_completions;
    int fixed_wing_waypoint_send_count{0};
    bool fixed_wing_waypoint_transport_enabled{true};
    bool fixed_wing_waypoint_auto_complete{true};
    bool fixed_wing_waypoint_completion_before_ack{false};
    bool fixed_wing_waypoint_session_change_before_send{false};
    bool fixed_wing_waypoint_session_change_on_send{false};
    bool fixed_wing_waypoint_link_loss_on_send{false};
    bool fixed_wing_waypoint_heartbeat_loss_on_send{false};
    bool fixed_wing_waypoint_mode_loss_on_send{false};
    bool fixed_wing_waypoint_vtol_loss_on_send{false};
    bool fixed_wing_waypoint_vtol_mc_on_send{false};
    bool fixed_wing_waypoint_stale_position_on_send{false};
    bool fixed_wing_waypoint_stale_gps_on_send{false};
    bool fixed_wing_waypoint_stale_vtol_on_send{false};
    bool fixed_wing_waypoint_disarm_on_send{false};
    std::vector<nomad::telemetry::Position> fixed_wing_waypoint_samples;
    std::optional<nomad::telemetry::Position> fixed_wing_waypoint_position_before_ack;

  private:
    std::optional<nomad::mavlink::FixedWingWaypointCommand> fixed_wing_waypoint_after_ack_pending;
    std::size_t fixed_wing_waypoint_sample_index{0};

    void change_session_before_send() {
        if (fixed_wing_waypoint_session_change_before_send) {
            ++state->session_id;
        }
    }

    bool matches_expected_session(std::uint64_t expected_session_id) const {
        return expected_session_id != 0 && state->session_id == expected_session_id && state->connected &&
               state->heartbeat_fresh;
    }

    void record_waypoint(const nomad::mavlink::FixedWingWaypointCommand &waypoint) {
        fixed_wing_waypoint_requests.push_back(waypoint);
        ++fixed_wing_waypoint_send_count;
    }

    bool take_completion_result() {
        if (fixed_wing_waypoint_completions.empty()) {
            return fixed_wing_waypoint_auto_complete;
        }
        const bool complete = fixed_wing_waypoint_completions.front();
        fixed_wing_waypoint_completions.erase(fixed_wing_waypoint_completions.begin());
        return complete;
    }

    void apply_send_interruption_effects() {
        if (fixed_wing_waypoint_session_change_on_send) {
            ++state->session_id;
        }
        if (fixed_wing_waypoint_link_loss_on_send) {
            state->connected = false;
            state->heartbeat_fresh = false;
        }
        if (fixed_wing_waypoint_heartbeat_loss_on_send) {
            state->heartbeat_fresh = false;
        }
        if (fixed_wing_waypoint_mode_loss_on_send) {
            state->custom_mode = 10;
        }
        if (fixed_wing_waypoint_vtol_loss_on_send) {
            state->vtol_state_valid = false;
        }
        if (fixed_wing_waypoint_vtol_mc_on_send) {
            state->vtol_state = nomad::telemetry::VtolState::Multicopter;
        }
        if (fixed_wing_waypoint_disarm_on_send) {
            state->armed = false;
        }
    }

    void make_selected_samples_stale() {
        if (fixed_wing_waypoint_stale_gps_on_send) {
            auto_stamp_fresh_fields = false;
            state->gps_updated_at = std::chrono::steady_clock::now() - std::chrono::seconds(3);
        }
        if (fixed_wing_waypoint_stale_vtol_on_send) {
            auto_stamp_fresh_fields = false;
            state->vtol_state_updated_at = std::chrono::steady_clock::now() - std::chrono::seconds(4);
        }
        if (fixed_wing_waypoint_stale_position_on_send) {
            auto_stamp_fresh_fields = false;
            state->position_updated_at = std::chrono::steady_clock::now() - std::chrono::seconds(3);
        }
    }

    void apply_position_before_ack() {
        if (!fixed_wing_waypoint_position_before_ack.has_value()) {
            return;
        }
        state->position = *fixed_wing_waypoint_position_before_ack;
        state->position_updated_at = std::chrono::steady_clock::now();
    }

    void stage_waypoint_completion(const nomad::mavlink::FixedWingWaypointCommand &waypoint, bool complete) {
        if (!complete) {
            return;
        }
        if (fixed_wing_waypoint_completion_before_ack) {
            apply_waypoint_position(waypoint);
            return;
        }
        fixed_wing_waypoint_after_ack_pending = waypoint;
    }

    void complete_fixed_wing_waypoint_after_ack_on_state_poll() {
        std::lock_guard lock(state_mutex);
        if (!fixed_wing_waypoint_after_ack_pending.has_value()) {
            return;
        }
        apply_waypoint_position(*fixed_wing_waypoint_after_ack_pending);
        fixed_wing_waypoint_after_ack_pending.reset();
    }

    void apply_waypoint_position(const nomad::mavlink::FixedWingWaypointCommand &waypoint) {
        state->position.latitude_deg = waypoint.latitude_deg;
        state->position.longitude_deg = waypoint.longitude_deg;
        state->position.relative_altitude_m = waypoint.relative_altitude_m;
        state->position_valid = true;
        state->position_updated_at = std::chrono::steady_clock::now();
    }

    void apply_fixed_wing_waypoint_sample() {
        std::lock_guard lock(state_mutex);
        if (fixed_wing_waypoint_sample_index >= fixed_wing_waypoint_samples.size()) {
            return;
        }
        state->position = fixed_wing_waypoint_samples[fixed_wing_waypoint_sample_index++];
        state->position_valid = true;
        state->position_updated_at = std::chrono::steady_clock::now();
    }
};
