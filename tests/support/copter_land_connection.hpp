// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "fake_connection.hpp"

#include <algorithm>
#include <chrono>
#include <mutex>
#include <optional>
#include <thread>

class CopterLandConnection : public FakeConnection {
  public:
    enum class Observation {
        FreshLand,
        PreAckLand,
        OtherMode,
        ChangedSession,
        ChangedIdentity,
        ChangedSystem,
        ChangedComponent,
        LostLink,
        FutureHeartbeat,
    };

    Observation observation{Observation::FreshLand};
    std::chrono::milliseconds ack_delay{};
    std::chrono::milliseconds heartbeat_delay{25};
    std::chrono::milliseconds supplied_timeout{};
    std::atomic_bool land_sent{false};

    std::optional<nomad::mavlink::CommandAck> send_command(const nomad::mavlink::Command &command,
        std::uint64_t expected_session, std::chrono::milliseconds timeout) override {
        supplied_timeout = timeout;
        if (get_state().session_id != expected_session) {
            return std::nullopt;
        }
        const auto result = FakeConnection::send_command(command, timeout);
        land_sent = true;
        std::this_thread::sleep_for((std::min)(ack_delay, timeout));
        {
            std::lock_guard lock(state_mutex);
            ack_returned_at_ = std::chrono::steady_clock::now();
        }
        return ack_delay < timeout ? result : std::nullopt;
    }

    nomad::telemetry::VehicleState get_state() const override {
        auto observed = FakeConnection::get_state();
        std::lock_guard lock(state_mutex);
        if (ack_returned_at_ == std::chrono::steady_clock::time_point{} ||
            std::chrono::steady_clock::now() < ack_returned_at_ + heartbeat_delay) {
            return observed;
        }
        if (observation != Observation::PreAckLand) {
            observed.heartbeat_updated_at = std::chrono::steady_clock::now();
        }
        change_observation(observed);
        return observed;
    }

  private:
    std::chrono::steady_clock::time_point ack_returned_at_{};

    void change_observation(nomad::telemetry::VehicleState &observed) const {
        switch (observation) {
        case Observation::FreshLand:
        case Observation::PreAckLand:
            break;
        case Observation::OtherMode:
            observed.custom_mode = 4;
            break;
        case Observation::ChangedSession:
            ++observed.session_id;
            break;
        case Observation::ChangedIdentity:
            observed.identity.vehicle_type = nomad::telemetry::kHexarotor;
            break;
        case Observation::ChangedSystem:
            ++observed.system_id;
            break;
        case Observation::ChangedComponent:
            ++observed.component_id;
            break;
        case Observation::LostLink:
            observed.connected = false;
            observed.heartbeat_fresh = false;
            break;
        case Observation::FutureHeartbeat:
            observed.heartbeat_updated_at += std::chrono::seconds(1);
            break;
        }
    }
};
