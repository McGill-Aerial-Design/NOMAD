// SPDX-License-Identifier: Apache-2.0
// Coherent publication and callback-safe retirement of one selected MAVSDK system.

#include "mavsdk_mavlink_connection.hpp"

#include <utility>

namespace nomad::mavlink {

MavsdkMavlinkConnection::ConnectionResources::ConnectionResources(std::shared_ptr<mavsdk::System> selected)
    : system(std::move(selected)), action(std::make_unique<mavsdk::Action>(system)),
      telemetry(std::make_unique<mavsdk::Telemetry>(system)),
      passthrough(std::make_unique<mavsdk::MavlinkPassthrough>(system)),
      geofence(std::make_unique<mavsdk::Geofence>(system)), param(std::make_unique<mavsdk::Param>(system)),
      offboard(std::make_unique<mavsdk::Offboard>(system)) {}

MavsdkMavlinkConnection::ConnectionResources::~ConnectionResources() {
    if (position_handle) {
        telemetry->unsubscribe_position(*position_handle);
    }
    if (velocity_handle) {
        telemetry->unsubscribe_velocity_ned(*velocity_handle);
    }
    if (battery_handle) {
        telemetry->unsubscribe_battery(*battery_handle);
    }
    if (gps_handle) {
        telemetry->unsubscribe_gps_info(*gps_handle);
    }
    if (attitude_handle) {
        telemetry->unsubscribe_attitude_euler(*attitude_handle);
    }
    if (vtol_state_handle) {
        telemetry->unsubscribe_vtol_state(*vtol_state_handle);
    }
    if (landed_state_handle) {
        telemetry->unsubscribe_landed_state(*landed_state_handle);
    }
    if (heartbeat_handle) {
        passthrough->unsubscribe_message(MAVLINK_MSG_ID_HEARTBEAT, *heartbeat_handle);
    }
    if (connection_handle) {
        system->unsubscribe_is_connected(*connection_handle);
    }
}

void MavsdkMavlinkConnection::subscribe(ConnectionResources &candidate) {
    const auto gate = candidate.callbacks;
    candidate.position_handle = candidate.telemetry->subscribe_position([gate](const auto &value) {
        std::lock_guard lock(gate->mutex);
        if (gate->owner) {
            gate->owner->observe_position(value);
        }
    });
    candidate.velocity_handle = candidate.telemetry->subscribe_velocity_ned([gate](const auto &value) {
        std::lock_guard lock(gate->mutex);
        if (gate->owner) {
            gate->owner->observe_velocity(value);
        }
    });
    candidate.battery_handle = candidate.telemetry->subscribe_battery([gate](const auto &value) {
        std::lock_guard lock(gate->mutex);
        if (gate->owner) {
            gate->owner->observe_battery(value);
        }
    });
    candidate.gps_handle = candidate.telemetry->subscribe_gps_info([gate](const auto &value) {
        std::lock_guard lock(gate->mutex);
        if (gate->owner) {
            gate->owner->observe_gps(value);
        }
    });
    candidate.attitude_handle = candidate.telemetry->subscribe_attitude_euler([gate](const auto &value) {
        std::lock_guard lock(gate->mutex);
        if (gate->owner) {
            gate->owner->observe_attitude(value);
        }
    });
    candidate.vtol_state_handle = candidate.telemetry->subscribe_vtol_state([gate](const auto &value) {
        std::lock_guard lock(gate->mutex);
        if (gate->owner) {
            gate->owner->observe_vtol_state(value);
        }
    });
    candidate.landed_state_handle = candidate.telemetry->subscribe_landed_state([gate](const auto &value) {
        std::lock_guard lock(gate->mutex);
        if (gate->owner) {
            gate->owner->observe_landed_state(value);
        }
    });
    candidate.heartbeat_handle =
        candidate.passthrough->subscribe_message(MAVLINK_MSG_ID_HEARTBEAT, get_heartbeat_callback(gate));
    candidate.connection_handle = candidate.system->subscribe_is_connected([gate](bool connected) {
        std::lock_guard lock(gate->mutex);
        if (gate->owner && !connected) {
            gate->owner->observe_connection_loss();
        }
    });
}

mavsdk::MavlinkPassthrough::MessageCallback
MavsdkMavlinkConnection::get_heartbeat_callback(const std::shared_ptr<CallbackGate> &gate) {
    return [gate](const auto &message) {
        std::lock_guard lock(gate->mutex);
        if (gate->owner) {
            gate->owner->observe_heartbeat(message);
        }
    };
}

void MavsdkMavlinkConnection::publish(std::unique_ptr<ConnectionResources> candidate) {
    std::unique_lock lifetime_lock(plugin_lifetime_mutex_);
    std::lock_guard callback_lock(candidate->callbacks->mutex);
    ObservationUpdate update(observation_mutex_, observation_changed_);
    ++session_id_counter_;
    if (session_id_counter_ == 0) {
        ++session_id_counter_;
    }
    notify_vehicle_session_changed(session_id_counter_);
    state_.session_id = session_id_counter_;
    candidate->callbacks->owner = this;
    resources_ = std::move(candidate);
}

void MavsdkMavlinkConnection::observe_connection_loss() {
    ObservationUpdate update(observation_mutex_, observation_changed_);
    ++session_id_counter_;
    if (session_id_counter_ == 0) {
        ++session_id_counter_;
    }
    notify_vehicle_session_changed(session_id_counter_);
    state_.session_id = session_id_counter_;
    state_.connected = false;
    state_.heartbeat_fresh = false;
}

void MavsdkMavlinkConnection::close() {
    std::unique_ptr<ConnectionResources> retired;
    {
        std::unique_lock lifetime_lock(plugin_lifetime_mutex_);
        if (resources_) {
            // Unsubscribe does not drain MAVSDK's copied user callbacks. Revoking
            // the gate waits for entered callbacks; queued copies never touch this.
            std::lock_guard callback_lock(resources_->callbacks->mutex);
            resources_->callbacks->owner = nullptr;
        }
        retired = std::move(resources_);
        ObservationUpdate update(observation_mutex_, observation_changed_);
        ++session_id_counter_;
        if (session_id_counter_ == 0) {
            ++session_id_counter_;
        }
        notify_vehicle_session_changed(session_id_counter_);
        state_ = {};
        quadplane_enabled_.reset();
        heartbeat_.reset();
        last_heartbeat_ = {};
        velocity_active_ = false;
    }
    // Shared command users have finished before detachment. SDK teardown needs
    // none of the publication/callback/observation locks to make progress.
    retired.reset();
    if (handle_) {
        sdk_.remove_connection(*handle_);
        handle_.reset();
    }
}

} // namespace nomad::mavlink
