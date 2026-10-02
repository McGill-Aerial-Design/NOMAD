// SPDX-License-Identifier: Apache-2.0
// Private status projection of the runtime owner.

    Json status_snapshot() const {
        const auto state = connection_->get_state();
        const bool connection_open = connection_->is_connected();
        const auto identity = state.identity.aircraft_class;
        std::string owner;
        std::uint64_t generation;
        bool stopping;
        {
            std::shared_lock lock(authority_gate_->mutex);
            owner = authority_gate_->owner;
            generation = authority_gate_->generation;
            stopping = authority_gate_->stopping;
        }
        const bool healthy = journal_->healthy();
        const std::string lifecycle = stopping ? "stopping" :
            healthy && connection_open && state.connected && state.heartbeat_fresh ? "ready" : "degraded";
        return {{"runtime_ready", ready()},
                {"lifecycle", lifecycle},
                {"runtime_incarnation", incarnation_},
                {"audit_healthy", journal_->healthy()},
                {"actuation_enabled", config_.actuation_enabled},
                {"vehicle_session", state.session_id},
                {"authority_generation", generation},
                {"authority_owner", owner.empty() ? Json(nullptr) : Json(owner)},
                {"server_time_ms", unix_milliseconds()},
                {"mavsdk_connection_open", connection_open},
                {"vehicle_transport_connected", connection_open},
                {"vehicle_session_established", state.session_id != 0},
                {"vehicle_connected", state.connected},
                {"identity_resolved", identity != telemetry::AircraftClass::Unknown},
                {"aircraft_class", telemetry::aircraft_class_name(identity)},
                {"armed", state.armed},
                {"custom_mode", state.custom_mode},
                {"telemetry",
                 {{"heartbeat_fresh", state.heartbeat_fresh},
                  {"position_valid", state.position_valid},
                  {"position_age_ms", optional_age(age_milliseconds(state.position_updated_at))},
                  {"gps_valid", state.gps_valid},
                  {"gps_age_ms", optional_age(age_milliseconds(state.gps_updated_at))},
                  {"attitude_valid", state.attitude_valid},
                  {"attitude_age_ms", optional_age(age_milliseconds(state.attitude_updated_at))},
                  {"battery_valid", state.battery_valid},
                  {"battery_age_ms", optional_age(age_milliseconds(state.battery_updated_at))}}}};
    }

    std::uint64_t current_generation() const {
        std::shared_lock lock(authority_gate_->mutex);
        return authority_gate_->generation;
    }

    std::uint64_t next_sequence() const {
        std::shared_lock lock(authority_gate_->mutex);
        return authority_gate_->last_sequence + 1;
    }
