// SPDX-License-Identifier: Apache-2.0
#include "runtime_implementation.hpp"

namespace nomad::runtime {

using namespace detail;

Json Runtime::Implementation::handle_read_request(const Request &request) {
    if (request.type == "hello" || request.type == "status") {
        observe_vehicle_session();
    }
    if (request.type == "hello") {
        return {{"protocol", kProtocolName},
                {"version", kProtocolVersion},
                {"id", request.id},
                {"ok", true},
                {"type", "hello_response"},
                {"runtime_version", config_.version},
                {"runtime_incarnation", incarnation_},
                {"client_authentication", "hmac-sha256-v1"},
                {"server_proof", hello_proof(request)},
                {"authority", {{"vehicle_session", connection_->get_state().session_id},
                                {"generation", current_generation()},
                                {"next_sequence", next_sequence()},
                                {"server_time_ms", unix_milliseconds()}}},
                {"capabilities",
                 {"hello", "ping", "status", "get_actuators", "configure_actuators", "actuator_action",
                  "land", "set_servo", "set_relay", "motor_test",
                  "configure_gimbal", "set_gimbal_target", "configure_gimbal_target",
                  "admit_authority", "revoke_authority",
                  "handback_authority"}}};
    }
    if (request.type == "get_actuators") {
        observe_vehicle_session();
        std::uint64_t revision{};
        auto catalog = actuators_.discover(actuator_authority(), Clock::now(), &revision);
        return {{"protocol", kProtocolName}, {"version", kProtocolVersion}, {"id", request.id},
                {"ok", true}, {"type", "actuators_response"}, {"runtime_incarnation", incarnation_},
                {"configuration_recovery_required", actuator_configuration_recovery_.load()},
                {"actuator_configuration_revision", revision}, {"actuators", std::move(catalog)}};
    }
    if (request.type == "ping") {
        return {{"protocol", kProtocolName}, {"version", kProtocolVersion}, {"id", request.id},
                {"ok", true}, {"type", "pong"}};
    }
    if (request.type == "status") {
        return {{"protocol", kProtocolName}, {"version", kProtocolVersion}, {"id", request.id},
                {"ok", true}, {"type", "status_response"}, {"status", status_snapshot()}};
    }
    return error_response(request.id, "unsupported_request", "request type is not supported in protocol v1");
}

Json Runtime::Implementation::status_snapshot() const {
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

std::uint64_t Runtime::Implementation::current_generation() const {
    std::shared_lock lock(authority_gate_->mutex);
    return authority_gate_->generation;
}

std::uint64_t Runtime::Implementation::next_sequence() const {
    std::shared_lock lock(authority_gate_->mutex);
    return authority_gate_->last_sequence + 1;
}

} // namespace nomad::runtime
