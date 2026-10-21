// SPDX-License-Identifier: Apache-2.0
#include "runtime_implementation.hpp"

#include "nomad/vehicle/operation.hpp"

#include <cmath>

namespace nomad::runtime {

using namespace detail;

namespace {

Json finite_number(double value) {
    return std::isfinite(value) ? Json(value) : Json(nullptr);
}

Json observed_position(const telemetry::VehicleState &state) {
    if (state.position_updated_at == Clock::time_point{}) {
        return nullptr;
    }
    const auto &position = state.position;
    return {{"latitude_deg", finite_number(position.latitude_deg)},
            {"longitude_deg", finite_number(position.longitude_deg)},
            {"absolute_altitude_m", finite_number(position.altitude_m)},
            {"relative_altitude_m", finite_number(position.relative_altitude_m)}};
}

Json supported_flight_operations(telemetry::AircraftClass aircraft_class) {
    if (aircraft_class == telemetry::AircraftClass::Unknown) {
        return nullptr;
    }
    if (vehicle::supports_operation(aircraft_class, vehicle::VehicleOperation::Land)) {
        return Json::array({"land"});
    }
    return Json::array();
}

} // namespace

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
            {"supported_flight_operations", supported_flight_operations(identity)},
            {"last_flight_request", last_flight_request_snapshot()},
            {"armed", state.armed},
            {"custom_mode", state.custom_mode},
            {"telemetry",
             {{"heartbeat_fresh", state.heartbeat_fresh},
              {"heartbeat_age_ms", optional_age(age_milliseconds(state.heartbeat_updated_at))},
              {"vtol_state", telemetry::vtol_state_name(state.vtol_state)},
              {"vtol_state_valid", state.vtol_state_valid},
              {"vtol_state_age_ms", optional_age(age_milliseconds(state.vtol_state_updated_at))},
              {"landed_state", telemetry::landed_state_name(state.landed_state)},
              {"landed_state_valid", state.landed_state_valid},
              {"landed_state_age_ms", optional_age(age_milliseconds(state.landed_state_updated_at))},
              {"position", observed_position(state)},
              {"position_valid", state.position_valid},
              {"position_age_ms", optional_age(age_milliseconds(state.position_updated_at))},
              {"gps_valid", state.gps_valid},
              {"gps_age_ms", optional_age(age_milliseconds(state.gps_updated_at))},
              {"attitude_valid", state.attitude_valid},
              {"attitude_age_ms", optional_age(age_milliseconds(state.attitude_updated_at))},
              {"battery_valid", state.battery_valid},
              {"battery_age_ms", optional_age(age_milliseconds(state.battery_updated_at))}}}};
}

Json Runtime::Implementation::last_flight_request_snapshot() const {
    std::lock_guard lock(cache_mutex_);
    if (!last_flight_request_) {
        return nullptr;
    }
    const auto &request = *last_flight_request_;
    const bool pending = in_flight_.contains(request.key);
    const auto completed = response_cache_.find(request.key);
    if (!pending && completed == response_cache_.end()) {
        return nullptr;
    }
    const char *phase = "completed";
    if (pending) {
        phase = request.observing ? "observing" : "sending";
    }
    return {{"request_id", request.id},
            {"operation", request.operation},
            {"vehicle_session", request.session},
            {"authority_generation", request.generation},
            {"age_ms", optional_age(age_milliseconds(request.started_at))},
            {"phase", phase},
            {"acknowledged", request.acknowledged->load()},
            {"outcome", pending ? Json(nullptr) : completed->second.response.at("outcome")}};
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
