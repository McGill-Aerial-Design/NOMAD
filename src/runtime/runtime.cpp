// SPDX-License-Identifier: Apache-2.0
#include "nomad/runtime/runtime.hpp"

#include "ipc_server.hpp"
#include "nomad/safety/velocity_config.hpp"
#include "nomad/telemetry/state.hpp"
#include "nomad/vehicle/vehicle.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <iostream>
#include <limits>
#include <mutex>
#include <optional>
#include <random>
#include <shared_mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include "runtime_detail.hpp"

namespace nomad::runtime {

struct Runtime::Implementation {
    struct CacheEntry {
        std::string fingerprint;
        Json response;
    };

    Implementation(std::unique_ptr<mavlink::MavlinkConnection> connection, RuntimeConfig config)
        : incarnation_(new_incarnation()), authority_gate_(std::make_shared<AuthorityGate>()),
          connection_(std::move(connection)), config_(std::move(config)),
          vehicle_(require_connection(connection_), make_vehicle_config(config_)) {
        authority_gate_->incarnation = incarnation_;
        const std::weak_ptr<AuthorityGate> weak_gate = authority_gate_;
        connection_->set_transmission_admission_factory([weak_gate] {
            if (active_request == nullptr) {
                return mavlink::TransmissionAdmission([](const std::function<void()> &) { return false; });
            }
            const auto &request = *active_request;
            const mavlink::SendAuthorityToken token{request.incarnation, request.session, request.generation,
                                                    request.source, request.id, request.sequence,
                                                    request.expires_at_ms};
            return mavlink::TransmissionAdmission([weak_gate, token](const std::function<void()> &send) {
                const auto gate = weak_gate.lock();
                if (!gate || !send) {
                    return false;
                }
                std::shared_lock lock(gate->mutex);
                if (!matches_authority(*gate, token)) {
                    return false;
                }
                send();
                return true;
            });
        });
        connection_->set_vehicle_session_changed_handler([weak_gate](std::uint64_t session) {
            const auto gate = weak_gate.lock();
            if (!gate) {
                return;
            }
            std::unique_lock lock(gate->mutex);
            update_gate_session(*gate, session);
        });
    }

    bool start(std::string &error) {
        if (!server_.start(config_.ipc_port, [this](std::string_view request) { return handle_message(request); },
                           error)) {
            return false;
        }
        stopping_ = false;
        try {
            connection_worker_ = std::thread(&Implementation::maintain_connection, this);
        } catch (...) {
            server_.stop();
            error = "could not start MAVSDK connection worker";
            return false;
        }
        return true;
    }

    void stop() {
        stopping_ = true;
        {
            std::unique_lock lock(authority_gate_->mutex);
            if (!authority_gate_->stopping) {
                authority_gate_->stopping = true;
                ++authority_gate_->generation;
                authority_gate_->owner.clear();
                authority_gate_->owner_session = 0;
                authority_gate_->last_sequence = 0;
            }
        }
        server_.stop();
        if (connection_worker_.joinable()) {
            connection_worker_.join();
        }
        connection_->disconnect();
    }

    bool ready() const {
        return server_.running();
    }

    void maintain_connection() {
        while (!stopping_) {
            if (!connection_->is_connected()) {
                if (!connection_->connect()) {
                    observe_vehicle_session();
                    std::this_thread::sleep_for(config_.reconnect_delay);
                    continue;
                }
            }
            observe_vehicle_session();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    void observe_vehicle_session() {
        const auto state = connection_->get_state();
        const bool connection_open = connection_->is_connected();
        std::unique_lock lock(authority_gate_->mutex);
        revoke_changed_session_locked(state, connection_open);
    }

    void revoke_changed_session_locked(const telemetry::VehicleState &state, bool connection_open) {
        if (state.session_id != authority_gate_->vehicle_session) {
            if (is_newer_session(state.session_id, authority_gate_->vehicle_session)) {
                update_gate_session(*authority_gate_, state.session_id);
            }
            return;
        }
        if (authority_gate_->owner.empty()) {
            return;
        }
        if (connection_open && state.connected && state.session_id == authority_gate_->owner_session) {
            return;
        }
        ++authority_gate_->generation;
        authority_gate_->owner.clear();
        authority_gate_->owner_session = 0;
        authority_gate_->last_sequence = 0;
        authority_gate_->vehicle_session = state.session_id;
    }

    std::string handle_message(std::string_view line) {
        auto parsed = parse_request(line);
        if (!parsed.request.has_value()) {
            return parsed.error.dump();
        }
        const auto &request = *parsed.request;
        if (request.type == "admit_authority" || request.type == "revoke_authority" ||
            request.type == "handback_authority") {
            return handle_authority_request(request).dump();
        }
        if (is_mutating(request.type)) {
            return handle_mutating_request(request).dump();
        }
        return handle_read_request(request).dump();
    }

    Json handle_mutating_request(const Request &request) {
        if (!config_.actuation_enabled) {
            return error_response(request.id, "missing_api_key", "NOMAD_API_KEY is not set for the runtime");
        }
        const auto admission = check_request_authority(request, false);
        if (admission.has_value()) {
            return *admission;
        }
        const auto key = cache_key(request);
        const auto fingerprint = request.original.dump();
        {
            std::lock_guard lock(cache_mutex_);
            const auto cached = response_cache_.find(key);
            if (cached != response_cache_.end()) {
                if (cached->second.fingerprint != fingerprint) {
                    return error_response(request.id, "request_id_conflict", "request ID was used with different data");
                }
                return cached->second.response;
            }
            if (const auto expired = check_request_authority(request); expired.has_value()) {
                return *expired;
            }
            if (in_flight_.contains(key)) {
                return error_response(request.id, "request_in_progress", "request with this ID is still running");
            }
            if (!reserve_sequence(request)) {
                return error_response(request.id, "stale_request", "request sequence was already consumed");
            }
            in_flight_.insert(key);
        }

        Json response;
        try {
            response = execute_mutating_request(request);
        } catch (...) {
            response = error_response(request.id, "internal_error", "vehicle request failed internally");
        }
        remember_response(key, fingerprint, response);
        return response;
    }

    Json handle_read_request(const Request &request) {
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
                    {"authority", {{"vehicle_session", connection_->get_state().session_id},
                                    {"generation", current_generation()},
                                    {"next_sequence", next_sequence()},
                                    {"server_time_ms", unix_milliseconds()}}},
                    {"capabilities",
                     {"hello", "ping", "status", "set_servo", "set_relay", "motor_test",
                      "configure_gimbal", "set_gimbal_target", "admit_authority", "revoke_authority",
                      "handback_authority"}}};
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

    Json status_snapshot() const {
        const auto state = connection_->get_state();
        const bool connection_open = connection_->is_connected();
        const auto identity = state.identity.aircraft_class;
        std::string owner;
        std::uint64_t generation;
        {
            std::shared_lock lock(authority_gate_->mutex);
            owner = authority_gate_->owner;
            generation = authority_gate_->generation;
        }
        return {{"runtime_ready", ready()},
                {"runtime_incarnation", incarnation_},
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

    Json execute_mutating_request(const Request &request) {
        if (!config_.actuation_enabled) {
            return error_response(request.id, "missing_api_key", "NOMAD_API_KEY is not set for the runtime");
        }
        std::unique_lock command_lock(command_mutex_, std::try_to_lock);
        if (!command_lock.owns_lock()) {
            return error_response(request.id, "busy", "another NOMAD command is still executing");
        }
        if (const auto denied = check_request_authority(request); denied.has_value()) {
            return *denied;
        }
        ActiveRequest active(request);
        const auto result = invoke_vehicle(request);
        if (!owns_generation(request)) {
            return error_response(request.id, "authority_interrupted", "authority changed during vehicle operation");
        }
        Json response{{"protocol", kProtocolName}, {"version", kProtocolVersion}, {"id", request.id},
                      {"ok", true}, {"type", "command_response"},
                      {"command_result", {{"success", result.success}, {"message", result.message}}}};
        return response;
    }

    vehicle::CommandResult invoke_vehicle(const Request &request) {
        if (request.type == "set_servo") {
            return vehicle_.set_servo(request.channel, request.pwm_microseconds);
        }
        if (request.type == "set_relay") {
            return vehicle_.set_relay(request.relay_number, request.relay_on);
        }
        if (request.type == "motor_test") {
            return vehicle_.motor_test(request.motor_instance, request.pwm_microseconds,
                                        static_cast<float>(request.timeout_seconds));
        }
        if (request.type == "configure_gimbal") {
            return vehicle_.configure_gimbal(request.mount_mode);
        }
        if (request.type == "set_gimbal_target") {
            return vehicle_.set_gimbal_target(request.pitch_deg, request.roll_deg);
        }
        return {false, "request type is not supported in protocol v1"};
    }

    void remember_response(const std::string &key, const std::string &fingerprint, const Json &response) {
        std::lock_guard lock(cache_mutex_);
        in_flight_.erase(key);
        response_cache_[key] = CacheEntry{fingerprint, response};
        cache_order_.push_back(key);
        while (cache_order_.size() > kRequestCacheCapacity) {
            const auto oldest = std::move(cache_order_.front());
            cache_order_.pop_front();
            response_cache_.erase(oldest);
        }
    }

    bool valid_context(const Request &request, std::uint64_t session) const {
        return request.incarnation == authority_gate_->incarnation && request.session == session && session != 0 &&
               session == authority_gate_->vehicle_session && request.generation == authority_gate_->generation;
    }

    bool valid_expiry(const Request &request) const {
        const auto now = unix_milliseconds();
        return request.expires_at_ms >= now && request.expires_at_ms <= now + 5000;
    }

    std::optional<Json> check_request_authority(const Request &request, bool require_expiry = true) {
        const auto state = connection_->get_state();
        const bool connection_open = connection_->is_connected();
        {
            std::unique_lock lock(authority_gate_->mutex);
            revoke_changed_session_locked(state, connection_open);
        }
        std::shared_lock lock(authority_gate_->mutex);
        if (!valid_context(request, state.session_id) || !connection_open || !state.connected ||
            !state.heartbeat_fresh) {
            return error_response(request.id, "stale_authority", "runtime, vehicle session or generation changed");
        }
        if (authority_gate_->stopping || authority_gate_->owner.empty() ||
            request.source != authority_gate_->owner || request.client_id != authority_gate_->owner) {
            return error_response(request.id, "not_authoritative", "client is not the admitted command source");
        }
        if (require_expiry && !valid_expiry(request)) {
            return error_response(request.id, "expired_request", "request validity must end within five seconds");
        }
        if (request.sequence == 0) {
            return error_response(request.id, "invalid_request", "mutation requires a positive sequence");
        }
        return std::nullopt;
    }

    bool owns_generation(const Request &request) const {
        const auto state = connection_->get_state();
        const bool connection_open = connection_->is_connected();
        std::shared_lock lock(authority_gate_->mutex);
        return !authority_gate_->stopping && valid_context(request, state.session_id) &&
               authority_gate_->owner == request.source && authority_gate_->owner == request.client_id &&
               authority_gate_->owner_session == state.session_id && state.connected && connection_open;
    }

    bool reserve_sequence(const Request &request) {
        std::unique_lock lock(authority_gate_->mutex);
        if (authority_gate_->stopping || request.generation != authority_gate_->generation ||
            request.sequence <= authority_gate_->last_sequence) {
            return false;
        }
        authority_gate_->last_sequence = request.sequence;
        return true;
    }

    Json handle_authority_request(const Request &request) {
        const auto state = connection_->get_state();
        const bool connection_open = connection_->is_connected();
        std::unique_lock lock(authority_gate_->mutex);
        revoke_changed_session_locked(state, connection_open);
        if (!config_.actuation_enabled || !valid_context(request, state.session_id) || !valid_expiry(request)) {
            return error_response(request.id, "stale_authority", "authority context or request validity is stale");
        }
        if (authority_gate_->stopping) {
            return error_response(request.id, "stale_authority", "runtime is stopping");
        }
        if (request.type == "revoke_authority") {
            ++authority_gate_->generation;
            authority_gate_->owner.clear();
            authority_gate_->owner_session = 0;
            authority_gate_->last_sequence = 0;
            return authority_response(request);
        }
        if (!authority_gate_->owner.empty() || !connection_open || !state.connected || !state.heartbeat_fresh ||
            request.source.empty() || request.source != request.client_id || request.source.size() > 64) {
            return error_response(request.id, "authority_unavailable", "source or fresh aircraft state is unavailable");
        }
        const bool handback = request.type == "handback_authority";
        if (handback == !authority_gate_->ever_admitted) {
            return error_response(request.id, "invalid_handover", "use admission first and handback after revocation");
        }
        ++authority_gate_->generation;
        authority_gate_->owner = request.source;
        authority_gate_->owner_session = state.session_id;
        authority_gate_->ever_admitted = true;
        authority_gate_->last_sequence = 0;
        return authority_response(request);
    }

    Json authority_response(const Request &request) const {
        return {{"protocol", kProtocolName}, {"version", kProtocolVersion}, {"id", request.id},
                {"ok", true}, {"type", "authority_response"},
                {"authority_generation", authority_gate_->generation},
                {"authority_owner", authority_gate_->owner.empty() ? Json(nullptr) : Json(authority_gate_->owner)}};
    }

    const std::string incarnation_;
    std::shared_ptr<AuthorityGate> authority_gate_;
    std::unique_ptr<mavlink::MavlinkConnection> connection_;
    RuntimeConfig config_;
    vehicle::Vehicle vehicle_;
    detail::IpcServer server_;
    std::atomic_bool stopping_{false};
    std::thread connection_worker_;
    std::mutex command_mutex_;
    std::mutex cache_mutex_;
    std::unordered_map<std::string, CacheEntry> response_cache_;
    std::deque<std::string> cache_order_;
    std::unordered_set<std::string> in_flight_;
};

Runtime::Runtime(std::unique_ptr<mavlink::MavlinkConnection> connection, RuntimeConfig config)
    : implementation_(std::make_unique<Implementation>(std::move(connection), std::move(config))) {}

Runtime::~Runtime() {
    stop();
}

bool Runtime::start(std::string &error) {
    return implementation_->start(error);
}

void Runtime::stop() {
    if (implementation_ != nullptr) {
        implementation_->stop();
    }
}

bool Runtime::ready() const {
    return implementation_ != nullptr && implementation_->ready();
}

} // namespace nomad::runtime
