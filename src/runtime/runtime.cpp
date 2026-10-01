// SPDX-License-Identifier: Apache-2.0
#include "nomad/runtime/runtime.hpp"

#include "ipc_server.hpp"
#include "audit_journal.hpp"
#include "client_auth.hpp"
#include "auth_proof.hpp"
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
        : incarnation_(new_incarnation()), journal_(std::make_shared<detail::AuditJournal>(config.audit_write_guard)),
          authority_gate_(std::make_shared<AuthorityGate>()),
          connection_(std::move(connection)), config_(std::move(config)),
          vehicle_(require_connection(connection_), make_vehicle_config(config_)) {
        authority_gate_->incarnation = incarnation_;
        const std::weak_ptr<AuthorityGate> weak_gate = authority_gate_;
        const auto journal = journal_;
        connection_->set_transmission_admission_factory([weak_gate, journal] {
            if (active_request == nullptr) {
                return mavlink::TransmissionAdmission([](const std::function<void()> &) { return false; });
            }
            const auto &request = *active_request;
            const auto evidence = request.admission_check_passed;
            const mavlink::SendAuthorityToken token{request.incarnation, request.session, request.generation,
                                                    request.source, request.id, request.sequence,
                                                    request.expires_at_ms};
            return mavlink::TransmissionAdmission([weak_gate, journal, token, evidence](const auto &send) {
                const auto gate = weak_gate.lock();
                if (!gate || !send) {
                    return false;
                }
                std::shared_lock lock(gate->mutex);
                if (!matches_authority(*gate, token)) {
                    return false;
                }
                return journal->admit_send([&] {
                    *evidence = true;
                    send();
                });
            });
        });
        connection_->set_vehicle_session_changed_handler([weak_gate, journal](std::uint64_t session) {
            const auto gate = weak_gate.lock();
            if (!gate) {
                return;
            }
            std::unique_lock lock(gate->mutex);
            if (gate->vehicle_session != session) {
                journal->append({{"event", gate->owner.empty() ? "vehicle_session" : "session_authority_loss"},
                                 {"vehicle_session", session}, {"previous_session", gate->vehicle_session},
                                 {"authority_generation", gate->generation},
                                 {"resulting_generation", gate->generation + (gate->owner.empty() ? 0 : 1)},
                                 {"client", gate->owner}});
            }
            update_gate_session(*gate, session);
        });
    }

    bool start(std::string &error) {
        if (!detail::valid_credentials(config_.client_credentials)) {
            error = "valid client authentication configuration required";
            return false;
        }
        if (!journal_->start(config_.audit_directory, incarnation_) ||
            !journal_->append({{"event", "runtime_start"}, {"vehicle_session", 0}, {"authority_generation", 0}})) {
            error = "durable audit startup failed; mutations inhibited";
            return false;
        }
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

    void request_stop() {
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
    }

    bool stop() {
        request_stop();
        const bool was_running = server_.running();
        server_.stop();
        if (connection_worker_.joinable()) {
            connection_worker_.join();
        }
        connection_->disconnect();
        if (journal_->healthy()) {
            shutdown_recorded_ = journal_->append(
                {{"event", "runtime_shutdown"}, {"authority_generation", current_generation()}});
        } else if (was_running) {
            shutdown_recorded_ = false;
        }
        journal_->stop();
        return shutdown_recorded_;
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
                audit_session_loss(state.session_id);
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
        audit_session_loss(state.session_id);
        ++authority_gate_->generation;
        authority_gate_->owner.clear();
        authority_gate_->owner_session = 0;
        authority_gate_->last_sequence = 0;
        authority_gate_->vehicle_session = state.session_id;
    }

    std::string handle_message(std::string_view line) {
        return detail::redact_credentials(handle_authenticated_message(line).dump(), config_.client_credentials);
    }

    Json handle_authenticated_message(std::string_view line) {
        if (line.size() > detail::kMaximumMessageBytes || !has_reasonable_json_depth(line)) {
            return parse_request(line).error;
        }
        auto envelope = Json::parse(line, nullptr, false);
        const auto type = envelope.is_object() ? field_string(envelope, "type") : "";
        const bool protected_request = is_mutating(type) || type == "admit_authority" ||
                                       type == "revoke_authority" || type == "handback_authority";
        if (protected_request && !authenticate_request(envelope)) {
            const auto id = field_string(envelope, "id");
            return journal_->healthy() ?
                error_response(id.size() <= 64 ? id : "", "authentication_failed",
                               "valid credential proof and matching client identity required") :
                rejected_audit_error(id);
        }
        const auto authenticated_client = protected_request ? field_string(envelope, "client_id") : "";
        if (envelope.is_object()) {
            envelope.erase("auth_payload");
            envelope.erase("auth_proof");
            envelope.erase("credential");
        }
        auto parsed = parse_request(envelope.is_discarded() ? line : envelope.dump());
        if (!parsed.request.has_value()) {
            if (protected_request) {
                if (!audit_rejection(envelope, "invalid_request", authenticated_client)) {
                    return rejected_audit_error(field_string(envelope, "id"));
                }
            }
            return parsed.error;
        }
        const auto &request = *parsed.request;
        if (request.type == "admit_authority" || request.type == "revoke_authority" ||
            request.type == "handback_authority") {
            const auto response = handle_authority_request(request);
            if (!response.value("ok", false)) {
                if (!audit_request(request, "request_rejected", response["error"]["code"])) {
                    return audit_error(request);
                }
            }
            return response;
        }
        if (is_mutating(request.type)) {
            return handle_mutating_request(request);
        }
        return handle_read_request(request);
    }

    Json handle_mutating_request(const Request &request) {
        const auto response = process_mutating_request(request);
        if (!response.value("ok", false) && response["error"]["code"] != "authority_interrupted" &&
            response["error"]["code"] != "internal_error" && response["error"]["code"] != "audit_failure") {
            if (!audit_request(request, "request_rejected", response["error"]["code"])) {
                return audit_error(request);
            }
        }
        return response;
    }

    Json process_mutating_request(const Request &request) {
        if (!journal_->healthy()) {
            return audit_error(request);
        }
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
            response = finish_operation(request, response, "unknown");
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
                    {"client_authentication", "hmac-sha256-v1"},
                    {"server_proof", hello_proof(request)},
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

#include "runtime_status_methods.hpp"

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
        if (!journal_->healthy()) {
            return audit_error(request);
        }
        if (!audit_request(request, "mutation_intent", "pending")) {
            return audit_error(request);
        }
        ActiveRequest active(request);
        const auto result = invoke_vehicle(request);
        *request.ack_observed = result.acknowledged;
        *request.observed_success = result.success;
        if (!owns_generation(request)) {
            return finish_operation(request,
                error_response(request.id, "authority_interrupted", "authority changed during vehicle operation"),
                "interrupted");
        }
        Json response{{"protocol", kProtocolName}, {"version", kProtocolVersion}, {"id", request.id},
                      {"ok", true}, {"type", "command_response"},
                      {"command_result", {{"success", result.success}, {"message", result.message},
                                          {"acknowledged", result.acknowledged}}}};
        const auto outcome = result.success ? "success" : result.acknowledged ? "failed" :
                             request.admission_check_passed->load() ? "unknown" : "rejected";
        response["outcome"] = outcome;
        return finish_operation(request, response, outcome);
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

#include "runtime_authority_methods.hpp"

#include "runtime_audit_methods.hpp"

    Json authority_response(const Request &request) const {
        return {{"protocol", kProtocolName}, {"version", kProtocolVersion}, {"id", request.id},
                {"ok", true}, {"type", "authority_response"},
                {"authority_generation", authority_gate_->generation},
                {"authority_owner", authority_gate_->owner.empty() ? Json(nullptr) : Json(authority_gate_->owner)}};
    }

    const std::string incarnation_;
    std::shared_ptr<detail::AuditJournal> journal_;
    std::shared_ptr<AuthorityGate> authority_gate_;
    std::unique_ptr<mavlink::MavlinkConnection> connection_;
    RuntimeConfig config_;
    vehicle::Vehicle vehicle_;
    detail::IpcServer server_;
    std::atomic_bool stopping_{false};
    bool shutdown_recorded_{true};
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

void Runtime::request_stop() {
    implementation_->request_stop();
}

bool Runtime::stop() {
    if (implementation_ != nullptr) {
        return implementation_->stop();
    }
    return true;
}

bool Runtime::ready() const {
    return implementation_ != nullptr && implementation_->ready();
}

} // namespace nomad::runtime
