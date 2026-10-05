// SPDX-License-Identifier: Apache-2.0
#include "runtime_implementation.hpp"

namespace nomad::runtime {

using namespace detail;

Json Runtime::Implementation::handle_mutating_request(const Request &request) {
    auto response = process_mutating_request(request);
    if (!response.contains("outcome")) {
        response["outcome"] = "rejected";
    }
    if (!response.value("ok", false) && response["error"]["code"] != "authority_interrupted" &&
        response["error"]["code"] != "internal_error" && response["error"]["code"] != "audit_failure") {
        if (!audit_request(request, "request_rejected", response["error"]["code"], response["outcome"])) {
            return audit_error(request, response["outcome"] != "rejected");
        }
    }
    return response;
}

Json Runtime::Implementation::process_mutating_request(const Request &request) {
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
        if (in_flight_.contains(key)) {
            auto response = error_response(request.id, "request_in_progress",
                                           "request with this ID is still running");
            response["outcome"] = "unknown";
            return response;
        }
        if (!journal_->healthy()) {
            return audit_error(request);
        }
        if (const auto expired = check_request_authority(request); expired.has_value()) {
            return *expired;
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
        const auto outcome = request.admission_check_passed->load() || request.ack_observed->load() ?
                             "unknown" : "rejected";
        response = finish_operation(request, response, outcome);
    }
    remember_response(key, fingerprint, response);
    return response;
}

Json Runtime::Implementation::execute_mutating_request(const Request &request) {
    if (!config_.actuation_enabled) {
        return error_response(request.id, "missing_api_key", "NOMAD_API_KEY is not set for the runtime");
    }
    if (request.type == "actuator_action" || request.type == "configure_actuators") {
        return execute_actuator_request(request);
    }
    std::unique_lock command_lock(command_mutex_, std::try_to_lock);
    if (!command_lock.owns_lock()) {
        return error_response(request.id, "busy", "another NOMAD command is still executing");
    }
    if (((request.type == "set_servo" || request.type == "set_relay") && actuator_configuration_recovery_.load()) ||
        (request.type == "set_servo" && actuators_.contains_output(request.channel, false)) ||
        (request.type == "set_relay" && actuators_.contains_output(request.relay_number, true))) {
        return error_response(request.id, "configured_output",
              "use semantic actuator actions; raw configured outputs are blocked");
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

vehicle::CommandResult Runtime::Implementation::invoke_vehicle(const Request &request) {
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

void Runtime::Implementation::remember_response(
    const std::string &key, const std::string &fingerprint, const Json &response) {
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

} // namespace nomad::runtime
