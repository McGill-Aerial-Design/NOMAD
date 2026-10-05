// SPDX-License-Identifier: Apache-2.0
#include "runtime_implementation.hpp"
#include "actuator_storage.hpp"
#include "actuator_sequence.hpp"

namespace nomad::runtime {
using namespace detail;
namespace {

Request stage_request(const Request &request) {
    auto stage = request;
    stage.admission_check_passed = std::make_shared<std::atomic_bool>(false);
    stage.ack_observed = std::make_shared<std::atomic_bool>(false);
    stage.observed_success = std::make_shared<std::atomic_bool>(false);
    return stage;
}

void merge_evidence(const Request &request, const Request &stage, const vehicle::CommandResult &result) {
    if (stage.admission_check_passed->load()) {
        *request.admission_check_passed = true;
    }
    if (result.acknowledged || stage.ack_observed->load()) {
        *request.ack_observed = true;
    }
}

ActuatorInput actuator_input(const Request &request, const std::string &authority) {
    const auto &body = request.original;
    ActuatorInput input{body["actuator_id"], body["operation"], body["input_source"], -1, {}, authority};
    if (body.contains("input_slot")) {
        input.slot = body["input_slot"].get<int>();
    }
    if (body.contains("value")) {
        input.value = body["value"].get<double>();
    }
    input.sequence = request.sequence;
    return input;
}

Json actuator_response(const Request &request, bool success, const std::string &message, bool executed,
                       bool observed_command_success = false) {
    return {{"protocol", kProtocolName}, {"version", kProtocolVersion}, {"id", request.id},
            {"ok", true}, {"type", "actuator_response"}, {"execution_attempted", executed},
            {"request_result", {{"success", success}, {"message", message}}},
            {"command_result", {{"success", executed && observed_command_success}, {"message", message},
                                {"acknowledged", request.ack_observed->load()}}}};
}

} // namespace

std::string Runtime::Implementation::actuator_authority() const {
    std::shared_lock lock(authority_gate_->mutex);
    return authority_gate_->incarnation + ":" + std::to_string(authority_gate_->vehicle_session) + ":" +
           std::to_string(authority_gate_->generation) + ":" + authority_gate_->owner;
}

Json Runtime::Implementation::configure_actuators(const Request &request) {
    actuators_.discover(actuator_authority());
    if (actuator_configuration_recovery_.load() || config_.actuator_config_file.empty() ||
        connection_->get_state().armed || !actuators_.can_configure()) {
        return error_response(request.id, "actuator_configuration_blocked",
            "configuration requires an explicit path, disarmed aircraft and no active/pending/recovery output");
    }
    std::vector<ActuatorDefinition> candidate;
    std::string error;
    if (!parse_actuator_definitions(request.original["actuator_configs"], candidate, error)) {
        return error_response(request.id, "invalid_actuator_configuration", error);
    }
    bool replacement_attempted = false;
    bool saved = false;
    try {
        saved = save_actuator_configuration(config_.actuator_config_file, candidate, error, replacement_attempted,
                                            config_.actuator_directory_sync_guard);
    } catch (...) {
        error = "actuator configuration persistence failed internally";
    }
    if (!saved) {
        if (replacement_attempted) {
            actuator_configuration_recovery_ = true;
            return complete_configuration_save(request, error_response(request.id, "configuration_recovery_required",
                "configuration may have changed on disk; activation inhibited until restart and operator review"),
                "unknown", false);
        }
        return error_response(request.id, "actuator_configuration_not_saved", error);
    }
    if (!owns_generation(request) || connection_->get_state().armed) {
        actuator_configuration_recovery_ = true;
        return complete_configuration_save(request, error_response(request.id, "authority_interrupted",
            "configuration changed on disk but authority, session or disarmed state changed; "
                  "restart and review required"),
            "interrupted", true);
    }
    actuators_.replace(std::move(candidate));
    auto response = actuator_response(request, true,
        "backend configuration saved; explicit safe commands required", false);
    response["runtime_incarnation"] = incarnation_;
    response["actuators"] = actuators_.discover(actuator_authority());
    return complete_configuration_save(request, response, "success", true);
}

Json Runtime::Implementation::complete_configuration_save(
    const Request &request, Json response, const std::string &outcome, bool change_known) {
    response = finish_operation(request, std::move(response), outcome);
    if (response.value("outcome", std::string("unknown")) != outcome) {
        response["outcome"] = "unknown";
    }
    if (response.value("outcome", std::string("unknown")) != "success") {
        actuator_configuration_recovery_ = true;
    }
    response["runtime_incarnation"] = incarnation_;
    response["configuration_changed"] = change_known;
    response["configuration_may_have_changed"] = !change_known;
    response["configuration_recovery_required"] = actuator_configuration_recovery_.load();
    return response;
}

Json Runtime::Implementation::execute_actuator_request(const Request &request) {
    if (const auto denied = check_request_authority(request); denied.has_value()) {
        return *denied;
    }
    const auto operation = field_string(request.original, "operation");
    const auto id = field_string(request.original, "actuator_id");
    bool safe = operation == "safe" || operation == "stop";
    bool intent_recorded = false;
    if (operation == "release_input") {
        if (const auto completed = process_actuator_release(request, safe); completed.has_value()) {
            return *completed;
        }
        intent_recorded = true;
    }
    if (safe) {
        if (!journal_->healthy() || (!intent_recorded && !audit_request(request, "mutation_intent", "pending"))) {
            return audit_error(request);
        }
        intent_recorded = true;
        actuators_.signal_safe(id);
    }
    std::unique_lock command_lock(command_mutex_, std::defer_lock);
    if (request.type == "configure_actuators" || operation != "neutral") {
        if (safe) {
            command_lock.lock();
        } else if (!command_lock.try_lock()) {
            return error_response(request.id, "busy", "another NOMAD command is still executing");
        }
    }
    if (const auto denied = check_request_authority(request); denied.has_value()) {
        if (intent_recorded) {
            return finish_operation(request, *denied, "rejected");
        }
        return *denied;
    }
    if (actuator_configuration_recovery_.load() && !safe) {
        return error_response(request.id, "configuration_recovery_required",
            "configured activations and reconfiguration are inhibited until restart and operator review");
    }
    if (!journal_->healthy() || (!intent_recorded && !audit_request(request, "mutation_intent", "pending"))) {
        return audit_error(request);
    }
    if (request.type == "configure_actuators") {
        return configure_actuators(request);
    }
    return execute_actuator_decision(request);
}

std::optional<Json> Runtime::Implementation::process_actuator_release(const Request &request, bool &safe) {
    if (!journal_->healthy() || !audit_request(request, "mutation_intent", "pending")) {
        return audit_error(request);
    }
    const auto release = actuators_.release_input(actuator_input(request, actuator_authority()));
    if (!release.allowed) {
        return finish_operation(request, error_response(request.id, "actuator_action_blocked", release.message),
                                "rejected");
    }
    if (release.execute) {
        safe = true;
        return std::nullopt;
    }
    auto response = actuator_response(request, true, release.message, false);
    std::string outcome = "success";
    if (!owns_generation(request)) {
        actuators_.discover(actuator_authority());
        response = error_response(request.id, "authority_interrupted", "authority changed during input release");
        outcome = "interrupted";
    }
    response = finish_operation(request, response, outcome);
    response["runtime_incarnation"] = incarnation_;
    response["actuator_state"] = actuators_.state(field_string(request.original, "actuator_id"));
    return response;
}

Json Runtime::Implementation::execute_actuator_decision(const Request &request) {
    const auto id = field_string(request.original, "actuator_id");
    auto input = actuator_input(request, actuator_authority());
    if (input.operation == "release_input") {
        input.operation = "stop";
    }
    const auto decision = actuators_.begin(input, Clock::now());
    if (!decision.allowed) {
        return finish_operation(request, error_response(request.id, "actuator_action_blocked", decision.message),
                                "rejected");
    }
    if (!decision.execute) {
        auto response = actuator_response(request, true, decision.message, false);
        std::string outcome = "success";
        if (!owns_generation(request)) {
            actuators_.discover(actuator_authority());
            response = error_response(request.id, "authority_interrupted",
                                      "authority changed during actuator confirmation");
            outcome = "interrupted";
        }
        response = finish_operation(request, response, outcome);
        response["runtime_incarnation"] = incarnation_;
        response["actuator_state"] = actuators_.state(id);
        return response;
    }
    if (decision.duration_ms > 0 && request.expires_at_ms - unix_milliseconds() < 3250 + decision.duration_ms) {
        actuators_.finish(decision, "rejected", "not-attempted", "rejected", false);
        return finish_operation(request, error_response(request.id, "insufficient_actuator_budget",
            "request validity must cover the ON acknowledgement budget and bounded run before required safe send"),
            "rejected");
    }
    auto response = execute_actuator_plan(request, decision);
    response["runtime_incarnation"] = incarnation_;
    response["actuator_state"] = actuators_.state(id);
    return response;
}

Json Runtime::Implementation::execute_actuator_plan(const Request &request, const ActuatorDecision &decision) {
    const auto stage = [&](bool safe, int pwm) {
        auto evidence = stage_request(request);
        ActuatorStageResult result;
        try {
            ActiveRequest active(evidence);
            result.command = actuator_is_relay(decision.definition) ?
                vehicle_.set_relay(decision.definition.channel, !safe) :
                vehicle_.set_servo(decision.definition.channel, pwm);
            result.outcome = result.command.success ? "success" : result.command.acknowledged ? "failed" :
                             evidence.admission_check_passed->load() ? "unknown" : "rejected";
        } catch (...) {
            result.command = {false, "actuator command failed internally"};
            result.outcome = evidence.admission_check_passed->load() ? "unknown" : "rejected";
        }
        merge_evidence(request, evidence, result.command);
        if (!owns_generation(request)) {
            result.outcome = "interrupted";
        }
        return result;
    };
    const auto sequence = run_actuator_sequence(decision, stage, [&](int duration) {
        actuators_.wait(decision.definition.id, std::chrono::milliseconds(duration));
    });
    *request.observed_success = sequence.observed_command_success;
    const auto activation_outcome = decision.safe ? "not-attempted" : sequence.initial.outcome;
    const auto safe_outcome = decision.safe ? sequence.initial.outcome : sequence.safe.outcome;
    actuators_.finish(decision, activation_outcome, safe_outcome, sequence.outcome, sequence.activation_success,
                      sequence.observed_command_success);
    auto response = finish_operation(request, actuator_response(request, sequence.success,
        sequence.message + "; physical actuator state unverified", true, sequence.observed_command_success),
        sequence.outcome);
    if (response.value("outcome", std::string("unknown")) != sequence.outcome) {
        actuators_.require_recovery(decision.definition.id);
    }
    return response;
}

} // namespace nomad::runtime
