// SPDX-License-Identifier: Apache-2.0
#include "runtime_implementation.hpp"
#include "auth_proof.hpp"
#include "client_auth.hpp"

namespace nomad::runtime {

using namespace detail;

Runtime::Implementation::Implementation(std::unique_ptr<mavlink::MavlinkConnection> connection, RuntimeConfig config)
    : incarnation_(new_incarnation()), journal_(std::make_shared<detail::AuditJournal>(config.audit_write_guard)),
      authority_gate_(std::make_shared<AuthorityGate>()),
      connection_(std::move(connection)), config_(std::move(config)),
      vehicle_(require_connection(connection_), vehicle::VehicleConfig{}), actuators_(config_.actuators) {
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

bool Runtime::Implementation::start(std::string &error) {
    if (!validate_actuator_definitions(config_.actuators, error)) {
        return false;
    }
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

void Runtime::Implementation::request_stop() {
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

bool Runtime::Implementation::stop() {
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

bool Runtime::Implementation::ready() const {
    return server_.running();
}

void Runtime::Implementation::maintain_connection() {
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

std::string Runtime::Implementation::handle_message(std::string_view line) {
    return detail::redact_credentials(handle_authenticated_message(line).dump(), config_.client_credentials);
}

Json Runtime::Implementation::handle_authenticated_message(std::string_view line) {
    if (line.size() > detail::kMaximumMessageBytes || !has_reasonable_json_depth(line)) {
        return parse_request(line).error;
    }
    auto envelope = Json::parse(line, nullptr, false);
    const auto type = envelope.is_object() ? field_string(envelope, "type") : "";
    const bool protected_request = is_mutating(type) || type == "get_actuators" || type == "admit_authority" ||
                                   type == "revoke_authority" || type == "handback_authority";
    if (protected_request && !authenticate_request(envelope)) {
        const auto id = field_string(envelope, "id");
        auto response = journal_->healthy() ?
            error_response(id.size() <= 64 ? id : "", "authentication_failed",
                           "valid credential proof and matching client identity required") :
            rejected_audit_error(id);
        if (is_mutating(type)) {
            response["outcome"] = "rejected";
        }
        return response;
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
        if (is_mutating(type)) {
            parsed.error["outcome"] = "rejected";
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
