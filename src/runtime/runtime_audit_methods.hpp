// SPDX-License-Identifier: Apache-2.0
// Private Implementation methods; included inside the runtime owner.

    Json audit_error(const Request &request, bool possible_send = false) const {
        auto response = error_response(request.id, "audit_failure",
                                       "durable audit failed; mutations inhibited until restart");
        response["outcome"] = possible_send ? "unknown" : "rejected";
        return response;
    }

    Json rejected_audit_error(const std::string &id) const {
        auto response = error_response(id.size() <= 64 ? id : "", "audit_failure",
                                       "durable audit failed; mutations inhibited until restart");
        response["outcome"] = "rejected";
        return response;
    }

    void audit_session_loss(std::uint64_t session) {
        journal_->append({{"event", authority_gate_->owner.empty() ? "vehicle_session" : "session_authority_loss"},
                          {"vehicle_session", session}, {"previous_session", authority_gate_->vehicle_session},
                          {"authority_generation", authority_gate_->generation},
                          {"resulting_generation", authority_gate_->generation +
                              (authority_gate_->owner.empty() ? 0 : 1)},
                          {"client", authority_gate_->owner}});
    }

    bool audit_rejection(const Json &envelope, const std::string &reason, std::string identity = {}) {
        if (identity.empty()) {
            identity = authenticated_identity(envelope);
        }
        Json record{{"event", "request_rejected"}, {"client", identity.empty() ? Json(nullptr) : Json(identity)},
                    {"reason", reason}, {"send_eligible", false}, {"operation", field_string(envelope, "type")}};
        if (!identity.empty()) {
            record["request_id"] = field_string(envelope, "id");
        }
        for (const auto *field : {"vehicle_session", "authority_generation", "sequence", "expires_at_ms"}) {
            if (envelope.contains(field) && envelope[field].is_number_integer()) {
                record[std::string("claimed_") + field] = envelope[field];
            }
        }
        const auto incarnation = field_string(envelope, "runtime_incarnation");
        if (incarnation.size() <= 64) {
            record["claimed_incarnation"] = incarnation;
        }
        return journal_->append(Json::parse(detail::redact_credentials(record.dump(), config_.client_credentials)));
    }

    bool authenticate_request(Json &envelope) {
        const auto identity = authenticated_identity(envelope);
        if (identity.empty() || field_string(envelope, "client_id") != identity ||
            field_string(envelope, "command_source") != identity ||
            (envelope.contains("source") && field_string(envelope, "source") != identity)) {
            audit_rejection(envelope, "authentication_failed");
            return false;
        }
        envelope["client_id"] = identity;
        envelope["command_source"] = identity;
        return true;
    }

    std::string authenticated_identity(const Json &envelope) const {
        const auto payload = field_string(envelope, "auth_payload");
        if (payload.empty() || payload.size() > 32768 || !has_reasonable_json_depth(payload)) {
            return {};
        }
        auto unsigned_request = envelope;
        unsigned_request.erase("auth_payload");
        unsigned_request.erase("auth_proof");
        if (envelope.contains("credential") || Json::parse(payload, nullptr, false) != unsigned_request) {
            return {};
        }
        std::string identity;
        for (const auto &[client, secret] : config_.client_credentials) {
            if (detail::equal_proof(field_string(envelope, "auth_proof"),
                                    detail::make_proof(secret, "nomad-core:request:v1:" + payload))) {
                identity = client;
            }
        }
        return identity;
    }

    std::string hello_proof(const Request &request) const {
        const auto found = config_.client_credentials.find(request.client_id);
        const auto nonce = field_string(request.original, "auth_nonce");
        if (found == config_.client_credentials.end() || nonce.size() != 64) {
            return {};
        }
        return detail::make_proof(found->second, "nomad-core:server:v1:" + request.client_id + ":" + nonce +
                                               ":" + incarnation_);
    }

    Json request_record(const Request &request, const std::string &event, const std::string &result) {
        Json normalized = Json::object();
        if (request.type == "set_servo") {
            normalized = {{"channel", request.channel}, {"pwm_microseconds", request.pwm_microseconds}};
        } else if (request.type == "set_relay") {
            normalized = {{"relay_number", request.relay_number}, {"on", request.relay_on}};
        } else if (request.type == "motor_test") {
            normalized = {{"motor_instance", request.motor_instance}, {"pwm_microseconds", request.pwm_microseconds},
                          {"timeout_seconds", request.timeout_seconds}};
        } else if (request.type == "configure_gimbal") {
            normalized = {{"mount_mode", request.mount_mode}};
        } else if (request.type == "set_gimbal_target") {
            normalized = {{"pitch_deg", request.pitch_deg}, {"roll_deg", request.roll_deg}};
        }
        Json record{{"event", event}, {"client", request.client_id}, {"request_id", request.id},
                {"request_incarnation", request.incarnation}, {"vehicle_session", request.session},
                {"authority_generation", request.generation}, {"sequence", request.sequence},
                {"expires_at_ms", request.expires_at_ms}, {"operation", request.type},
                {"normalized_request", normalized}, {"result", result},
                {"send_eligible", request.admission_check_passed->load() ? Json("unknown") : Json(false)},
                {"admission_checked", request.admission_check_passed->load()}};
        if (request.ack_observed->load()) {
            record["send_eligible"] = true;
        }
        if (event == "mutation_outcome") {
            record["acknowledged"] = request.ack_observed->load();
            record["observed_command_success"] = request.observed_success->load();
        }
        return Json::parse(detail::redact_credentials(record.dump(), config_.client_credentials));
    }

    bool audit_request(const Request &request, const std::string &event, const std::string &result) {
        auto record = request_record(request, event, result);
        {
            std::shared_lock lock(authority_gate_->mutex);
            record["observed_session"] = authority_gate_->vehicle_session;
            record["observed_generation"] = authority_gate_->generation;
        }
        const auto sanitized = detail::redact_credentials(record.dump(), config_.client_credentials);
        return journal_->append(Json::parse(sanitized));
    }

    Json finish_operation(const Request &request, Json response, const std::string &outcome) {
        if (!audit_request(request, "mutation_outcome", outcome)) {
            return audit_error(request, request.admission_check_passed->load() || request.ack_observed->load());
        }
        return response;
    }
