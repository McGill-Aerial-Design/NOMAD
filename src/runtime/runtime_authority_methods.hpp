// SPDX-License-Identifier: Apache-2.0
// Private authority methods of the runtime owner.

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
        if (!journal_->healthy()) {
            return audit_error(request);
        }
        if (request.type == "revoke_authority") {
            if (!journal_->append(request_record(request, "authority_revoke_intent", "pending"))) {
                return audit_error(request);
            }
            ++authority_gate_->generation;
            authority_gate_->owner.clear();
            authority_gate_->owner_session = 0;
            authority_gate_->last_sequence = 0;
            return record_authority_response(request, "authority_revoke");
        }
        if (!authority_gate_->owner.empty() || !connection_open || !state.connected || !state.heartbeat_fresh ||
            request.source.empty() || request.source != request.client_id || request.source.size() > 64) {
            return error_response(request.id, "authority_unavailable", "source or fresh aircraft state is unavailable");
        }
        const bool handback = request.type == "handback_authority";
        if (handback == !authority_gate_->ever_admitted) {
            return error_response(request.id, "invalid_handover", "use admission first and handback after revocation");
        }
        if (!journal_->append(request_record(request, "authority_intent", "pending"))) {
            return audit_error(request);
        }
        ++authority_gate_->generation;
        authority_gate_->owner = request.source;
        authority_gate_->owner_session = state.session_id;
        authority_gate_->ever_admitted = true;
        authority_gate_->last_sequence = 0;
        return record_authority_response(request, handback ? "authority_handback" : "authority_admission");
    }

    Json record_authority_response(const Request &request, const std::string &event) {
        auto record = request_record(request, event, "success");
        record["observed_generation"] = authority_gate_->generation;
        record["observed_session"] = authority_gate_->vehicle_session;
        if (!journal_->append(record)) {
            return audit_error(request, true);
        }
        return authority_response(request);
    }
