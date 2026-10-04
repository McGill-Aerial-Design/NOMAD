// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "nomad/runtime/runtime.hpp"
#include "nomad/vehicle/vehicle.hpp"

#include "audit_journal.hpp"
#include "ipc_server.hpp"
#include "runtime_detail.hpp"

#include <deque>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <unordered_set>

namespace nomad::runtime {

// Ownership and synchronization:
// - config_/incarnation_ are fixed after construction; Vehicle borrows connection_.
// - authority_gate_->mutex protects every gate field: exclusive changes, shared reads/final send.
// - command_mutex_ serializes Vehicle mutations; revoke/status/session callbacks do not take it.
// - cache_mutex_ protects response_cache_, cache_order_ and in_flight_; released before execution.
// - AuditJournal owns its mutex/files; final admission holds gate then journal locks through send.
// - Nested locks: command OR cache -> gate -> journal; never acquire command/cache under gate.
// - *_locked, valid_context, authority_response, record_authority_response and audit_session_loss
//   require the caller's gate lock. audit_request releases its gate snapshot lock before append.
// - MAVSDK callbacks retain a weak gate/shared journal, never this; request evidence is shared atomic.
// - One lifecycle caller owns server_/worker_/shutdown_recorded_; stopping_ is atomic.
// - Shutdown fences the gate, drains IPC, joins the worker, disconnects, then closes the journal.
// Keep member order: Vehicle is destroyed before its connection; gate/journal outlive the connection.
struct Runtime::Implementation {
    using Json = detail::Json;
    using Request = detail::Request;

    struct CacheEntry {
        std::string fingerprint;
        Json response;
    };

    // Lifecycle and IPC.
    Implementation(std::unique_ptr<mavlink::MavlinkConnection> connection, RuntimeConfig config);
    bool start(std::string &error);
    void request_stop();
    bool stop();
    bool ready() const;
    void maintain_connection();
    std::string handle_message(std::string_view line);
    Json handle_authenticated_message(std::string_view line);

    // Authority and session fencing.
    void observe_vehicle_session();
    void revoke_changed_session_locked(const telemetry::VehicleState &state, bool connection_open);
    bool valid_context(const Request &request, std::uint64_t session) const;
    bool valid_expiry(const Request &request) const;
    std::optional<Json> check_request_authority(const Request &request, bool require_expiry = true);
    bool owns_generation(const Request &request) const;
    bool reserve_sequence(const Request &request);
    Json handle_authority_request(const Request &request);
    Json record_authority_response(const Request &request, const std::string &event);
    Json authority_response(const Request &request) const;

    // Mutation execution and response cache.
    Json handle_mutating_request(const Request &request);
    Json process_mutating_request(const Request &request);
    Json execute_mutating_request(const Request &request);
    vehicle::CommandResult invoke_vehicle(const Request &request);
    void remember_response(const std::string &key, const std::string &fingerprint, const Json &response);

    // Read requests and status.
    Json handle_read_request(const Request &request);
    Json status_snapshot() const;
    std::uint64_t current_generation() const;
    std::uint64_t next_sequence() const;

    // Audit and authentication.
    Json audit_error(const Request &request, bool possible_send = false) const;
    Json rejected_audit_error(const std::string &id) const;
    void audit_session_loss(std::uint64_t session);
    bool audit_rejection(const Json &envelope, const std::string &reason, std::string identity = {});
    bool authenticate_request(Json &envelope);
    std::string authenticated_identity(const Json &envelope) const;
    std::string hello_proof(const Request &request) const;
    Json request_record(const Request &request, const std::string &event, const std::string &result);
    bool audit_request(const Request &request, const std::string &event, const std::string &result,
                       const std::string &outcome = "rejected");
    Json finish_operation(const Request &request, Json response, const std::string &outcome);

    const std::string incarnation_;
    std::shared_ptr<detail::AuditJournal> journal_;
    std::shared_ptr<detail::AuthorityGate> authority_gate_;
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

} // namespace nomad::runtime
