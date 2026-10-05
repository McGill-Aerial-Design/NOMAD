// SPDX-License-Identifier: Apache-2.0
#include "support/fake_connection.hpp"
#include "nomad/runtime/runtime.hpp"
#include "../src/runtime/auth_proof.hpp"
#include "../src/runtime/client_auth.hpp"
#include "../src/runtime/protected_file.hpp"
#include "../src/runtime/actuator_config.hpp"
#include "../src/runtime/actuator_storage.hpp"
#include "../src/runtime/runtime_detail.hpp"
#include "support/test_harness.hpp"
#include "../tools/runtime/lifecycle.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cerrno>
#include <cstdint>
#include <functional>
#include <filesystem>
#include <fstream>
#include <map>
#include <random>
#include <source_location>
#include <future>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>
#endif

namespace {

using Json = nlohmann::json;

#include "runtime_test_clients.hpp"

Json base_request(std::string id, std::string type, std::string client = "test-client") {
    return {{"protocol", "nomad-core"}, {"version", 1}, {"id", std::move(id)},
            {"client_id", client}, {"type", std::move(type)}, {"command_source", client},
            {"credential", test_credentials.contains(client) ? test_credentials.at(client) : ""}};
}

struct AuthorityContext {
    std::string incarnation;
    std::uint64_t session{};
    std::uint64_t generation{};
    std::atomic<std::uint64_t> sequence{0};
};

AuthorityContext authority;

void bind_authority(Json &request) {
    request["runtime_incarnation"] = authority.incarnation;
    request["vehicle_session"] = authority.session;
    request["authority_generation"] = authority.generation;
    request["command_source"] = "test-client";
    request["sequence"] = ++authority.sequence;
    request["expires_at_ms"] = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count() + 3000;
}

Json authority_request(std::string id, std::string type, std::string source = "test-client") {
    auto request = base_request(std::move(id), std::move(type), source);
    bind_authority(request);
    request["command_source"] = source;
    return request;
}

void read_authority(Client &client) {
    const auto status = client.request(base_request("authority-status", "status"))["status"];
    authority.incarnation = status["runtime_incarnation"].get<std::string>();
    authority.session = status["vehicle_session"].get<std::uint64_t>();
    authority.generation = status["authority_generation"].get<std::uint64_t>();
    authority.sequence = 0;
}

void admit_authority(std::uint16_t port) {
    Client client(port);
    read_authority(client);
    const auto admitted = client.request(authority_request("admit", "admit_authority"));
    CHECK(admitted["ok"] == true);
    authority.generation = admitted["authority_generation"].get<std::uint64_t>();
}

Json servo_request(std::string id, int pwm, std::string client = "test-client") {
    auto request = base_request(std::move(id), "set_servo", client);
    request["channel"] = 8;
    request["pwm_microseconds"] = pwm;
    bind_authority(request);
    request["command_source"] = client;
    return request;
}

void wait_until(const std::function<bool()> &predicate) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (std::chrono::steady_clock::now() < deadline) {
        if (predicate()) {
            return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    CHECK(predicate());
}

#include "runtime_gimbal_cases.hpp"
#include "runtime_authority_cases.hpp"
#include "runtime_security_cases.hpp"
#include "runtime_security_recovery_cases.hpp"
#include "runtime_lifecycle_cases.hpp"
#include "runtime_outcome_cases.hpp"
#include "runtime_client_cases.hpp"
#include "runtime_actuator_cases.hpp"
void test_protocol_and_status(std::uint16_t port, FakeConnection &connection) {
    Client client(port);
    const auto hello = client.request(base_request("1", "hello"));
    CHECK(hello["ok"] == true);
    CHECK(hello["protocol"] == "nomad-core");
    CHECK(hello["version"] == 1);
    CHECK(std::find(hello["capabilities"].begin(), hello["capabilities"].end(), "status") !=
          hello["capabilities"].end());
    CHECK(std::find(hello["capabilities"].begin(), hello["capabilities"].end(), "goto_location") ==
          hello["capabilities"].end());
    const auto status = client.request(base_request("2", "status"));
    CHECK(status["status"]["runtime_ready"] == true);
    CHECK(status["status"]["identity_resolved"] == false);
    CHECK(status["status"]["aircraft_class"] == "Unknown");
    CHECK(status["status"]["vehicle_session_established"] == true);

    test_gimbal_target_request_validation(client, connection, hello);

    connection.set_identity(nomad::telemetry::identify_vehicle(nomad::telemetry::kArduPilotAutopilot,
                                                                nomad::telemetry::kQuadrotor));
    const auto resolved = client.request(base_request("3", "status"));
    CHECK(resolved["status"]["identity_resolved"] == true);
    CHECK(resolved["status"]["aircraft_class"] == "Copter");
    CHECK(resolved["status"]["vehicle_connected"] == true);
    CHECK(client.request(base_request("4", "ping"))["type"] == "pong");

    auto read_status = [port] {
        Client independent_client(port);
        return independent_client.request(base_request("parallel-status", "status"));
    };
    auto first_status = std::async(std::launch::async, read_status);
    auto second_status = std::async(std::launch::async, read_status);
    CHECK(first_status.get()["ok"] == true);
    CHECK(second_status.get()["ok"] == true);
}

void test_protocol_errors(std::uint16_t port, FakeConnection &connection) {
    Client client(port);
    const auto incompatible = client.request([] {
        auto request = base_request("v2", "hello");
        request["version"] = 2;
        return request;
    }());
    CHECK(incompatible["error"]["code"] == "incompatible_version");
    Client wrong_protocol(port);
    auto wrong_name = base_request("wrong-name", "hello");
    wrong_name["protocol"] = "nomad-other";
    CHECK(wrong_protocol.request(wrong_name)["error"]["code"] == "incompatible_protocol");
    CHECK(client.request(base_request("unknown", "send_command"))["error"]["code"] == "unsupported_request");
    const auto commands_before_navigation_request = connection.command_count();
    CHECK(client.request(base_request("no-navigation", "goto_location"))["error"]["code"] ==
          "unsupported_request");
    CHECK(connection.command_count() == commands_before_navigation_request);
    CHECK(!connection.last_goto.has_value());
    CHECK(client.request(base_request("no-shell", "execute_shell"))["error"]["code"] == "unsupported_request");
    CHECK(client.request(base_request("no-mavlink", "send_mavlink"))["error"]["code"] ==
          "unsupported_request");
    Client malformed(port);
    malformed.send_raw("{bad");
    CHECK(malformed.receive()["error"]["code"] == "malformed_json");

    Client deeply_nested(port);
    deeply_nested.send_raw(std::string(65, '[') + std::string(65, ']'));
    CHECK(deeply_nested.receive()["error"]["code"] == "malformed_json");

    Client oversized(port);
    oversized.send_raw(std::string(65537, 'x'));
    CHECK(oversized.receive()["error"]["code"] == "message_too_large");
    CHECK(client.request(base_request("after-error", "status"))["ok"] == true);
}

void test_command_dispatch_and_dedupe(std::uint16_t port, FakeConnection &connection) {
    connection.acknowledgement = nomad::mavlink::CommandAck{183, 0};
    const auto request = servo_request("servo-1", 1500);
    Client client(port);
    const auto result = client.request(request);
    CHECK(result["command_result"]["success"] == true);
    CHECK(connection.command_count() == 1);
    const auto hello = client.request(base_request("sequence-after-servo", "hello"));
    CHECK(hello["authority"]["next_sequence"] == request["sequence"].get<std::uint64_t>() + 1);

    const auto duplicate = client.request(request);
    CHECK(duplicate == result);
    CHECK(connection.command_count() == 1);
    auto reused_id = servo_request("servo-1", 1600);
    CHECK(client.request(reused_id)["error"]["code"] == "request_id_conflict");
    CHECK(connection.command_count() == 1);

    CHECK(client.request(servo_request("c", 1500, "a\nb"))["error"]["code"] == "authentication_failed");
    CHECK(client.request(servo_request("b\nc", 1500, "a"))["error"]["code"] == "authentication_failed");
    CHECK(connection.command_count() == 1);
}

void test_vehicle_admission_is_authoritative(std::uint16_t port, FakeConnection &connection) {
    connection.set_identity(nomad::telemetry::identify_vehicle(nomad::telemetry::kArduPilotAutopilot,
                                                                nomad::telemetry::kVtolTiltrotor));
    Client client(port);
    const auto result = client.request(servo_request("unqualified-servo", 1500));
    CHECK(result["command_result"]["success"] == false);
    CHECK(connection.command_count() == 1);
}

void test_busy_and_slow_client(std::uint16_t port, FakeConnection &connection) {
    connection.set_identity(nomad::telemetry::identify_vehicle(nomad::telemetry::kArduPilotAutopilot,
                                                                nomad::telemetry::kQuadrotor));
    connection.command_started = false;
    connection.command_delay = std::chrono::milliseconds(300);
    Client first(port);
    first.send(servo_request("slow-command", 1500));
    wait_until([&connection] { return connection.command_started.load(); });

    Client second(port);
    CHECK(second.request(gimbal_target_request("busy-gimbal-command", 10.0, 5.0))["error"]["code"] == "busy");
    CHECK(second.request(base_request("while-busy", "status"))["ok"] == true);
    const auto completed = first.receive();
    CHECK(completed["command_result"]["success"] == true);
    CHECK(connection.command_count() == 2);

    Client slow_client(port);
    slow_client.send_partial("{\"protocol\":");
    Client responsive(port);
    CHECK(responsive.request(base_request("responsive", "status"))["ok"] == true);
    connection.command_delay = std::chrono::milliseconds(0);
}

void test_runtime_owns_one_connection_and_releases_port() {
    const auto port = free_port();
    auto connection = std::make_unique<FakeConnection>();
    auto *observed = connection.get();
    observed->set_identity({});
    auto config = test_config();
    config.ipc_port = port;
    config.actuation_enabled = true;
    nomad::runtime::Runtime runtime(std::move(connection), config);
    std::string error;
    CHECK(runtime.start(error));
    wait_until([&runtime] { return runtime.ready(); });
    test_protocol_and_status(port, *observed);
    test_protocol_errors(port, *observed);
    { Client startup(port); read_authority(startup); }
    CHECK(Client(port).request(servo_request("startup-denied", 1500))["error"]["code"] ==
          "not_authoritative");
    admit_authority(port);
    test_command_dispatch_and_dedupe(port, *observed);
    test_vehicle_admission_is_authoritative(port, *observed);
    test_busy_and_slow_client(port, *observed);
    test_gimbal_target_dispatch_and_dedupe(port, *observed);
    test_disconnect_does_not_cancel_or_replay(port, *observed);
    test_evicted_replay_and_wrong_source(port, *observed);
    test_revoke_and_handback(port, *observed);
    test_expired_and_delayed_request(port, *observed);
    test_expired_exact_retry_returns_known_outcome(port, *observed);
    test_reconnect_is_observation_only(port, *observed);
    test_revoke_during_operation(port, *observed);
    CHECK(observed->connect_count == 1);
    runtime.stop();
    CHECK(!runtime.ready());
}

void test_runtime_restart_and_missing_key() {
    const auto port = free_port();
    auto config = test_config();
    config.ipc_port = port;
    config.actuation_enabled = false;
    std::string error;
    {
        nomad::runtime::Runtime runtime(std::make_unique<FakeConnection>(), config);
        CHECK(runtime.start(error));
        Client client(port);
        CHECK(client.request(servo_request("no-key", 1500))["error"]["code"] == "missing_api_key");
        runtime.stop();
    }
    {
        nomad::runtime::Runtime runtime(std::make_unique<FakeConnection>(), config);
        CHECK(runtime.start(error));
        Client restarted(port);
        CHECK(restarted.request(base_request("restart-hello", "hello"))["ok"] == true);
        runtime.stop();
    }
}

void test_restart_rejects_old_request() {
    const auto port = free_port();
    auto config = test_config();
    config.ipc_port = port;
    config.actuation_enabled = true;
    std::string error;
    Json old;
    {
        nomad::runtime::Runtime runtime(std::make_unique<FakeConnection>(), config);
        CHECK(runtime.start(error));
        Client client(port);
        wait_until([&client] {
            return client.request(base_request("restart-ready", "status"))["status"]["vehicle_session"] != 0;
        });
        read_authority(client);
        const auto admitted = client.request(authority_request("restart-admit", "admit_authority"));
        CHECK(admitted["ok"] == true);
        authority.generation = admitted["authority_generation"].get<std::uint64_t>();
        old = servo_request("before-restart", 1500);
        CHECK(client.request(old)["command_result"]["success"] == true);
        runtime.stop();
    }
    auto replacement = std::make_unique<FakeConnection>();
    auto *observed = replacement.get();
    nomad::runtime::Runtime runtime(std::move(replacement), config);
    CHECK(runtime.start(error));
    Client client(port);
    CHECK(client.request(old)["error"]["code"] == "stale_authority");
    CHECK(observed->command_count() == 0);
    runtime.stop();
}

void test_competing_admission() {
    const auto port = free_port();
    auto config = test_config();
    config.ipc_port = port;
    config.actuation_enabled = true;
    auto connection = std::make_unique<FakeConnection>();
    auto *observed = connection.get();
    nomad::runtime::Runtime runtime(std::move(connection), config);
    std::string error;
    CHECK(runtime.start(error));
    Client observer(port);
    wait_until([&observer] {
        return observer.request(base_request("claim-ready", "status"))["status"]["vehicle_session"] != 0;
    });
    read_authority(observer);
    auto claim = [port](const std::string &source) {
        Client client(port);
        return client.request(authority_request("claim-" + source, "admit_authority", source));
    };
    auto first = std::async(std::launch::async, claim, "client-a");
    auto second = std::async(std::launch::async, claim, "client-b");
    const auto a = first.get();
    const auto b = second.get();
    CHECK(a["ok"].get<bool>() != b["ok"].get<bool>());
    const auto winner = a["ok"].get<bool>() ? "client-a" : "client-b";
    const auto loser = a["ok"].get<bool>() ? "client-b" : "client-a";
    authority.generation = (a["ok"].get<bool>() ? a : b)["authority_generation"].get<std::uint64_t>();
    Client rejected(port);
    CHECK(rejected.request(servo_request("losing-command", 1500, loser))["error"]["code"] ==
          "not_authoritative");
    CHECK(observed->command_count() == 0);
    CHECK(rejected.request(servo_request("winning-command", 1500, winner))["command_result"]["success"] == true);
    CHECK(observed->command_count() == 1);
    runtime.stop();
    const auto records = read_journal(config.audit_directory);
    CHECK(count_event(records, "authority_admission") == 1);
    CHECK(count_event(records, "mutation_intent") == 1);
    std::uint64_t ordinal = 0;
    for (const auto &record : records) {
        CHECK(record["ordinal"] == ++ordinal);
    }
}

} // namespace

void run_runtime_scenarios() {
    using nomad::test::run_scenario;
    run_scenario("socket_failure_classification", test_socket_failure_classification);
    run_scenario("backend_actuator_authorization", test_backend_actuator_authorization_and_raw_boundary);
    run_scenario("backend_pulse_recovery", test_backend_pulse_failure_and_explicit_recovery);
    run_scenario("backend_actuator_configuration_uncertainty", test_backend_configuration_persistence_uncertainty);
    run_scenario("backend_pending_neutral_and_safe_interrupt", test_backend_pending_neutral_and_safe_interrupt);
    run_scenario("backend_configuration_authority_recovery", test_configuration_requires_current_authority_recovery);
    run_scenario("backend_configuration_revocation_after_save", test_configuration_revocation_after_native_save);
    run_scenario("backend_configuration_audit_failure_after_save", [] {
        test_configuration_audit_failure_after_native_save();
    });
    run_scenario("backend_configuration_sync_and_audit_failure", [] {
        test_configuration_audit_failure_after_native_save(true);
    });
    run_scenario("backend_configuration_arming_after_save", test_configuration_arming_after_native_save);
    run_scenario("backend_hid_direction_release", test_hid_bidirectional_release_preserves_confirmations_and_stops);
    run_scenario("idle_observer_authority_phase", test_idle_observer_during_independent_authority_phase);
    run_scenario("response_timeout_without_resend", test_response_timeout_does_not_resend_mutation);
    run_scenario("one_connection_and_released_port", test_runtime_owns_one_connection_and_releases_port);
    run_scenario("runtime_restart_and_missing_key", test_runtime_restart_and_missing_key);
    run_scenario("restart_rejects_old_request", test_restart_rejects_old_request);
    run_scenario("service_stop_closes_admission", test_service_stop_closes_admission);
    run_scenario("shutdown_fences_queued_command", test_shutdown_fences_queued_command);
    run_scenario("shutdown_audit_failure", [] { test_shutdown_audit_failure(false); });
    run_scenario("shutdown_previously_failed_audit", [] { test_shutdown_audit_failure(true); });
    run_scenario("shutdown_drain_audit_failure", test_shutdown_drain_audit_failure);
    run_scenario("competing_admission", test_competing_admission);
    run_scenario("client_authentication", test_client_authentication);
    run_scenario("journal_order_and_outcomes", test_journal_order_and_outcomes);
    run_scenario("journal_failure_before_send", [] { test_journal_failure(false); });
    run_scenario("journal_failure_after_send", [] { test_journal_failure(true); });
    run_scenario("journal_startup_and_recovery", test_journal_startup_and_recovery);
    run_scenario("credential_configuration", test_credential_configuration);
    run_scenario("damaged_history", test_damaged_history);
    run_scenario("native_file_failure_and_lock", test_native_file_failure_and_lock);
    run_scenario("authenticated_authority_events", test_authenticated_authority_events);
    run_scenario("rejection_audit_failure", [] { test_rejection_audit_failure(false); });
    run_scenario("authentication_rejection_audit_failure", [] { test_rejection_audit_failure(true); });
    run_scenario("ack_without_admission_evidence", [] { test_acknowledgement_without_admission_evidence(false); });
    run_scenario("ack_without_evidence_audit_failure", [] { test_acknowledgement_without_admission_evidence(true); });
    run_scenario("session_rollover_revokes_at_admission", test_session_rollover_revokes_at_admission);
    run_scenario("all_mutation_outcomes", test_all_mutation_outcomes);
    run_scenario("authority_interruption_after_delivery", test_authority_interruption_after_delivery);
    run_scenario("in_progress_audit_failure", test_in_progress_audit_failure);
    run_scenario("execution_exception_before_send", [] { test_execution_exception(false); });
    run_scenario("execution_exception_after_send", [] { test_execution_exception(true); });
    run_scenario("admission_cancellation_without_send", test_admission_cancellation_without_send);
    run_scenario("definite_rejection_outcomes", test_definite_rejection_outcomes);
}

int main(int argc, char **argv) {
    return nomad::test::run_tests([argc, argv] {
        if (argc == 2 && std::string_view(argv[1]) == "--actuator-tests") {
            test_backend_actuator_authorization_and_raw_boundary();
            test_backend_pulse_failure_and_explicit_recovery();
            test_backend_configuration_persistence_uncertainty();
            test_backend_pending_neutral_and_safe_interrupt();
            test_configuration_requires_current_authority_recovery();
            test_configuration_revocation_after_native_save();
            test_configuration_audit_failure_after_native_save();
            test_configuration_audit_failure_after_native_save(true);
            test_configuration_arming_after_native_save();
            test_hid_bidirectional_release_preserves_confirmations_and_stops();
            return;
        }
        if (argc == 2 && std::string_view(argv[1]) == "--authority-events-stress") {
            for (int iteration = 1; iteration <= 50; ++iteration) {
                nomad::test::run_scenario("authority_events_iteration_" + std::to_string(iteration),
                                         test_authenticated_authority_events);
            }
            return;
        }
        CHECK(argc == 1);
        run_runtime_scenarios();
    });
}
