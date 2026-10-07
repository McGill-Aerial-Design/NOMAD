// SPDX-License-Identifier: Apache-2.0
#pragma once

std::string receive_failure(Client &client,
                            std::source_location location = std::source_location::current()) {
    try {
        static_cast<void>(client.receive(location));
    } catch (const std::runtime_error &error) {
        return error.what();
    }
    return {};
}

void test_socket_failure_classification() {
    CHECK(socket_failure_reason(0, 0) == "orderly_peer_close");
    CHECK(socket_failure_reason(7, 0) == "incomplete_transfer");
#ifdef _WIN32
    CHECK(socket_failure_reason(-1, WSAETIMEDOUT) == "timeout");
    CHECK(socket_failure_reason(-1, WSAEWOULDBLOCK) == "timeout");
    CHECK(socket_failure_reason(-1, WSAECONNRESET) == "socket_error");
#else
    CHECK(socket_failure_reason(-1, EAGAIN) == "timeout");
    CHECK(socket_failure_reason(-1, EWOULDBLOCK) == "timeout");
    CHECK(socket_failure_reason(-1, ECONNRESET) == "socket_error");
#endif
    CHECK(safe_request_field(test_credentials.at("test-client")) == "[redacted]");
    CHECK(safe_request_field("line\nbreak") == "line?break");
}

void check_closed_client_context(Client &client) {
    const auto failure = receive_failure(client);
    CHECK(failure.find("orderly_peer_close; code=0; count=0; partial_bytes=0") != std::string::npos);
    CHECK(failure.find("id=[redacted]; type=status") != std::string::npos);
    CHECK(failure.find("sent_at=runtime_client_cases.hpp:") != std::string::npos);
    CHECK(failure.find("observed_at=runtime_client_cases.hpp:") != std::string::npos);
    CHECK(failure.find(test_credentials.at("test-client")) == std::string::npos);
}

void test_idle_observer_during_independent_authority_phase() {
    auto config = test_config();
    config.ipc_port = free_port();
    config.actuation_enabled = true;
    std::promise<void> release;
    const auto released = release.get_future().share();
    std::atomic_bool phase_entered{false};
    config.audit_write_guard = [&](const std::string &line) {
        if (Json::parse(line)["event"] == "authority_revoke_intent") {
            phase_entered = true;
            return released.wait_for(std::chrono::seconds(5)) == std::future_status::ready;
        }
        return true;
    };
    auto connection = std::make_unique<FakeConnection>();
    auto *observed = connection.get();
    nomad::runtime::Runtime runtime(std::move(connection), config);
    start_ready(runtime, config.ipc_port);
    admit_authority(config.ipc_port);
    Client old_observer(config.ipc_port);
    CHECK(old_observer.request(base_request(test_credentials.at("test-client"), "status"))["ok"] == true);
    Client phase(config.ipc_port);
    phase.send(authority_request("independent-phase-revoke", "revoke_authority", "operator"));
    wait_until([&] { return phase_entered.load(); });
    CHECK(old_observer.wait_for_peer_close());
    check_closed_client_context(old_observer);
    release.set_value();
    CHECK(phase.receive()["ok"] == true);
    old_observer.disconnect();
    phase.disconnect();
    Client observer(config.ipc_port);
    const auto status = observer.request(base_request("fresh-observer", "status"))["status"];
    CHECK(status["authority_owner"].is_null());
    CHECK(status["authority_generation"].get<std::uint64_t>() > authority.generation);
    CHECK(observed->command_count() == 0);
    observer.disconnect();
    CHECK(runtime.stop());
    CHECK(count_event(read_journal(config.audit_directory), "authority_revoke") == 1);
}

void test_response_timeout_does_not_resend_mutation() {
    auto config = test_config();
    config.ipc_port = free_port();
    config.actuation_enabled = true;
    auto connection = std::make_unique<CompletionGatedConnection>();
    auto *observed = connection.get();
    std::promise<void> release;
    observed->release = release.get_future().share();
    nomad::runtime::Runtime runtime(std::move(connection), config);
    start_ready(runtime, config.ipc_port);
    admit_authority(config.ipc_port);
    Client client(config.ipc_port);
    client.send(servo_request("diagnostic-timeout", 1500));
    wait_until([&] { return observed->delivered.load(); });
    const auto failure = receive_failure(client);
    release.set_value();
    CHECK(failure.find("receive: timeout; code=") != std::string::npos);
    CHECK(failure.find("partial_bytes=0; id=diagnostic-timeout; type=set_servo") != std::string::npos);
    CHECK(failure.find("sent_at=runtime_client_cases.hpp:") != std::string::npos);
    CHECK(failure.find("observed_at=runtime_client_cases.hpp:") != std::string::npos);
    const auto response = client.receive();
    CHECK(response["outcome"] == "success");
    CHECK(response["command_result"]["acknowledged"] == true);
    CHECK(observed->command_count() == 1);
    check_audited_outcome(config.audit_directory, response);
    client.disconnect();
    CHECK(runtime.stop());
}
