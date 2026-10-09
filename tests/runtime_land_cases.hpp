// SPDX-License-Identifier: Apache-2.0
#pragma once

Json land_request(const std::string &id) {
    return authority_request(id, "land");
}

void test_runtime_land_engagement_and_replay() {
    auto config = test_config();
    config.ipc_port = free_port();
    config.actuation_enabled = true;
    auto connection = std::make_unique<CopterLandConnection>();
    auto *observed = connection.get();
    nomad::runtime::Runtime runtime(std::move(connection), config);
    start_ready(runtime, config.ipc_port);
    admit_authority(config.ipc_port);
    Client client(config.ipc_port, std::chrono::seconds(5));
    auto request = land_request("land-engagement");
    request["source"] = "test-client";
    request["client_metadata"] = {{"operator_note", "additive fields remain compatible"}};

    const auto response = client.request(request);

    CHECK(response["outcome"] == "success");
    CHECK(response["command_result"]["acknowledged"] == true);
    CHECK(response["command_result"]["message"] == "LAND mode observed; touchdown not verified");
    CHECK(observed->last_command.id == 21);
    CHECK(observed->command_count() == 1);
    check_audited_outcome(config.audit_directory, response);
    CHECK(client.request(request) == response);
    auto stale = request;
    stale["id"] = "land-stale-sequence";
    CHECK(client.request(stale)["error"]["code"] == "stale_request");
    CHECK(observed->command_count() == 1);
    CHECK(runtime.stop());
}

void test_runtime_land_outcomes() {
    for (const auto *expected : {"failed", "unknown", "interrupted"}) {
        auto config = test_config();
        config.ipc_port = free_port();
        config.actuation_enabled = true;
        auto connection = std::make_unique<CopterLandConnection>();
        auto *observed = connection.get();
        if (std::string_view(expected) == "failed") {
            observed->acknowledgement = nomad::mavlink::CommandAck{21, 2};
        } else if (std::string_view(expected) == "unknown") {
            observed->observation = CopterLandConnection::Observation::PreAckLand;
        } else {
            observed->observation = CopterLandConnection::Observation::ChangedIdentity;
        }
        nomad::runtime::Runtime runtime(std::move(connection), config);
        start_ready(runtime, config.ipc_port);
        admit_authority(config.ipc_port);
        Client client(config.ipc_port, std::chrono::seconds(5));
        const auto request = land_request(std::string("land-") + expected);
        const auto response = client.request(request);
        CHECK(response["outcome"] == expected);
        CHECK(response["command_result"]["success"] == false);
        CHECK(response["command_result"]["acknowledged"] == true);
        check_audited_outcome(config.audit_directory, response);
        CHECK(client.request(request) == response);
        CHECK(observed->command_count() == 1);
        CHECK(runtime.stop());
    }
}

void test_runtime_land_validation() {
    auto config = test_config();
    config.ipc_port = free_port();
    config.actuation_enabled = true;
    auto connection = std::make_unique<CopterLandConnection>();
    auto *observed = connection.get();
    nomad::runtime::Runtime runtime(std::move(connection), config);
    start_ready(runtime, config.ipc_port);
    admit_authority(config.ipc_port);
    Client client(config.ipc_port);
    auto unauthenticated = land_request("land-unauthenticated");
    unauthenticated.erase("credential");
    CHECK(client.request(unauthenticated)["error"]["code"] == "authentication_failed");
    auto invalid = land_request("land-invalid");
    invalid["altitude_m"] = 10;
    CHECK(client.request(invalid)["error"]["code"] == "invalid_request");
    auto expired = land_request("land-expired");
    expired["expires_at_ms"] = 1;
    CHECK(client.request(expired)["error"]["code"] == "expired_request");
    observed->set_identity(nomad::telemetry::identify_vehicle(3, 1));
    const auto rejected = client.request(land_request("land-plane"));
    CHECK(rejected["outcome"] == "rejected");
    CHECK(rejected["command_result"]["acknowledged"] == false);
    CHECK(observed->command_count() == 0);
    CHECK(runtime.stop());
}

void test_runtime_land_revoke() {
    auto config = test_config();
    config.ipc_port = free_port();
    config.actuation_enabled = true;
    auto connection = std::make_unique<CopterLandConnection>();
    auto *observed = connection.get();
    observed->observation = CopterLandConnection::Observation::OtherMode;
    nomad::runtime::Runtime runtime(std::move(connection), config);
    start_ready(runtime, config.ipc_port);
    admit_authority(config.ipc_port);
    Client pending(config.ipc_port, std::chrono::seconds(5));
    const auto request = land_request("land-revoke-pending");
    pending.send(request);
    wait_until([&] { return observed->land_sent.load(); });
    Client operator_client(config.ipc_port);
    const auto started = std::chrono::steady_clock::now();
    CHECK(operator_client.request(authority_request("land-revoke", "revoke_authority", "operator"))["ok"] == true);
    const auto response = pending.receive();
    CHECK(response["outcome"] == "interrupted");
    CHECK(response["command_result"]["acknowledged"] == true);
    CHECK(std::chrono::steady_clock::now() - started < std::chrono::milliseconds(400));
    CHECK(observed->command_count() == 1);
    check_audited_outcome(config.audit_directory, response);
    CHECK(runtime.stop());
}

void test_runtime_land_safe_output_budget() {
    auto config = test_config();
    config.ipc_port = free_port();
    config.actuation_enabled = true;
    config.actuators = {runtime_actuator(nomad::runtime::ActuatorBehavior::ServoToggle)};
    auto connection = std::make_unique<CopterLandConnection>();
    auto *observed = connection.get();
    observed->observation = CopterLandConnection::Observation::PreAckLand;
    nomad::runtime::Runtime runtime(std::move(connection), config);
    start_ready(runtime, config.ipc_port);
    admit_authority(config.ipc_port);
    Client pending(config.ipc_port, std::chrono::seconds(5));
    const auto request = land_request("land-safe-pending");
    pending.send(request);
    wait_until([&] { return observed->land_sent.load(); });
    Client observer(config.ipc_port);
    const auto observer_started = std::chrono::steady_clock::now();
    CHECK(observer.request(base_request("land-live-status", "status"))["ok"] == true);
    CHECK(std::chrono::steady_clock::now() - observer_started < std::chrono::milliseconds(400));
    CHECK(observer.request(request)["error"]["code"] == "request_in_progress");
    Client safe_client(config.ipc_port, std::chrono::seconds(5));
    const auto safe_started = std::chrono::steady_clock::now();
    const auto safe = safe_client.request(actuator_request("land-safe-output", "safe"));
    CHECK(safe["outcome"] == "success");
    CHECK(safe["command_result"]["success"] == true);
    CHECK(std::chrono::steady_clock::now() - safe_started < std::chrono::milliseconds(3400));
    const auto response = pending.receive();
    CHECK(response["outcome"] == "unknown");
    CHECK(response["command_result"]["acknowledged"] == true);
    CHECK(observed->command_count() == 2);
    CHECK(observed->last_command.id == 183);
    check_audited_outcome(config.audit_directory, response);
    Client replay_client(config.ipc_port);
    CHECK(replay_client.request(request) == response);
    CHECK(observed->command_count() == 2);
    CHECK(runtime.stop());
}

void test_runtime_land_expired_safe_output_is_not_sent() {
    auto config = test_config();
    config.ipc_port = free_port();
    config.actuation_enabled = true;
    config.actuators = {runtime_actuator(nomad::runtime::ActuatorBehavior::ServoToggle)};
    auto connection = std::make_unique<CopterLandConnection>();
    auto *observed = connection.get();
    observed->observation = CopterLandConnection::Observation::PreAckLand;
    nomad::runtime::Runtime runtime(std::move(connection), config);
    start_ready(runtime, config.ipc_port);
    admit_authority(config.ipc_port);
    Client pending(config.ipc_port, std::chrono::seconds(5));
    pending.send(land_request("land-expired-safe-pending"));
    wait_until([&] { return observed->land_sent.load(); });
    Client safe_client(config.ipc_port, std::chrono::seconds(5));
    auto safe_request = actuator_request("land-expired-safe-output", "safe");
    safe_request["expires_at_ms"] = nomad::runtime::detail::unix_milliseconds() + 200;

    const auto safe = safe_client.request(safe_request);

    CHECK(safe["outcome"] == "rejected");
    CHECK(safe["error"]["code"] == "expired_request");
    CHECK(pending.receive()["outcome"] == "unknown");
    CHECK(observed->command_count() == 1);
    CHECK(observed->last_command.id == 21);
    CHECK(runtime.stop());
}

void test_runtime_land_audit_failure(bool after_send) {
    auto config = test_config();
    config.ipc_port = free_port();
    config.actuation_enabled = true;
    config.audit_write_guard = [after_send](const std::string &line) {
        return Json::parse(line)["event"] != (after_send ? "mutation_outcome" : "mutation_intent");
    };
    auto connection = std::make_unique<CopterLandConnection>();
    auto *observed = connection.get();
    nomad::runtime::Runtime runtime(std::move(connection), config);
    start_ready(runtime, config.ipc_port);
    admit_authority(config.ipc_port);
    Client client(config.ipc_port, std::chrono::seconds(5));
    const auto response = client.request(land_request("land-audit-failure"));
    CHECK(response["error"]["code"] == "audit_failure");
    CHECK(response["outcome"] == (after_send ? "unknown" : "rejected"));
    CHECK(observed->command_count() == (after_send ? 1U : 0U));
    if (after_send) {
        CHECK(response["command_result"]["acknowledged"] == true);
    }
    CHECK(!runtime.stop());
}
