// SPDX-License-Identifier: Apache-2.0
#pragma once

Json land_request(const std::string &id) {
    return authority_request(id, "land");
}

class LandProgressConnection final : public CopterLandConnection {
  public:
    std::optional<nomad::mavlink::CommandAck> send_command(const nomad::mavlink::Command &command,
        std::uint64_t expected_session, std::chrono::milliseconds timeout) override {
        const auto result = CopterLandConnection::send_command(command, expected_session, timeout);
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        ack_ready = true;
        while (!release_ack && std::chrono::steady_clock::now() < deadline) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        return release_ack ? result : std::nullopt;
    }

    nomad::telemetry::VehicleState get_state() const override {
        auto observed = CopterLandConnection::get_state();
        if (!release_observation) {
            observed.custom_mode = 4;
        }
        return observed;
    }

    std::atomic_bool ack_ready{false};
    std::atomic_bool release_ack{false};
    std::atomic_bool release_observation{false};
};

void test_runtime_land_request_progress() {
    auto config = test_config();
    config.ipc_port = free_port();
    config.actuation_enabled = true;
    auto connection = std::make_unique<LandProgressConnection>();
    auto *observed = connection.get();
    nomad::runtime::Runtime runtime(std::move(connection), config);
    start_ready(runtime, config.ipc_port);
    Client observer(config.ipc_port);
    const auto initial = observer.request(base_request("initial-flight-request", "status"))["status"];
    CHECK(initial.contains("last_flight_request"));
    CHECK(initial["last_flight_request"].is_null());
    admit_authority(config.ipc_port);
    Client pending(config.ipc_port, std::chrono::seconds(5));
    const auto request = land_request("land-request-progress");
    const auto progress = [&] {
        return observer.request(base_request("flight-request-progress", "status"))["status"]["last_flight_request"];
    };

    pending.send(request);
    wait_until([&] { return observed->ack_ready.load(); });
    const auto sending = progress();
    CHECK(sending["request_id"] == request["id"]);
    CHECK(sending["operation"] == "land");
    CHECK(sending["vehicle_session"] == request["vehicle_session"]);
    CHECK(sending["authority_generation"] == request["authority_generation"]);
    CHECK(sending["age_ms"].is_number_integer());
    CHECK(sending["age_ms"] >= 0);
    CHECK(sending["phase"] == "sending");
    CHECK(sending["acknowledged"] == false);
    CHECK(sending["outcome"].is_null());
    observed->release_ack = true;
    Json observing;
    wait_until([&] { observing = progress(); return observing["phase"] == "observing"; });
    CHECK(observing["acknowledged"] == true);
    CHECK(observing["outcome"].is_null());
    CHECK(observed->command_count() == 1);
    observed->release_observation = true;
    const auto response = pending.receive();

    const auto completed = progress();
    CHECK(response["outcome"] == "success");
    CHECK(completed["phase"] == "completed");
    CHECK(completed["acknowledged"] == true);
    CHECK(completed["outcome"] == response["outcome"]);
    CHECK(completed["age_ms"] >= sending["age_ms"]);
    CHECK(observer.request(request) == response);
    const auto replayed = progress();
    CHECK(replayed["request_id"] == completed["request_id"]);
    CHECK(replayed["outcome"] == completed["outcome"]);
    CHECK(replayed["age_ms"] >= completed["age_ms"]);
    CHECK(observed->command_count() == 1);
    CHECK(runtime.stop());
}

void test_runtime_land_request_cache_lifetime() {
    auto config = test_config();
    config.ipc_port = free_port();
    config.actuation_enabled = true;
    auto connection = std::make_unique<CopterLandConnection>();
    auto *observed = connection.get();
    nomad::runtime::Runtime runtime(std::move(connection), config);
    start_ready(runtime, config.ipc_port);
    admit_authority(config.ipc_port);
    Client client(config.ipc_port);
    CHECK(client.request(land_request("land-cache-lifetime"))["outcome"] == "success");
    CHECK(client.request(base_request("completed-flight-request", "status"))
              ["status"]["last_flight_request"]["phase"] == "completed");

    for (int index = 0; index < 257; ++index) {
        CHECK(client.request(servo_request("flight-cache-fill-" + std::to_string(index), 1500))
                  ["outcome"] == "success");
    }

    CHECK(client.request(base_request("evicted-flight-request", "status"))
              ["status"]["last_flight_request"].is_null());
    CHECK(client.request(servo_request("land-cache-lifetime", 1500))["outcome"] == "success");
    CHECK(client.request(base_request("reused-flight-id", "status"))
              ["status"]["last_flight_request"].is_null());
    CHECK(observed->command_count() == 259);
    CHECK(runtime.stop());
}

class LandAfterSafeConnection final : public CopterLandConnection {
  public:
    std::optional<nomad::mavlink::CommandAck> send_command(
        const nomad::mavlink::Command &command, std::chrono::milliseconds timeout) override {
        const auto before = command_count();
        const auto result = FakeConnection::send_command(command, timeout);
        if (command.id == 183 && command_count() > before) {
            safe_sent = true;
        }
        return result;
    }

    nomad::telemetry::VehicleState get_state() const override {
        auto observed = CopterLandConnection::get_state();
        if (land_sent && (!safe_sent || !allow_land_observation)) {
            observed.custom_mode = 4;
        }
        return observed;
    }

    std::atomic_bool safe_sent{false};
    std::atomic_bool allow_land_observation{false};
};

void test_runtime_safe_output_during_land_observation() {
    auto config = test_config();
    config.ipc_port = free_port();
    config.actuation_enabled = true;
    config.actuators = {runtime_actuator(nomad::runtime::ActuatorBehavior::ServoToggle)};
    auto connection = std::make_unique<LandAfterSafeConnection>();
    auto *observed = connection.get();
    nomad::runtime::Runtime runtime(std::move(connection), config);
    start_ready(runtime, config.ipc_port);
    admit_authority(config.ipc_port);
    Client pending(config.ipc_port, std::chrono::seconds(5));
    pending.send(land_request("land-needs-safe-observation"));
    wait_until([&] { return observed->land_sent.load(); });
    Client safe_client(config.ipc_port, std::chrono::seconds(5));

    const auto safe = safe_client.request(actuator_request("safe-before-land-observed", "safe"));
    Client observer(config.ipc_port);
    CHECK(observer.request(land_request("land-competing-operation"))["error"]["code"] == "busy");
    CHECK(observer.request(actuator_request("land-competing-activation", "activate"))["error"]["code"] == "busy");
    for (const auto *metadata : {"neutral", "safe"}) {
        auto configure = authority_request(std::string("land-competing-config-") + metadata, "configure_actuators");
        configure["actuator_configs"] = Json::array();
        configure["operation"] = metadata;
        CHECK(observer.request(configure)["error"]["code"] == "busy");
    }
    observed->allow_land_observation = true;
    const auto land = pending.receive();

    CHECK(safe["outcome"] == "success");
    CHECK(safe["command_result"]["acknowledged"] == true);
    CHECK(observed->safe_sent.load());
    CHECK(land["outcome"] == "success");
    CHECK(land["command_result"]["acknowledged"] == true);
    CHECK(observed->command_count() == 2);
    check_audited_outcome(config.audit_directory, safe);
    check_audited_outcome(config.audit_directory, land);
    CHECK(observer.request(land_request("land-explicit-new-operation"))["outcome"] == "success");
    CHECK(observed->command_count() == 3);
    CHECK(runtime.stop());
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
        const auto progress = client.request(base_request("land-final-request-state", "status"))
            ["status"]["last_flight_request"];
        CHECK(progress["phase"] == "completed");
        CHECK(progress["acknowledged"] == true);
        CHECK(progress["outcome"] == expected);
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
    std::atomic_bool safe_intent_seen{false};
    auto config = test_config();
    config.ipc_port = free_port();
    config.actuation_enabled = true;
    config.actuators = {runtime_actuator(nomad::runtime::ActuatorBehavior::ServoToggle)};
    config.audit_write_guard = [&](const std::string &line) {
        const auto record = Json::parse(line);
        if (record.value("event", "") == "mutation_intent" &&
            record.value("request_id", "") == "land-expired-safe-output") {
            safe_intent_seen = true;
            const auto expiry = record.at("expires_at_ms").get<std::int64_t>();
            std::this_thread::sleep_until(
                std::chrono::system_clock::time_point(std::chrono::milliseconds(expiry + 1)));
        }
        return true;
    };
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

    CHECK(safe_intent_seen.load());
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
    const auto progress = client.request(base_request("land-audit-request-state", "status"))
        ["status"]["last_flight_request"];
    if (after_send) {
        CHECK(progress["phase"] == "completed");
        CHECK(progress["acknowledged"] == true);
        CHECK(progress["outcome"] == "unknown");
    } else {
        CHECK(progress.is_null());
    }
    CHECK(!runtime.stop());
}
