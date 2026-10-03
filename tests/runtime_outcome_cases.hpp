// SPDX-License-Identifier: Apache-2.0
#pragma once

void check_audited_outcome(const std::string &directory, const Json &response) {
    const auto records = read_journal(directory);
    const auto found = std::find_if(records.begin(), records.end(), [&](const Json &record) {
        return record.value("event", "") == "mutation_outcome" &&
               record.value("request_id", "") == response.value("id", "");
    });
    CHECK(found != records.end());
    CHECK((*found)["result"] == response["outcome"]);
    CHECK((*found)["acknowledged"] == response["command_result"]["acknowledged"]);
}

Json output_request(const std::string &type, const std::string &id) {
    auto request = base_request(id, type);
    bind_authority(request);
    if (type == "set_servo") {
        request["channel"] = 8;
        request["pwm_microseconds"] = 1500;
    } else if (type == "set_relay") {
        request["relay_number"] = 0;
        request["on"] = true;
    } else if (type == "motor_test") {
        request["motor_instance"] = 1;
        request["pwm_microseconds"] = 1500;
        request["timeout_seconds"] = 0.1;
    } else if (type == "configure_gimbal") {
        request["mount_mode"] = 2;
    } else {
        request["pitch_deg"] = 10.0;
        request["roll_deg"] = 5.0;
    }
    return request;
}

void test_all_mutation_outcomes() {
    auto config = test_config();
    config.ipc_port = free_port();
    config.actuation_enabled = true;
    auto connection = std::make_unique<FakeConnection>();
    auto *observed = connection.get();
    nomad::runtime::Runtime runtime(std::move(connection), config);
    start_ready(runtime, config.ipc_port);
    admit_authority(config.ipc_port);
    Client client(config.ipc_port);
    for (const auto *type : {"set_servo", "set_relay", "motor_test", "configure_gimbal", "set_gimbal_target"}) {
        for (const auto *outcome : {"success", "failed", "unknown"}) {
            observed->acknowledgement = nomad::mavlink::CommandAck{0, outcome == std::string("failed") ?
                std::uint8_t{2} : std::uint8_t{0}};
            if (outcome == std::string("unknown")) {
                observed->acknowledgement.reset();
            }
            const auto request = output_request(type, std::string(type) + "-" + outcome);
            const auto response = client.request(request);
            CHECK(response["outcome"] == outcome);
            CHECK(response["command_result"]["acknowledged"] == (outcome != std::string("unknown")));
            check_audited_outcome(config.audit_directory, response);
            const auto count = observed->command_count();
            CHECK(client.request(request) == response);
            CHECK(observed->command_count() == count);
        }
    }
    CHECK(observed->command_count() == 15);
    CHECK(runtime.stop());
}

class CompletionGatedConnection : public FakeConnection {
  public:
    std::atomic_bool delivered{false};
    std::shared_future<void> release;

    std::optional<nomad::mavlink::CommandAck> send_command(const nomad::mavlink::Command &command,
                                                         std::chrono::milliseconds timeout) override {
        const auto result = FakeConnection::send_command(command, timeout);
        delivered = true;
        CHECK(release.wait_for(std::chrono::seconds(5)) == std::future_status::ready);
        return result;
    }
};

void check_in_progress_outcome(Client &client, const Json &request, const std::string &directory) {
    const auto duplicate = client.request(request);
    CHECK(duplicate["error"]["code"] == "request_in_progress");
    CHECK(duplicate["outcome"] == "unknown");
    const auto records = read_journal(directory);
    CHECK(records.back()["result"] == duplicate["outcome"]);
    CHECK(records.back()["send_eligible"] == "unknown");
}

void test_authority_interruption_after_delivery() {
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
    const auto request = servo_request("interrupted-after-delivery", 1500);
    client.send(request);
    wait_until([&] { return observed->delivered.load(); });
    Client operator_client(config.ipc_port);
    check_in_progress_outcome(operator_client, request, config.audit_directory);
    CHECK(observed->command_count() == 1);
    const auto revoked = operator_client.request(authority_request("interrupt", "revoke_authority", "operator"));
    CHECK(revoked["ok"] == true);
    CHECK(!revoked.contains("outcome"));
    release.set_value();
    const auto response = client.receive();
    CHECK(response["error"]["code"] == "authority_interrupted");
    CHECK(response["outcome"] == "interrupted");
    CHECK(response["command_result"]["acknowledged"] == true);
    CHECK(observed->command_count() == 1);
    check_audited_outcome(config.audit_directory, response);
    const auto replay = client.request(request);
    CHECK(replay["outcome"] == "rejected");
    CHECK(observed->command_count() == 1);
    CHECK(runtime.stop());
}

void test_in_progress_audit_failure() {
    auto config = test_config();
    config.ipc_port = free_port();
    config.actuation_enabled = true;
    config.audit_write_guard = [](const std::string &line) {
        return Json::parse(line)["event"] != "request_rejected";
    };
    auto connection = std::make_unique<CompletionGatedConnection>();
    auto *observed = connection.get();
    std::promise<void> release;
    observed->release = release.get_future().share();
    nomad::runtime::Runtime runtime(std::move(connection), config);
    start_ready(runtime, config.ipc_port);
    admit_authority(config.ipc_port);
    Client original(config.ipc_port);
    const auto request = servo_request("in-progress-audit-failure", 1500);
    original.send(request);
    wait_until([&] { return observed->delivered.load(); });
    Client duplicate(config.ipc_port);
    const auto response = duplicate.request(request);
    CHECK(response["error"]["code"] == "audit_failure");
    CHECK(response["outcome"] == "unknown");
    auto expired_duplicate = request;
    expired_duplicate["expires_at_ms"] = 1;
    const auto latched = duplicate.request(expired_duplicate);
    CHECK(latched["error"]["code"] == "audit_failure");
    CHECK(latched["outcome"] == "unknown");
    CHECK(observed->command_count() == 1);
    release.set_value();
    const auto completed = original.receive();
    CHECK(completed["outcome"] == "unknown");
    CHECK(duplicate.request(request) == completed);
    CHECK(observed->command_count() == 1);
    CHECK(!runtime.stop());
}

class ExceptionalConnection : public FakeConnection {
  public:
    bool after_send{false};
    std::optional<nomad::mavlink::CommandAck> send_command(const nomad::mavlink::Command &command,
                                                         std::chrono::milliseconds timeout) override {
        if (after_send) {
            static_cast<void>(FakeConnection::send_command(command, timeout));
        }
        throw std::runtime_error("deterministic execution exception");
    }
};

void test_execution_exception(bool after_send) {
    auto config = test_config();
    config.ipc_port = free_port();
    config.actuation_enabled = true;
    auto connection = std::make_unique<ExceptionalConnection>();
    connection->after_send = after_send;
    auto *observed = connection.get();
    nomad::runtime::Runtime runtime(std::move(connection), config);
    start_ready(runtime, config.ipc_port);
    admit_authority(config.ipc_port);
    Client client(config.ipc_port);
    const auto request = servo_request("exception", 1500);
    const auto response = client.request(request);
    CHECK(response["error"]["code"] == "internal_error");
    CHECK(response["outcome"] == (after_send ? "unknown" : "rejected"));
    CHECK(observed->command_count() == (after_send ? 1U : 0U));
    check_audited_outcome(config.audit_directory, response);
    CHECK(client.request(request) == response);
    CHECK(observed->command_count() == (after_send ? 1U : 0U));
    CHECK(runtime.stop());
}

class AdmissionCancelledConnection : public FakeConnection {
  public:
    std::optional<nomad::mavlink::CommandAck> send_command(const nomad::mavlink::Command &command,
                                                         std::chrono::milliseconds) override {
        return nomad::mavlink::CommandAck{command.id, 0,
                                        nomad::mavlink::CommandAck::Status::AdmissionCancelled};
    }
};

void test_admission_cancellation_without_send() {
    auto config = test_config();
    config.ipc_port = free_port();
    config.actuation_enabled = true;
    auto connection = std::make_unique<AdmissionCancelledConnection>();
    auto *observed = connection.get();
    nomad::runtime::Runtime runtime(std::move(connection), config);
    start_ready(runtime, config.ipc_port);
    admit_authority(config.ipc_port);
    Client client(config.ipc_port);
    const auto response = client.request(servo_request("admission-cancelled", 1500));
    CHECK(response["outcome"] == "rejected");
    CHECK(response["command_result"]["acknowledged"] == false);
    CHECK(observed->command_count() == 0);
    check_audited_outcome(config.audit_directory, response);
    CHECK(runtime.stop());
}

void test_definite_rejection_outcomes() {
    auto config = test_config();
    config.ipc_port = free_port();
    config.actuation_enabled = true;
    auto connection = std::make_unique<FakeConnection>();
    auto *observed = connection.get();
    nomad::runtime::Runtime runtime(std::move(connection), config);
    start_ready(runtime, config.ipc_port);
    admit_authority(config.ipc_port);
    Client client(config.ipc_port);
    auto stale = servo_request("outcome-stale", 1500);
    stale["authority_generation"] = 0;
    auto expired = servo_request("outcome-expired", 1500);
    expired["expires_at_ms"] = 1;
    auto invalid = servo_request("outcome-invalid", 1500);
    invalid.erase("channel");
    auto unauthenticated = servo_request("outcome-auth", 1500);
    unauthenticated.erase("credential");
    for (const auto &request : {stale, expired, invalid, unauthenticated}) {
        CHECK(client.request(request)["outcome"] == "rejected");
    }
    CHECK(observed->command_count() == 0);
    const auto original = servo_request("outcome-replay-original", 1500);
    CHECK(client.request(original)["outcome"] == "success");
    auto replay = original;
    replay["id"] = "outcome-stale-sequence";
    const auto denied = client.request(replay);
    CHECK(denied["error"]["code"] == "stale_request");
    CHECK(denied["outcome"] == "rejected");
    CHECK(observed->command_count() == 1);
    const auto records = read_journal(config.audit_directory);
    CHECK(records.back()["result"] == denied["outcome"]);
    CHECK(runtime.stop());
}
