// SPDX-License-Identifier: Apache-2.0
#pragma once

std::vector<Json> read_journal(const std::string &directory) {
    std::vector<Json> records;
    for (const auto &entry : std::filesystem::directory_iterator(directory)) {
        if (entry.path().extension() != ".jsonl") {
            continue;
        }
        std::ifstream file(entry.path());
        CHECK(file.is_open());
        std::string line;
        while (std::getline(file, line)) {
            const auto record = Json::parse(line);
            CHECK(record["schema"] == 1);
            CHECK(record.contains("runtime_incarnation"));
            for (const auto &[identity, token] : test_credentials) {
                CHECK(line.find(token) == std::string::npos);
            }
            records.push_back(record);
        }
    }
    return records;
}

std::size_t count_event(const std::vector<Json> &records, const std::string &event) {
    return std::count_if(records.begin(), records.end(), [&](const Json &record) { return record["event"] == event; });
}

void start_ready(nomad::runtime::Runtime &runtime, std::uint16_t port) {
    std::string error;
    CHECK(runtime.start(error));
    Client observer(port);
    wait_until([&observer] {
        return observer.request(base_request("ready-security", "status"))["status"]["vehicle_session"] != 0;
    });
}

void check_authentication_faults(Client &client, const Json &valid) {
    std::vector<Json> forged(7, valid);
    forged[0].erase("credential");
    forged[1]["credential"] = std::string(64, '0');
    forged[2]["client_id"] = "unknown";
    forged[3]["client_id"] = "client-b";
    forged[4]["command_source"] = "client-b";
    forged[5]["source"] = "client-b";
    forged[6]["credential"] = "malformed";
    for (const auto &request : forged) {
        const auto denied = client.request(request);
        CHECK(denied["error"]["code"] == "authentication_failed");
        CHECK(denied.dump().find(test_credentials.at("test-client")) == std::string::npos);
    }
}

void check_tampered_authentication_payload(Client &client, const Json &valid) {
    auto tampered = valid;
    tampered.erase("credential");
    const auto payload = tampered.dump();
    tampered["auth_payload"] = payload;
    tampered["auth_proof"] = nomad::runtime::detail::make_proof(
        test_credentials.at("test-client"), "nomad-core:request:v1:" + payload);
    tampered["pwm_microseconds"] = 1900;
    client.send_raw(tampered.dump());
    CHECK(client.receive()["error"]["code"] == "authentication_failed");
}

void check_distinct_client_cache_ownership(Client &client, FakeConnection &observed) {
    auto other = servo_request("auth-command", 1700, "client-b");
    CHECK(client.request(other)["error"]["code"] == "not_authoritative");
    CHECK(observed.command_count() == 1);
    CHECK(client.request(authority_request("distinct-revoke", "revoke_authority", "client-b"))["ok"] == true);
    read_authority(client);
    const auto handback = client.request(authority_request("distinct-handback", "handback_authority", "client-b"));
    CHECK(handback["ok"] == true);
    authority.generation = handback["authority_generation"].get<std::uint64_t>();
    CHECK(client.request(servo_request("auth-command", 1700, "client-b"))["outcome"] == "success");
    CHECK(observed.command_count() == 2);
}

void test_client_authentication() {
    auto config = test_config();
    config.ipc_port = free_port();
    config.actuation_enabled = true;
    auto connection = std::make_unique<FakeConnection>();
    auto *observed = connection.get();
    nomad::runtime::Runtime runtime(std::move(connection), config);
    start_ready(runtime, config.ipc_port);
    Client client(config.ipc_port);
    read_authority(client);
    auto claim = authority_request("unauthenticated-admission", "admit_authority");
    claim.erase("credential");
    CHECK(client.request(claim)["error"]["code"] == "authentication_failed");
    admit_authority(config.ipc_port);
    const auto valid = servo_request("auth-command", 1500);
    check_authentication_faults(client, valid);
    CHECK(observed->command_count() == 0);
    check_tampered_authentication_payload(client, valid);
    CHECK(client.request(valid)["command_result"]["success"] == true);
    CHECK(observed->command_count() == 1);
    auto public_read = base_request("public-status", "status");
    public_read.erase("credential");
    CHECK(client.request(public_read)["ok"] == true);
    check_distinct_client_cache_ownership(client, *observed);
    runtime.stop();
    CHECK(count_event(read_journal(config.audit_directory), "request_rejected") == 10);
    CHECK(count_event(read_journal(config.audit_directory), "mutation_intent") == 2);
}

class IntentObservingConnection : public FakeConnection {
  public:
    explicit IntentObservingConnection(std::string directory) : directory_(std::move(directory)) {}
    std::optional<nomad::mavlink::CommandAck> send_command(const nomad::mavlink::Command &command,
                                                         std::chrono::milliseconds timeout) override {
        CHECK(count_event(read_journal(directory_), "mutation_intent") > command_count());
        return FakeConnection::send_command(command, timeout);
    }
  private:
    std::string directory_;
};

void test_journal_order_and_outcomes() {
    auto config = test_config();
    config.ipc_port = free_port();
    config.actuation_enabled = true;
    auto connection = std::make_unique<IntentObservingConnection>(config.audit_directory);
    auto *observed = connection.get();
    nomad::runtime::Runtime runtime(std::move(connection), config);
    start_ready(runtime, config.ipc_port);
    admit_authority(config.ipc_port);
    Client client(config.ipc_port);
    const auto successful = servo_request("audited-success", 1500);
    CHECK(client.request(successful)["outcome"] == "success");
    CHECK(client.request(successful)["outcome"] == "success");
    observed->acknowledgement = nomad::mavlink::CommandAck{183, 2};
    CHECK(client.request(servo_request("audited-failure", 1600))["outcome"] == "failed");
    observed->acknowledgement.reset();
    CHECK(client.request(servo_request("audited-no-ack", 1600))["outcome"] == "unknown");
    observed->acknowledgement = nomad::mavlink::CommandAck{183, 0};
    CHECK(client.request(servo_request("audited-invalid", 10))["outcome"] == "rejected");
    auto secret_id = servo_request(test_credentials.at("test-client"), 1500);
    CHECK(client.request(secret_id).dump().find(test_credentials.at("test-client")) == std::string::npos);
    const auto records_before = read_journal(config.audit_directory);
    const auto reads_before = records_before.size();
    CHECK(client.request(base_request("quiet-read", "status"))["ok"] == true);
    CHECK(read_journal(config.audit_directory).size() == reads_before);
    runtime.stop();
    const auto records = read_journal(config.audit_directory);
    CHECK(count_event(records, "mutation_intent") == 5);
    CHECK(count_event(records, "mutation_outcome") == 5);
    CHECK(count_event(records, "runtime_shutdown") == 1);
    CHECK(observed->command_count() == 4);
}

void test_journal_failure(bool after_send) {
    auto config = test_config();
    config.ipc_port = free_port();
    config.actuation_enabled = true;
    config.audit_write_guard = [after_send](const std::string &line) {
        const auto event = Json::parse(line)["event"];
        return event != (after_send ? "mutation_outcome" : "mutation_intent");
    };
    auto connection = std::make_unique<FakeConnection>();
    auto *observed = connection.get();
    nomad::runtime::Runtime runtime(std::move(connection), config);
    start_ready(runtime, config.ipc_port);
    admit_authority(config.ipc_port);
    Client client(config.ipc_port);
    const auto command = servo_request("audit-write-fault", 1500);
    const auto failed = client.request(command);
    CHECK(failed["error"]["code"] == "audit_failure");
    CHECK(failed["outcome"] == (after_send ? "unknown" : "rejected"));
    if (after_send) {
        CHECK(failed["command_result"]["acknowledged"] == true);
        CHECK(failed["command_result"]["success"] == true);
    }
    CHECK(observed->command_count() == (after_send ? 1U : 0U));
    CHECK(client.request(servo_request("audit-write-inhibited", 1700))["error"]["code"] == "audit_failure");
    CHECK(client.request(command) == failed);
    CHECK(client.request(base_request("audit-health", "status"))["status"]["audit_healthy"] == false);
    CHECK(observed->command_count() == (after_send ? 1U : 0U));
    runtime.stop();
    CHECK(count_event(read_journal(config.audit_directory), "mutation_intent") == (after_send ? 1U : 0U));
}

void test_journal_startup_and_recovery() {
    auto config = test_config();
    config.ipc_port = free_port();
    std::string error;
    {
        auto invalid = config;
        invalid.client_credentials.clear();
        nomad::runtime::Runtime runtime(std::make_unique<FakeConnection>(), invalid);
        CHECK(!runtime.start(error));
    }
    {
        nomad::runtime::Runtime runtime(std::make_unique<FakeConnection>(), config);
        CHECK(runtime.start(error));
        runtime.stop();
    }
    {
        nomad::runtime::Runtime runtime(std::make_unique<FakeConnection>(), config);
        CHECK(runtime.start(error));
        runtime.stop();
    }
    CHECK(count_event(read_journal(config.audit_directory), "runtime_start") == 2);
    for (const auto &entry : std::filesystem::directory_iterator(config.audit_directory)) {
        if (entry.path().extension() == ".jsonl") {
            std::ofstream damaged(entry.path(), std::ios::app);
            damaged << "{partial";
            break;
        }
    }
    nomad::runtime::Runtime refused(std::make_unique<FakeConnection>(), config);
    CHECK(!refused.start(error));
    CHECK(error.find("audit") != std::string::npos);
}
