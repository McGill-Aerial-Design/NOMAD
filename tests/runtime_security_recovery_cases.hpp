// SPDX-License-Identifier: Apache-2.0
#pragma once

void test_credential_configuration() {
    using nomad::runtime::detail::FileMode;
    using nomad::runtime::detail::ProtectedFile;
    using nomad::runtime::detail::load_credentials;
    using nomad::runtime::detail::valid_credentials;
    CHECK(valid_credentials(test_credentials));
    for (const auto &credentials : std::vector<std::map<std::string, std::string>>{
             {}, {{"bad identity", std::string(64, 'a')}}, {{"a", "short"}},
             {{"a", std::string(64, 'A')}}, {{"a", std::string(64, 'a')}, {"b", std::string(64, 'a')}}}) {
        CHECK(!valid_credentials(credentials));
    }
    unsigned index = 0;
    for (const auto &content : {Json(test_credentials).dump(), std::string("{broken"),
                              std::string("{\"a\":1}"), std::string("{\"a\":\"x\",\"a\":\"y\"}")}) {
        const auto path = (storage.path / ("credentials-" + std::to_string(index) + ".json")).string();
        ProtectedFile file;
        CHECK(file.open(path, FileMode::Create));
        CHECK(file.append(content));
        file.close();
        std::map<std::string, std::string> loaded;
        CHECK(load_credentials(path, loaded) == (index == 0));
        ++index;
    }
}

void test_damaged_history() {
    using nomad::runtime::detail::FileMode;
    using nomad::runtime::detail::ProtectedFile;
    for (const auto &content : {std::string(), std::string("not-json\n"), std::string("{}\n")}) {
        auto config = test_config();
        config.ipc_port = free_port();
        CHECK(nomad::runtime::detail::prepare_audit_directory(config.audit_directory));
        ProtectedFile damaged;
        CHECK(damaged.open(config.audit_directory + "/previous.jsonl", FileMode::Create));
        CHECK(damaged.append(content));
        damaged.close();
        nomad::runtime::Runtime runtime(std::make_unique<FakeConnection>(), config);
        std::string error;
        CHECK(!runtime.start(error));
        CHECK(error.find("audit") != std::string::npos);
        CHECK(std::filesystem::file_size(config.audit_directory + "/previous.jsonl") == content.size());
    }
}

void test_native_file_failure_and_lock() {
    using nomad::runtime::detail::FileMode;
    using nomad::runtime::detail::ProtectedFile;
    const auto path = (storage.path / "read-only-native.json").string();
    ProtectedFile file;
    CHECK(!file.append("not-open"));
    CHECK(file.open(path, FileMode::Create));
    CHECK(file.append("test"));
    file.close();
    CHECK(file.open(path, FileMode::Read));
    CHECK(!file.append("cannot-write-through-read-handle"));
    auto config = test_config();
    config.ipc_port = free_port();
    nomad::runtime::Runtime first(std::make_unique<FakeConnection>(), config);
    std::string error;
    CHECK(first.start(error));
    config.ipc_port = free_port();
    nomad::runtime::Runtime second(std::make_unique<FakeConnection>(), config);
    CHECK(!second.start(error));
    CHECK(error.find("audit") != std::string::npos);
    first.stop();
}

void test_authenticated_authority_events() {
    auto config = test_config();
    config.ipc_port = free_port();
    config.actuation_enabled = true;
    auto connection = std::make_unique<FakeConnection>();
    auto *observed = connection.get();
    nomad::runtime::Runtime runtime(std::move(connection), config);
    start_ready(runtime, config.ipc_port);
    admit_authority(config.ipc_port);
    Client client(config.ipc_port);
    for (const auto *type : {"revoke_authority", "handback_authority"}) {
        auto forged = authority_request(type, type);
        forged.erase("credential");
        CHECK(client.request(forged)["error"]["code"] == "authentication_failed");
    }
    test_revoke_and_handback(config.ipc_port, *observed);
    test_revoke_during_operation(config.ipc_port, *observed);
    read_authority(client);
    const auto restored = client.request(authority_request("before-session-loss", "handback_authority"));
    CHECK(restored["ok"] == true);
    authority.generation = restored["authority_generation"].get<std::uint64_t>();
    test_reconnect_is_observation_only(config.ipc_port, *observed);
    runtime.stop();
    const auto records = read_journal(config.audit_directory);
    CHECK(count_event(records, "authority_admission") == 1);
    CHECK(count_event(records, "authority_revoke") == 2);
    CHECK(count_event(records, "authority_handback") == 3);
    CHECK(count_event(records, "session_authority_loss") >= 1);
    CHECK(std::any_of(records.begin(), records.end(), [](const Json &record) {
        return record["event"] == "mutation_outcome" && record["result"] == "interrupted";
    }));
    std::uint64_t ordinal = 0;
    for (const auto &record : records) {
        CHECK(record["ordinal"] == ++ordinal);
        if (record["event"] == "vehicle_session" || record["event"] == "session_authority_loss") {
            const auto previous = record["authority_generation"].get<std::uint64_t>();
            CHECK(record["resulting_generation"] == previous +
                (record["event"] == "session_authority_loss" ? 1 : 0));
        }
    }
}

void test_rejection_audit_failure(bool authentication) {
    auto config = test_config();
    config.ipc_port = free_port();
    config.actuation_enabled = true;
    config.audit_write_guard = [](const std::string &line) {
        return Json::parse(line)["event"] != "request_rejected";
    };
    auto connection = std::make_unique<FakeConnection>();
    auto *observed = connection.get();
    nomad::runtime::Runtime runtime(std::move(connection), config);
    start_ready(runtime, config.ipc_port);
    admit_authority(config.ipc_port);
    Client client(config.ipc_port);
    auto rejected = servo_request("rejection-write-failure", 1500);
    if (authentication) {
        rejected.erase("credential");
    } else {
        rejected["authority_generation"] = 0;
    }
    CHECK(client.request(rejected)["error"]["code"] == "audit_failure");
    CHECK(client.request(servo_request("after-rejection-write", 1500))["error"]["code"] == "audit_failure");
    CHECK(observed->command_count() == 0);
    CHECK(client.request(base_request("rejected-health", "status"))["status"]["audit_healthy"] == false);
    runtime.stop();
}
