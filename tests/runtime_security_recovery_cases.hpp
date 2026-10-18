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
    {
        Client client(config.ipc_port);
        for (const auto *type : {"revoke_authority", "handback_authority"}) {
            auto forged = authority_request(type, type);
            forged.erase("credential");
            CHECK(client.request(forged)["error"]["code"] == "authentication_failed");
        }
    }
    test_revoke_and_handback(config.ipc_port, *observed);
    test_revoke_during_operation(config.ipc_port, *observed);
    // Independent client phases can outlast the IPC idle deadline; start a new observation connection.
    Client client(config.ipc_port);
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

void test_authority_expiry_after_durable_intent(bool handback) {
    auto config = test_config();
    config.ipc_port = free_port();
    config.actuation_enabled = true;
    std::atomic<bool> delay_intent{false}, intent_observed{false}, expired_after_wait{false};
    std::atomic<std::int64_t> expires_at{0};
    config.audit_write_guard = [&](const std::string &line) {
        if (Json::parse(line)["event"] == "authority_intent" && delay_intent.exchange(false)) {
            intent_observed = true;
            const auto remaining = expires_at.load() - nomad::runtime::detail::unix_milliseconds();
            std::this_thread::sleep_for(std::chrono::milliseconds((std::max<std::int64_t>)(0, remaining) + 20));
            expired_after_wait = nomad::runtime::detail::unix_milliseconds() > expires_at.load();
        }
        return true;
    };
    auto connection = std::make_unique<FakeConnection>();
    auto *observed = connection.get();
    nomad::runtime::Runtime runtime(std::move(connection), config);
    start_ready(runtime, config.ipc_port);
    // This fault deliberately spans request validity; observe its terminal rejection.
    Client client(config.ipc_port, std::chrono::seconds(8));
    if (handback) {
        admit_authority(config.ipc_port);
        const auto revoked = client.request(authority_request("expiry-prepare-revoke", "revoke_authority", "operator"));
        CHECK(revoked["ok"] == true);
    }
    read_authority(client);
    const auto generation_before = authority.generation;
    auto request = authority_request("expires-in-intent", handback ? "handback_authority" : "admit_authority");
    expires_at = request["expires_at_ms"].get<std::int64_t>();
    delay_intent = true;
    const auto response = client.request(request);
    CHECK(intent_observed && expired_after_wait);
    CHECK(response["ok"] == false);
    CHECK(response["error"]["code"] == "stale_authority");
    const auto status = client.request(base_request("after-expired-intent", "status"))["status"];
    CHECK(status["authority_owner"].is_null());
    CHECK(status["authority_generation"] == generation_before);
    CHECK(status["audit_healthy"] == true);
    CHECK(observed->command_count() == 0);
    const auto records = read_journal(config.audit_directory);
    CHECK(count_event(records, "authority_intent") == (handback ? 2U : 1U));
    CHECK(count_event(records, handback ? "authority_handback" : "authority_admission") == 0);
    CHECK(count_event(records, "request_rejected") == 1);
    CHECK(runtime.stop());
}

void run_authority_expiry_scenarios() {
    nomad::test::run_scenario("admission_expiry_after_intent", [] {
        test_authority_expiry_after_durable_intent(false);
    });
    nomad::test::run_scenario("handback_expiry_after_intent", [] {
        test_authority_expiry_after_durable_intent(true);
    });
}

enum class AuthorityIntentState { Unchanged, Session, Disconnected, StaleHeartbeat };

void change_authority_intent_state(FakeConnection &connection, AuthorityIntentState change) {
    if (change == AuthorityIntentState::Session) {
        connection.advance_session_before_monitor();
        return;
    }
    if (change == AuthorityIntentState::Disconnected) {
        connection.set_connected(false);
        return;
    }
    if (change == AuthorityIntentState::StaleHeartbeat) {
        auto state = connection.get_state();
        state.heartbeat_fresh = false;
        state.heartbeat_updated_at -= std::chrono::seconds(10);
        connection.set_state(state);
    }
}

void test_authority_state_after_durable_intent(bool handback, AuthorityIntentState change) {
    auto config = test_config();
    config.ipc_port = free_port();
    config.actuation_enabled = true;
    auto connection = std::make_unique<FakeConnection>();
    auto *observed = connection.get();
    std::atomic_bool armed{false}, intent_seen{false};
    config.audit_write_guard = [&](const std::string &line) {
        if (Json::parse(line)["event"] == "authority_intent" && armed.exchange(false)) {
            intent_seen = true;
            change_authority_intent_state(*observed, change);
        }
        return true;
    };
    nomad::runtime::Runtime runtime(std::move(connection), config);
    start_ready(runtime, config.ipc_port);
    Client client(config.ipc_port);
    if (handback) {
        admit_authority(config.ipc_port);
        CHECK(client.request(authority_request("state-prepare-revoke", "revoke_authority", "operator"))["ok"] == true);
    }
    read_authority(client);
    const auto generation_before = authority.generation;
    armed = true;
    const auto response = client.request(authority_request(
        "state-during-intent", handback ? "handback_authority" : "admit_authority"));
    CHECK(intent_seen);
    const bool unchanged = change == AuthorityIntentState::Unchanged;
    CHECK(response["ok"] == unchanged);
    if (!unchanged) {
        CHECK(response["error"]["code"] == "stale_authority");
    }
    const auto status = client.request(base_request("after-state-intent", "status"))["status"];
    CHECK(unchanged ? status["authority_owner"] == "test-client" : status["authority_owner"].is_null());
    CHECK(status["authority_generation"] == generation_before + (unchanged ? 1 : 0));
    CHECK(status["audit_healthy"] == true);
    CHECK(observed->command_count() == 0);
    const auto records = read_journal(config.audit_directory);
    CHECK(count_event(records, "authority_intent") == (handback ? 2U : 1U));
    CHECK(count_event(records, handback ? "authority_handback" : "authority_admission") == (unchanged ? 1U : 0U));
    CHECK(count_event(records, "request_rejected") == (unchanged ? 0U : 1U));
    CHECK(runtime.stop());
}

class AuthorityIntentConnection final : public FakeConnection {
  public:
    void after_intent(std::function<void()> callback) {
        std::lock_guard lock(state_mutex);
        writer_ = std::this_thread::get_id();
        after_intent_ = std::move(callback);
    }

    nomad::telemetry::VehicleState get_state() const override {
        std::function<void()> callback;
        {
            std::lock_guard lock(state_mutex);
            if (std::this_thread::get_id() == writer_) {
                callback = std::exchange(after_intent_, {});
            }
        }
        if (callback) {
            callback();
        }
        return FakeConnection::get_state();
    }

  private:
    std::thread::id writer_;
    mutable std::function<void()> after_intent_;
};

void test_authority_rechecks_generation_after_state_refresh(bool handback, bool compete) {
    auto config = test_config();
    config.ipc_port = free_port();
    config.actuation_enabled = true;
    auto connection = std::make_unique<AuthorityIntentConnection>();
    auto *observed = connection.get();
    std::atomic_bool armed{false}, interleaved{false};
    Json concurrent;
    config.audit_write_guard = [&](const std::string &line) {
        if (Json::parse(line)["event"] == "authority_intent" && armed.exchange(false)) {
            observed->after_intent([&] {
                Client competitor(config.ipc_port);
                concurrent = competitor.request(authority_request("interleaved-authority",
                    compete ? (handback ? "handback_authority" : "admit_authority") : "revoke_authority",
                    compete ? "client-b" : "operator"));
                interleaved = true;
            });
        }
        return true;
    };
    nomad::runtime::Runtime runtime(std::move(connection), config);
    start_ready(runtime, config.ipc_port);
    Client client(config.ipc_port);
    if (handback) {
        admit_authority(config.ipc_port);
        CHECK(client.request(authority_request("race-prepare-revoke", "revoke_authority", "operator"))["ok"] == true);
    }
    read_authority(client);
    const auto generation_before = authority.generation;
    armed = true;
    const auto response = client.request(authority_request(
        "original-authority", handback ? "handback_authority" : "admit_authority"));
    CHECK(interleaved);
    CHECK(concurrent["ok"] == true);
    CHECK(response["ok"] == false);
    CHECK(response["error"]["code"] == "stale_authority");
    const auto status = client.request(base_request("after-authority-race", "status"))["status"];
    CHECK(compete ? status["authority_owner"] == "client-b" : status["authority_owner"].is_null());
    CHECK(status["authority_generation"] == generation_before + 1);
    CHECK(status["audit_healthy"] == true);
    CHECK(observed->command_count() == 0);
    const auto records = read_journal(config.audit_directory);
    CHECK(count_event(records, handback ? "authority_handback" : "authority_admission") == (compete ? 1U : 0U));
    CHECK(count_event(records, "request_rejected") == 1);
    CHECK(runtime.stop());
}

void run_authority_state_scenarios() {
    struct Case {
        AuthorityIntentState state;
        const char *admission;
        const char *handback;
    };
    const Case cases[]{
        {AuthorityIntentState::Unchanged, "admission_unchanged_after_intent", "handback_unchanged_after_intent"},
        {AuthorityIntentState::Session, "admission_session_after_intent", "handback_session_after_intent"},
        {AuthorityIntentState::Disconnected,
         "admission_disconnected_after_intent", "handback_disconnected_after_intent"},
        {AuthorityIntentState::StaleHeartbeat,
         "admission_stale_heartbeat_after_intent", "handback_stale_heartbeat_after_intent"},
    };
    for (const auto &test : cases) {
        nomad::test::run_scenario(test.admission, [&] {
            test_authority_state_after_durable_intent(false, test.state);
        });
        nomad::test::run_scenario(test.handback, [&] {
            test_authority_state_after_durable_intent(true, test.state);
        });
    }
    nomad::test::run_scenario("admission_revoke_during_refresh", [] {
        test_authority_rechecks_generation_after_state_refresh(false, false);
    });
    nomad::test::run_scenario("handback_revoke_during_refresh", [] {
        test_authority_rechecks_generation_after_state_refresh(true, false);
    });
    nomad::test::run_scenario("admission_competitor_during_refresh", [] {
        test_authority_rechecks_generation_after_state_refresh(false, true);
    });
    nomad::test::run_scenario("handback_competitor_during_refresh", [] {
        test_authority_rechecks_generation_after_state_refresh(true, true);
    });
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

class AcknowledgementOnlyConnection : public FakeConnection {
  public:
    std::optional<nomad::mavlink::CommandAck> send_command(const nomad::mavlink::Command &command,
                                                         std::chrono::milliseconds) override {
        // Deliberately incomplete test transport: ACK evidence must outrank an unobserved admission callback.
        return nomad::mavlink::CommandAck{command.id, 2};
    }
};

void test_acknowledgement_without_admission_evidence(bool audit_failure) {
    auto config = test_config();
    config.ipc_port = free_port();
    config.actuation_enabled = true;
    if (audit_failure) {
        config.audit_write_guard = [](const std::string &line) {
            return Json::parse(line)["event"] != "mutation_outcome";
        };
    }
    nomad::runtime::Runtime runtime(std::make_unique<AcknowledgementOnlyConnection>(), config);
    start_ready(runtime, config.ipc_port);
    admit_authority(config.ipc_port);
    Client client(config.ipc_port);
    const auto response = client.request(servo_request("ack-stronger-than-callback", 1500));
    CHECK(response["outcome"] == (audit_failure ? "unknown" : "failed"));
    if (!audit_failure) {
        const auto records = read_journal(config.audit_directory);
        CHECK(records.back()["event"] == "mutation_outcome");
        CHECK(records.back()["send_eligible"] == true);
        CHECK(records.back()["result"] == "failed");
        CHECK(records.back()["admission_checked"] == false);
    }
    runtime.stop();
}
