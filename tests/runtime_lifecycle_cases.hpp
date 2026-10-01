// SPDX-License-Identifier: Apache-2.0
#pragma once

void test_service_stop_closes_admission() {
    const auto port = free_port();
    auto config = test_config();
    config.ipc_port = port;
    config.actuation_enabled = true;
    auto connection = std::make_unique<FakeConnection>();
    auto *observed = connection.get();
    nomad::runtime::Runtime runtime(std::move(connection), config);
    std::string error;
    CHECK(runtime.start(error));
    Client client(port);
    wait_until([&] {
        return client.request(base_request("service-ready", "status"))["status"]["vehicle_session"] != 0;
    });
    admit_authority(port);
#ifdef _WIN32
    nomad::runtime::process::test_begin_service();
    CHECK(nomad::runtime::process::test_service_state() == SERVICE_START_PENDING);
    nomad::runtime::process::publish_ready(runtime);
    CHECK(nomad::runtime::process::test_service_state() == SERVICE_RUNNING);
    nomad::runtime::process::test_stop_service();
    auto repeated_stop = std::async(std::launch::async, nomad::runtime::process::test_stop_service);
    auto shutdown = std::async(std::launch::async, nomad::runtime::process::test_shutdown_service);
    repeated_stop.get();
    shutdown.get();
    CHECK(nomad::runtime::process::test_service_state() == SERVICE_STOP_PENDING);
    CHECK(nomad::runtime::process::stop_requested());
#else
    runtime.request_stop();
#endif
    CHECK(!client.request(servo_request("service-stop-old-command", 1500))["ok"].get<bool>());
    const auto status = client.request(base_request("service-stopping", "status"))["status"];
    CHECK(status["lifecycle"] == "stopping");
    CHECK(status["authority_owner"].is_null());
    CHECK(observed->command_count() == 0);
#ifdef _WIN32
    nomad::runtime::process::publish_stopping();
#endif
    runtime.stop();
    CHECK(count_event(read_journal(config.audit_directory), "runtime_shutdown") == 1);
#ifdef _WIN32
    nomad::runtime::process::test_finish_service(0);
    CHECK(nomad::runtime::process::test_service_state() == SERVICE_STOPPED);
    CHECK(nomad::runtime::process::test_service_error() == 0);
    nomad::runtime::process::test_begin_service();
    nomad::runtime::process::test_stop_service();
    nomad::runtime::process::publish_ready(runtime);
    CHECK(nomad::runtime::process::test_service_state() == SERVICE_STOP_PENDING);
    nomad::runtime::process::publish_stopping();
    nomad::runtime::process::test_finish_service(78);
    CHECK(nomad::runtime::process::test_service_error() == 78);
#endif
}

void test_shutdown_fences_queued_command() {
    auto config = test_config();
    config.ipc_port = free_port();
    config.actuation_enabled = true;
    auto connection = std::make_unique<FakeConnection>();
    auto *observed = connection.get();
    observed->command_delay = std::chrono::milliseconds(500);
    nomad::runtime::Runtime runtime(std::move(connection), config);
    start_ready(runtime, config.ipc_port);
    admit_authority(config.ipc_port);
    const auto command = servo_request("queued-at-stop", 1500);
    auto pending = std::async(std::launch::async, [&] {
        Client client(config.ipc_port);
        return client.request(command);
    });
    wait_until([&] { return observed->command_started.load(); });
    runtime.request_stop();
    CHECK(pending.get()["error"]["code"] == "authority_interrupted");
    CHECK(observed->command_count() == 0);
    CHECK(runtime.stop());
    const auto records = read_journal(config.audit_directory);
    CHECK(count_event(records, "mutation_intent") == 1);
    CHECK(count_event(records, "mutation_outcome") == 1);
    CHECK(records.back()["event"] == "runtime_shutdown");
}

void test_shutdown_audit_failure(bool previously_failed) {
    auto config = test_config();
    config.ipc_port = free_port();
    config.actuation_enabled = true;
    config.audit_write_guard = [previously_failed](const std::string &line) {
        return Json::parse(line)["event"] != (previously_failed ? "mutation_intent" : "runtime_shutdown");
    };
    nomad::runtime::Runtime runtime(std::make_unique<FakeConnection>(), config);
    start_ready(runtime, config.ipc_port);
    if (previously_failed) {
        admit_authority(config.ipc_port);
        Client client(config.ipc_port);
        CHECK(client.request(servo_request("fail-audit-before-stop", 1500))["error"]["code"] == "audit_failure");
    }
    CHECK(!runtime.stop());
    CHECK(!runtime.stop());
    CHECK(count_event(read_journal(config.audit_directory), "runtime_shutdown") == 0);
}

void test_shutdown_drain_audit_failure() {
    auto config = test_config();
    config.ipc_port = free_port();
    config.actuation_enabled = true;
    std::atomic<bool> outcome_failed{false};
    config.audit_write_guard = [&](const std::string &line) {
        if (Json::parse(line)["event"] == "mutation_outcome") {
            outcome_failed = true;
            return false;
        }
        return true;
    };
    auto connection = std::make_unique<FakeConnection>();
    auto *observed = connection.get();
    observed->command_delay = std::chrono::milliseconds(500);
    nomad::runtime::Runtime runtime(std::move(connection), config);
    start_ready(runtime, config.ipc_port);
    admit_authority(config.ipc_port);
    const auto command = servo_request("audit-fails-during-drain", 1500);
    auto pending = std::async(std::launch::async, [&] {
        Client client(config.ipc_port);
        return client.request(command);
    });
    wait_until([&] { return observed->command_started.load(); });
    const int exit_code = nomad::runtime::process::shutdown_runtime(runtime);
    CHECK(exit_code == 74);
    CHECK(!pending.get()["ok"].get<bool>());
    CHECK(outcome_failed.load());
    CHECK(observed->command_count() == 0);
    CHECK(!runtime.stop());
    CHECK(count_event(read_journal(config.audit_directory), "runtime_shutdown") == 0);
#ifdef _WIN32
    nomad::runtime::process::test_finish_service(exit_code);
    CHECK(nomad::runtime::process::test_service_error() == 74);
#endif
}
