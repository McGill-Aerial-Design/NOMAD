// SPDX-License-Identifier: Apache-2.0
#pragma once

nomad::runtime::ActuatorDefinition runtime_actuator(nomad::runtime::ActuatorBehavior behavior) {
    nomad::runtime::ActuatorDefinition p;
    p.id = "configured-output";
    p.name = "Operator configured name";
    p.behavior = behavior;
    p.channel = behavior == nomad::runtime::ActuatorBehavior::RelayPulse ? 2 : 8;
    p.confirmation_count = 2;
    p.pulse_ms = 50;
    if (behavior == nomad::runtime::ActuatorBehavior::ServoBidirectional) {
        p.safe_pwm = p.pwm_neutral;
    }
    return p;
}

Json actuator_request(std::string id, std::string operation, std::string source = "ui", int slot = -1) {
    auto request = authority_request(std::move(id), "actuator_action");
    request["actuator_id"] = "configured-output";
    request["operation"] = std::move(operation);
    request["input_source"] = source;
    if (source == "hid") {
        request["input_slot"] = slot;
    }
    request["expires_at_ms"] = nomad::runtime::detail::unix_milliseconds() + 5000;
    return request;
}

void handback_actuator_authority(std::uint16_t port) {
    Client client(port);
    read_authority(client);
    const auto response = client.request(authority_request("actuator-authority-handback", "handback_authority"));
    CHECK(response["ok"] == true);
    authority.generation = response["authority_generation"].get<std::uint64_t>();
}

void test_configuration_requires_current_authority_recovery() {
    auto config = test_config();
    config.ipc_port = free_port();
    config.actuation_enabled = true;
    config.actuator_config_file = (std::filesystem::path(config.audit_directory) / "actuators.json").string();
    config.actuators = {runtime_actuator(nomad::runtime::ActuatorBehavior::ServoToggle)};
    auto connection = std::make_unique<FakeConnection>();
    nomad::runtime::Runtime runtime(std::move(connection), config);
    start_ready(runtime, config.ipc_port);
    admit_authority(config.ipc_port);
    Client client(config.ipc_port);
    CHECK(client.request(actuator_request("old-owner-safe", "safe"))["command_result"]["success"] == true);
    CHECK(client.request(authority_request("configuration-owner-revoke", "revoke_authority", "operator"))["ok"] ==
          true);
    handback_actuator_authority(config.ipc_port);
    auto request = authority_request("configuration-new-owner", "configure_actuators");
    request["actuator_configs"] = nomad::runtime::detail::serialize_actuator_definitions(config.actuators);
    CHECK(client.request(request)["error"]["code"] == "actuator_configuration_blocked");
    CHECK(!std::filesystem::exists(config.actuator_config_file));
    CHECK(client.request(actuator_request("new-owner-safe", "safe"))["command_result"]["success"] == true);
    request = authority_request("configuration-after-new-owner-safe", "configure_actuators");
    request["actuator_configs"] = nomad::runtime::detail::serialize_actuator_definitions(config.actuators);
    const auto saved = client.request(request);
    CHECK(saved["outcome"] == "success");
    CHECK(saved["request_result"]["success"] == true);
    CHECK(saved["command_result"]["success"] == false);
    std::vector<nomad::runtime::ActuatorDefinition> disk;
    std::string error;
    CHECK(nomad::runtime::detail::load_actuator_configuration(config.actuator_config_file, disk, error));
    CHECK(disk[0].id == config.actuators[0].id);
    runtime.stop();
}

void test_configuration_revocation_after_native_save() {
    auto config = test_config();
    config.ipc_port = free_port();
    config.actuation_enabled = true;
    config.actuator_config_file = (std::filesystem::path(config.audit_directory) / "actuators.json").string();
    config.actuators = {runtime_actuator(nomad::runtime::ActuatorBehavior::ServoToggle)};
    config.actuator_directory_sync_guard = [&] {
        Client operator_client(config.ipc_port);
        CHECK(operator_client.request(authority_request("revoke-during-native-save", "revoke_authority",
                                                      "operator"))["ok"] ==
              true);
        return true;
    };
    auto connection = std::make_unique<FakeConnection>();
    auto *observed = connection.get();
    nomad::runtime::Runtime runtime(std::move(connection), config);
    start_ready(runtime, config.ipc_port);
    admit_authority(config.ipc_port);
    Client client(config.ipc_port);
    CHECK(client.request(actuator_request("revocation-config-safe", "safe"))["command_result"]["success"] == true);
    auto changed = config.actuators;
    changed[0].name = "Persisted but not applied";
    auto request = authority_request("revoked-configuration", "configure_actuators");
    request["actuator_configs"] = nomad::runtime::detail::serialize_actuator_definitions(changed);
    const auto interrupted = client.request(request);
    CHECK(interrupted["outcome"] == "interrupted");
    CHECK(interrupted["configuration_changed"] == true);
    CHECK(interrupted["configuration_recovery_required"] == true);
    std::vector<nomad::runtime::ActuatorDefinition> disk;
    std::string error;
    CHECK(nomad::runtime::detail::load_actuator_configuration(config.actuator_config_file, disk, error));
    CHECK(disk[0].name == "Persisted but not applied");
    CHECK(client.request(base_request("revoked-config-discovery", "get_actuators"))["actuators"][0]["name"] ==
          "Operator configured name");
    handback_actuator_authority(config.ipc_port);
    CHECK(client.request(actuator_request("revoked-config-activation", "activate"))["error"]["code"] ==
          "configuration_recovery_required");
    CHECK(observed->command_count() == 1);
    runtime.stop();
}

void test_configuration_audit_failure_after_native_save(bool reject_sync = false) {
    auto config = test_config();
    config.ipc_port = free_port();
    config.actuation_enabled = true;
    config.actuator_config_file = (std::filesystem::path(config.audit_directory) / "actuators.json").string();
    config.actuators = {runtime_actuator(nomad::runtime::ActuatorBehavior::ServoToggle)};
    config.actuator_directory_sync_guard = [reject_sync] { return !reject_sync; };
    config.audit_write_guard = [](const std::string &record) {
        const auto event = Json::parse(record);
        return event.value("operation", "") != "configure_actuators" || event.value("event", "") != "mutation_outcome";
    };
    auto connection = std::make_unique<FakeConnection>();
    auto *observed = connection.get();
    nomad::runtime::Runtime runtime(std::move(connection), config);
    start_ready(runtime, config.ipc_port);
    admit_authority(config.ipc_port);
    Client client(config.ipc_port);
    CHECK(client.request(actuator_request("audit-config-safe", "safe"))["command_result"]["success"] == true);
    auto changed = config.actuators;
    changed[0].name = "Native file changed before audit failure";
    auto request = authority_request("configuration-audit-failure", "configure_actuators");
    request["actuator_configs"] = nomad::runtime::detail::serialize_actuator_definitions(changed);
    const auto failed = client.request(request);
    CHECK(failed["outcome"] == "unknown");
    CHECK(failed["error"]["code"] == "audit_failure");
    CHECK(failed["configuration_changed"] == !reject_sync);
    CHECK(failed["configuration_may_have_changed"] == reject_sync);
    CHECK(failed["configuration_recovery_required"] == true);
    CHECK(!failed.contains("command_result"));
    CHECK(observed->command_count() == 1);
    std::vector<nomad::runtime::ActuatorDefinition> disk;
    std::string error;
    CHECK(nomad::runtime::detail::load_actuator_configuration(config.actuator_config_file, disk, error));
    CHECK(disk[0].name == changed[0].name);
    CHECK(client.request(base_request("failed-audit-config-discovery", "get_actuators"))
          ["configuration_recovery_required"] == true);
    runtime.stop();
}

class ArmingDuringConfigurationConnection : public FakeConnection {
  public:
    void observe_external_arm() {
        std::lock_guard lock(state_mutex);
        state->armed = true;
    }
};

void test_configuration_arming_after_native_save() {
    auto config = test_config();
    config.ipc_port = free_port();
    config.actuation_enabled = true;
    config.actuator_config_file = (std::filesystem::path(config.audit_directory) / "actuators.json").string();
    config.actuators = {runtime_actuator(nomad::runtime::ActuatorBehavior::ServoToggle)};
    auto connection = std::make_unique<ArmingDuringConfigurationConnection>();
    auto *observed = connection.get();
    config.actuator_directory_sync_guard = [observed] {
        observed->observe_external_arm();
        return true;
    };
    nomad::runtime::Runtime runtime(std::move(connection), config);
    start_ready(runtime, config.ipc_port);
    admit_authority(config.ipc_port);
    Client client(config.ipc_port);
    CHECK(client.request(actuator_request("arming-config-safe", "safe"))["command_result"]["success"] == true);
    auto changed = config.actuators;
    changed[0].name = "Definition saved before external arming";
    auto request = authority_request("configuration-arming", "configure_actuators");
    request["actuator_configs"] = nomad::runtime::detail::serialize_actuator_definitions(changed);
    const auto response = client.request(request);
    CHECK(response["outcome"] == "interrupted");
    CHECK(response["configuration_changed"] == true);
    CHECK(response["configuration_recovery_required"] == true);
    const auto status = client.request(base_request("arming-config-discovery", "get_actuators"));
    CHECK(status["actuators"][0]["name"] == config.actuators[0].name);
    CHECK(observed->command_count() == 1);
    std::vector<nomad::runtime::ActuatorDefinition> disk;
    std::string error;
    CHECK(nomad::runtime::detail::load_actuator_configuration(config.actuator_config_file, disk, error));
    CHECK(disk[0].name == changed[0].name);
    runtime.stop();
}

void test_catalog_revision_orders_persistence_and_empty_replacement() {
    auto config = test_config();
    config.ipc_port = free_port();
    config.actuation_enabled = true;
    config.actuator_config_file = (std::filesystem::path(config.audit_directory) / "actuators.json").string();
    auto p = runtime_actuator(nomad::runtime::ActuatorBehavior::ServoPosition);
    p.hazardous = false;
    p.confirmation_count = 0;
    config.actuators = {p};
    Json during_save;
    config.actuator_directory_sync_guard = [&] {
        Client observer(config.ipc_port);
        during_save = observer.request(base_request("catalog-during-persistence", "get_actuators"));
        return true;
    };
    auto connection = std::make_unique<FakeConnection>();
    auto *observed = connection.get();
    nomad::runtime::Runtime runtime(std::move(connection), config);
    start_ready(runtime, config.ipc_port);
    admit_authority(config.ipc_port);
    Client client(config.ipc_port);
    const auto before = client.request(base_request("catalog-before-persistence", "get_actuators"));
    CHECK(client.request(actuator_request("catalog-config-safe", "safe"))["command_result"]["success"] == true);
    auto request = authority_request("empty-catalog-configuration", "configure_actuators");
    request["actuator_configs"] = Json::array();
    const auto saved = client.request(request);
    CHECK(saved["request_result"]["success"] == true);
    CHECK(during_save["actuators"].size() == 1);
    CHECK(during_save["actuator_configuration_revision"] == before["actuator_configuration_revision"]);
    CHECK(saved["actuators"].empty());
    CHECK(saved["actuator_configuration_revision"] > during_save["actuator_configuration_revision"]);
    const auto after = client.request(base_request("catalog-after-persistence", "get_actuators"));
    CHECK(after["actuators"].empty());
    CHECK(after["actuator_configuration_revision"] == saved["actuator_configuration_revision"]);
    CHECK(observed->command_count() == 1);
    runtime.stop();
}

void test_continuous_axis_eligibility_is_owned_by_backend() {
    for (const int confirmations : {0, 1, 2, 3}) {
        auto config = test_config();
        config.ipc_port = free_port();
        config.actuation_enabled = true;
        auto p = runtime_actuator(nomad::runtime::ActuatorBehavior::ServoPosition);
        p.confirmation_count = confirmations;
        p.hazardous = confirmations >= 2;
        config.actuators = {p};
        auto connection = std::make_unique<FakeConnection>();
        auto *observed = connection.get();
        nomad::runtime::Runtime runtime(std::move(connection), config);
        start_ready(runtime, config.ipc_port);
        admit_authority(config.ipc_port);
        Client client(config.ipc_port);
        const auto discovery = client.request(base_request("axis-discovery", "get_actuators"));
        const auto action = discovery["actuators"][0]["actions"][0];
        CHECK(action["continuous_axis_allowed"] == (confirmations == 0));
        CHECK(action["continuous_axis_blocked_reason"].get<std::string>().empty() == (confirmations == 0));
        CHECK(client.request(actuator_request("axis-safe", "safe"))["command_result"]["success"] == true);
        auto axis = actuator_request("axis-position", "position", "hid", 6);
        axis["value"] = 0.25;
        const auto result = client.request(axis);
        CHECK(observed->command_count() == (confirmations == 0 ? 2 : 1));
        if (confirmations > 0) {
            CHECK(result["error"]["code"] == "actuator_action_blocked");
            CHECK(result["error"]["message"] == action["continuous_axis_blocked_reason"]);
            CHECK(observed->command_count() == 1);
        }
        const int ui_requests = confirmations == 0 ? 1 : confirmations;
        for (int count = 0; count < ui_requests; ++count) {
            auto ui = actuator_request("axis-ui-confirm-" + std::to_string(count), "position");
            ui["value"] = 0.25;
            CHECK(client.request(ui)["execution_attempted"] == (count == ui_requests - 1));
        }
        CHECK(observed->command_count() == (confirmations == 0 ? 3 : 2));
        runtime.stop();
    }
}

void test_unknown_actuator_actions_are_rejected_without_state_change() {
    auto config = test_config();
    config.ipc_port = free_port();
    config.actuation_enabled = true;
    config.actuators = {runtime_actuator(nomad::runtime::ActuatorBehavior::ServoToggle)};
    auto connection = std::make_unique<FakeConnection>();
    auto *observed = connection.get();
    nomad::runtime::Runtime runtime(std::move(connection), config);
    start_ready(runtime, config.ipc_port);
    admit_authority(config.ipc_port);
    Client client(config.ipc_port);
    const auto before = client.request(base_request("before-unknown", "get_actuators"))["actuators"];
    for (const auto *operation : {"safe", "stop", "activate"}) {
        auto request = actuator_request(std::string("unknown-") + operation, operation);
        request["actuator_id"] = "stale-output";
        const auto rejected = client.request(request);
        CHECK(rejected["error"]["code"] == "actuator_action_blocked");
        CHECK(rejected["outcome"] == "rejected");
        CHECK(observed->command_count() == 0);
    }
    CHECK(client.request(base_request("after-unknown", "get_actuators"))["actuators"] == before);
    CHECK(client.request(actuator_request("known-safe-after-unknown", "safe"))["command_result"]["success"] == true);
    CHECK(observed->command_count() == 1);
    runtime.stop();
}

void test_backend_actuator_authorization_and_raw_boundary() {
    auto config = test_config();
    config.ipc_port = free_port();
    config.actuation_enabled = true;
    config.actuators = {runtime_actuator(nomad::runtime::ActuatorBehavior::ServoToggle)};
    auto connection = std::make_unique<FakeConnection>();
    auto *observed = connection.get();
    nomad::runtime::Runtime runtime(std::move(connection), config);
    start_ready(runtime, config.ipc_port);
    admit_authority(config.ipc_port);
    Client client(config.ipc_port);
    CHECK(client.request(servo_request("raw-configured", 2000))["error"]["code"] == "configured_output");
    CHECK(observed->command_count() == 0);
    CHECK(client.request(actuator_request("startup-activation", "activate"))["error"]["code"] ==
          "actuator_action_blocked");
    CHECK(client.request(actuator_request("explicit-safe", "safe"))["command_result"]["success"] == true);
    auto first = actuator_request("confirm-one", "activate");
    const auto accepted = client.request(first);
    CHECK(accepted["type"] == "actuator_response");
    CHECK(accepted["execution_attempted"] == false);
    CHECK(accepted["request_result"]["success"] == true);
    CHECK(accepted["command_result"]["success"] == false);
    CHECK(client.request(first)["execution_attempted"] == false);
    CHECK(observed->command_count() == 1);
    CHECK(client.request(actuator_request("confirm-two", "activate"))["execution_attempted"] == true);
    CHECK(observed->command_count() == 2);
    auto discovery = base_request("discover", "get_actuators");
    CHECK(client.request(discovery)["actuators"][0]["name"] == "Operator configured name");
    discovery.erase("credential");
    CHECK(client.request(discovery)["error"]["code"] == "authentication_failed");
    CHECK(client.request(actuator_request("ui-neutral", "neutral"))["error"]["code"] == "invalid_request");
    CHECK(observed->command_count() == 2);
    CHECK(client.request(actuator_request("safe-after-active", "safe"))["command_result"]["success"] == true);
    auto hid = actuator_request("held-hid", "activate", "hid", 0);
    CHECK(client.request(hid)["execution_attempted"] == false);
    CHECK(client.request(actuator_request("hid-neutral", "neutral", "hid", 0))["execution_attempted"] == false);
    CHECK(client.request(actuator_request("hid-first", "activate", "hid", 0))["execution_attempted"] == false);
    CHECK(client.request(actuator_request("hid-held", "activate", "hid", 0))["execution_attempted"] == false);
    CHECK(observed->command_count() == 3);
    CHECK(client.request(actuator_request("hid-neutral-again", "neutral", "hid", 0))["execution_attempted"] == false);
    CHECK(client.request(actuator_request("hid-final", "activate", "hid", 0))["execution_attempted"] == true);
    CHECK(observed->command_count() == 4);
    runtime.stop();
}

void test_backend_pulse_failure_and_explicit_recovery() {
    auto config = test_config();
    config.ipc_port = free_port();
    config.actuation_enabled = true;
    config.actuators = {runtime_actuator(nomad::runtime::ActuatorBehavior::RelayPulse)};
    auto connection = std::make_unique<FakeConnection>();
    auto *observed = connection.get();
    nomad::runtime::Runtime runtime(std::move(connection), config);
    start_ready(runtime, config.ipc_port);
    admit_authority(config.ipc_port);
    Client client(config.ipc_port);
    CHECK(client.request(actuator_request("pulse-safe", "safe"))["command_result"]["success"] == true);
    CHECK(client.request(actuator_request("pulse-confirm-one", "activate"))["execution_attempted"] == false);
    observed->before_command_send = [&] {
        if (observed->command_count() == 2) {
            observed->acknowledgement = nomad::mavlink::CommandAck{181, 1};
        }
    };
    const auto failed = client.request(actuator_request("pulse-confirm-two", "activate"));
    CHECK(failed["outcome"] == "failed");
    CHECK(failed["actuator_state"]["activation_outcome"] == "success");
    CHECK(failed["actuator_state"]["safe_outcome"] == "failed");
    CHECK(failed["actuator_state"]["pulse_on_succeeded"] == true);
    CHECK(failed["actuator_state"]["recovery_required"] == true);
    CHECK(observed->command_count() == 3);
    CHECK(client.request(actuator_request("pulse-blocked", "activate"))["error"]["code"] == "actuator_action_blocked");
    CHECK(observed->command_count() == 3);
    CHECK(client.request(actuator_request("pulse-safe-fails", "safe"))["actuator_state"]["recovery_required"] == true);
    observed->before_command_send = {};
    observed->acknowledgement = nomad::mavlink::CommandAck{181, 0};
    CHECK(client.request(actuator_request("pulse-safe-succeeds", "safe"))["actuator_state"]["recovery_required"] ==
          false);
    CHECK(observed->command_count() == 5);
    CHECK(client.request(actuator_request("budget-confirm-one", "activate"))["execution_attempted"] == false);
    auto short_budget = actuator_request("budget-confirm-two", "activate");
    short_budget["expires_at_ms"] = nomad::runtime::detail::unix_milliseconds() + 3000;
    CHECK(client.request(short_budget)["error"]["code"] == "insufficient_actuator_budget");
    CHECK(observed->command_count() == 5);
    runtime.stop();
}

void test_backend_configuration_persistence_uncertainty() {
    auto config = test_config();
    config.ipc_port = free_port();
    config.actuation_enabled = true;
    config.actuator_config_file = (std::filesystem::path(config.audit_directory) / "actuators.json").string();
    config.actuators = {runtime_actuator(nomad::runtime::ActuatorBehavior::ServoToggle)};
    config.actuator_directory_sync_guard = [] { return false; };
    auto connection = std::make_unique<FakeConnection>();
    auto *observed = connection.get();
    nomad::runtime::Runtime runtime(std::move(connection), config);
    start_ready(runtime, config.ipc_port);
    admit_authority(config.ipc_port);
    Client client(config.ipc_port);
    CHECK(client.request(actuator_request("config-safe", "safe"))["command_result"]["success"] == true);
    auto changed = config.actuators;
    changed[0].name = "Reviewed replacement name";
    auto request = authority_request("config-sync-failure", "configure_actuators");
    request["actuator_configs"] = nomad::runtime::detail::serialize_actuator_definitions(changed);
    const auto response = client.request(request);
    CHECK(response["outcome"] == "unknown");
    CHECK(response["error"]["code"] == "configuration_recovery_required");
    std::vector<nomad::runtime::ActuatorDefinition> disk;
    std::string error;
    CHECK(nomad::runtime::detail::load_actuator_configuration(config.actuator_config_file, disk, error));
    CHECK(disk[0].name == "Reviewed replacement name");
    CHECK(client.request(base_request("config-discovery", "get_actuators"))["configuration_recovery_required"] == true);
    CHECK(client.request(actuator_request("config-inhibited", "activate"))["error"]["code"] ==
          "configuration_recovery_required");
    CHECK(client.request(servo_request("config-raw-inhibited", 2000))["error"]["code"] == "configured_output");
    CHECK(observed->command_count() == 1);
    CHECK(client.request(actuator_request("config-safe-still-available", "safe"))["command_result"]["success"] == true);
    runtime.stop();
}
