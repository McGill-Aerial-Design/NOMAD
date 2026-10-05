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

void test_hid_bidirectional_release_preserves_confirmations_and_stops() {
    auto config = test_config();
    config.ipc_port = free_port();
    config.actuation_enabled = true;
    config.actuators = {runtime_actuator(nomad::runtime::ActuatorBehavior::ServoBidirectional)};
    config.actuators[0].hold_ms = 1500;
    auto connection = std::make_unique<FakeConnection>();
    auto *observed = connection.get();
    nomad::runtime::Runtime runtime(std::move(connection), config);
    start_ready(runtime, config.ipc_port);
    admit_authority(config.ipc_port);
    Client client(config.ipc_port);
    const auto description = client.request(base_request("release-metadata", "get_actuators"));
    CHECK(description["actuators"][0]["actions"][0]["release_operation"] == "release_input");
    CHECK(client.request(actuator_request("direction-safe", "safe"))["command_result"]["success"] == true);
    client.request(actuator_request("direction-neutral-one", "neutral", "hid", 0));
    CHECK(client.request(actuator_request("direction-positive-one", "positive", "hid", 0))["execution_attempted"] ==
          false);
    const auto release = client.request(actuator_request("direction-release-one", "release_input", "hid", 0));
    CHECK(release["execution_attempted"] == false);
    CHECK(release["actuator_state"]["confirmation_remaining"] == 1);
    CHECK(observed->command_count() == 1);
    client.request(actuator_request("direction-neutral-two", "neutral", "hid", 0));
    const auto final_press = actuator_request("direction-positive-two", "positive", "hid", 0);
    auto motion = std::async(std::launch::async, [&] { return Client(config.ipc_port).request(final_press); });
    wait_until([&] { return observed->command_count() == 2; });
    CHECK(motion.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready);
    const auto stopped = client.request(actuator_request("direction-release-two", "release_input", "hid", 0));
    CHECK(stopped["execution_attempted"] == true);
    CHECK(stopped["command_result"]["success"] == true);
    CHECK(motion.get()["command_result"]["success"] == true);
    CHECK(stopped["actuator_state"]["recovery_required"] == false);
    CHECK(observed->command_count() == 4);
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

void test_backend_pending_neutral_and_safe_interrupt() {
    auto config = test_config();
    config.ipc_port = free_port();
    config.actuation_enabled = true;
    config.actuators = {runtime_actuator(nomad::runtime::ActuatorBehavior::RelayPulse)};
    config.actuators[0].pulse_ms = 1500;
    auto connection = std::make_unique<FakeConnection>();
    auto *observed = connection.get();
    nomad::runtime::Runtime runtime(std::move(connection), config);
    start_ready(runtime, config.ipc_port);
    admit_authority(config.ipc_port);
    Client client(config.ipc_port);
    CHECK(client.request(actuator_request("pending-safe", "safe"))["command_result"]["success"] == true);
    client.request(actuator_request("pending-neutral-one", "neutral", "hid", 0));
    CHECK(client.request(actuator_request("pending-confirm-one", "activate", "hid", 0))["execution_attempted"] ==
          false);
    client.request(actuator_request("pending-neutral-two", "neutral", "hid", 0));
    const auto activation = actuator_request("pending-confirm-two", "activate", "hid", 0);
    auto pulse = std::async(std::launch::async, [&] {
        return Client(config.ipc_port).request(activation);
    });
    wait_until([&] { return observed->command_count() == 2; });
    const auto neutral = client.request(actuator_request("pending-neutral-live", "neutral", "hid", 0));
    CHECK(neutral["execution_attempted"] == false);
    CHECK(neutral["actuator_state"]["pending"] == true);
    CHECK(pulse.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready);
    const auto safe = client.request(actuator_request("pending-explicit-stop", "safe"));
    const auto completed = pulse.get();
    CHECK(completed["command_result"]["success"] == true);
    CHECK(safe["command_result"]["success"] == true);
    CHECK(safe["actuator_state"]["recovery_required"] == false);
    CHECK(safe["actuator_state"]["state_revision"].get<std::uint64_t>() >
          completed["actuator_state"]["state_revision"].get<std::uint64_t>());
    CHECK(observed->command_count() == 4);
    runtime.stop();
}
