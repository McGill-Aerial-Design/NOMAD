// SPDX-License-Identifier: Apache-2.0
#pragma once

struct PendingActuatorGate {
    std::promise<void> entered;
    std::future<void> entry{entered.get_future()};
    std::promise<void> released;
    std::shared_future<void> release{released.get_future().share()};
    std::atomic_bool opened{false};
    void open() {
        if (!opened.exchange(true)) {
            released.set_value();
        }
    }
    ~PendingActuatorGate() { open(); }
    void install(FakeConnection &connection) {
        connection.before_command_send = [this, &connection] {
            if (connection.command_count() != 1) {
                return;
            }
            entered.set_value();
            if (release.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
                throw std::runtime_error("test ON gate was not explicitly released");
            }
        };
    }
};

void wait_for_actuator_entry(PendingActuatorGate &gate, std::future<Json> &operation, FakeConnection &connection) {
    if (gate.entry.wait_for(std::chrono::seconds(2)) == std::future_status::ready) {
        return;
    }
    const auto response = operation.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready ?
        operation.get().dump() : "request still pending";
    throw std::runtime_error("actuator did not reach ON gate; commands=" +
        std::to_string(connection.command_count()) + "; response=" + response);
}

void wait_for_actuator_safe_intent(Client &client, std::future<Json> &safe, FakeConnection &connection) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    Json state;
    while (std::chrono::steady_clock::now() < deadline) {
        state = client.request(base_request("pending-safe-observer", "get_actuators"))["actuators"][0]["state"];
        if (state["pending"] == true && state["recovery_required"] == true) {
            return;
        }
        if (safe.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
            throw std::runtime_error("safe intent finished before ON gate release; commands=" +
                std::to_string(connection.command_count()) + "; response=" + safe.get().dump());
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    throw std::runtime_error("safe intent not observed; commands=" + std::to_string(connection.command_count()) +
                             "; state=" + state.dump());
}
void test_hid_bidirectional_release_preserves_confirmations_and_stops() {
    auto config = test_config();
    config.ipc_port = free_port();
    config.actuation_enabled = true;
    config.actuators = {runtime_actuator(nomad::runtime::ActuatorBehavior::ServoBidirectional)};
    config.actuators[0].hold_ms = 50;
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
    PendingActuatorGate gate;
    gate.install(*observed);
    const auto final_press = actuator_request("direction-positive-two", "positive", "hid", 0);
    auto motion = std::async(std::launch::async, [&] { return Client(config.ipc_port).request(final_press); });
    wait_for_actuator_entry(gate, motion, *observed);
    CHECK(motion.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready);
    const auto release_request = actuator_request("direction-release-two", "release_input", "hid", 0);
    auto stop = std::async(std::launch::async, [&] { return Client(config.ipc_port).request(release_request); });
    wait_for_actuator_safe_intent(client, stop, *observed);
    CHECK(observed->command_count() == 1);
    gate.open();
    const auto stopped = stop.get();
    CHECK(stopped["execution_attempted"] == true);
    CHECK(stopped["command_result"]["success"] == true);
    CHECK(motion.get()["command_result"]["success"] == true);
    CHECK(stopped["actuator_state"]["recovery_required"] == false);
    CHECK(observed->command_count() == 4);
    runtime.stop();
}

void test_backend_pending_neutral_and_safe_interrupt() {
    auto config = test_config();
    config.ipc_port = free_port();
    config.actuation_enabled = true;
    config.actuators = {runtime_actuator(nomad::runtime::ActuatorBehavior::RelayPulse)};
    config.actuators[0].pulse_ms = 50;
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
    PendingActuatorGate gate;
    gate.install(*observed);
    const auto activation = actuator_request("pending-confirm-two", "activate", "hid", 0);
    auto pulse = std::async(std::launch::async, [&] {
        return Client(config.ipc_port).request(activation);
    });
    wait_for_actuator_entry(gate, pulse, *observed);
    const auto neutral = client.request(actuator_request("pending-neutral-live", "neutral", "hid", 0));
    CHECK(neutral["execution_attempted"] == false);
    CHECK(neutral["actuator_state"]["pending"] == true);
    CHECK(pulse.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready);
    const auto safe_request = actuator_request("pending-explicit-stop", "safe");
    auto stopping = std::async(std::launch::async, [&] { return Client(config.ipc_port).request(safe_request); });
    wait_for_actuator_safe_intent(client, stopping, *observed);
    CHECK(observed->command_count() == 1);
    gate.open();
    const auto safe = stopping.get();
    const auto completed = pulse.get();
    CHECK(completed["command_result"]["success"] == true);
    CHECK(safe["command_result"]["success"] == true);
    CHECK(safe["actuator_state"]["recovery_required"] == false);
    CHECK(safe["actuator_state"]["state_revision"].get<std::uint64_t>() >
          completed["actuator_state"]["state_revision"].get<std::uint64_t>());
    CHECK(observed->command_count() == 4);
    runtime.stop();
}
