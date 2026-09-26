// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
//
// SR-LNK-03 on the MAVSDK transport: the safety zero setpoint must reach the
// wire when the watchdog stops the stream, when the caller stops velocity, when
// the vehicle is destroyed, and when the link or the VIO feed dies.
//
// This binary is the loopback proof on the transport that remains (the legacy
// codec's equivalent was deleted at the Phase E cutover), driven by
// scripts/dev/mavsdk_connection_fixture.py against a deterministic peer. The
// peer decodes what actually arrived, so the fixture judges the wire rather than
// this binary's own report.
//
// Without arguments it runs the no-link failure paths for CTest. The fixture
// invokes it as `--zero-delivery <endpoint> <system-id> <scenario>`.
#include "nomad/mavlink/mavsdk_transport.hpp"
#include "nomad/vehicle/vehicle.hpp"
#include "test_harness.hpp"

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <memory>
#include <string>
#include <string_view>
#include <thread>

namespace {

constexpr std::uint8_t kSystemId = 1;
constexpr auto kConnectTimeout = std::chrono::seconds(3);
constexpr auto kStreamPeriod = std::chrono::milliseconds(50);

// Must match STREAMED_SETPOINT in scripts/dev/mavsdk_connection_fixture.py: the
// fixture asserts these rates on the wire, sign-converted for the body-frame
// NED convention, so both sides have to name the same setpoint.
constexpr nomad::safety::VelocityCommand kStreamedVelocity{0.5F, 0.0F, 0.0F, 0.0F};

bool wait_until(const std::function<bool()> &ready, std::chrono::milliseconds budget) {
    const auto deadline = std::chrono::steady_clock::now() + budget;
    while (std::chrono::steady_clock::now() < deadline) {
        if (ready()) {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return false;
}

// Wait for the watchdog to stop the vehicle. The VIO feed is kept alive for the
// scenarios where only the command stream or the link may go stale.
void wait_for_velocity_stop(nomad::vehicle::Vehicle &vehicle, bool feed_vio) {
    static_cast<void>(wait_until(
        [&] {
            if (feed_vio) {
                static_cast<void>(vehicle.update_vio(true, 1.0F));
            }
            return !vehicle.velocity_control_active();
        },
        std::chrono::seconds(10)));
}

void stream_for(nomad::vehicle::Vehicle &vehicle, std::chrono::milliseconds duration) {
    const auto deadline = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < deadline) {
        static_cast<void>(vehicle.update_vio(true, 1.0F));
        static_cast<void>(vehicle.set_velocity(kStreamedVelocity));
        std::this_thread::sleep_for(kStreamPeriod);
    }
}

// Bring the vehicle to the state a velocity stream requires: armed, GUIDED, VIO
// healthy, and the first non-zero setpoint accepted.
std::unique_ptr<nomad::vehicle::Vehicle> start_streaming(nomad::mavlink::MavlinkConnection &connection,
                                                        const nomad::safety::WatchdogPolicy &policy) {
    if (!connection.wait_for_heartbeat(std::chrono::seconds(3)).has_value()) {
        return nullptr;
    }
    nomad::vehicle::VehicleConfig config{};
    config.watchdog = policy;
    auto vehicle = std::make_unique<nomad::vehicle::Vehicle>(connection, config);
    if (!vehicle->set_mode(nomad::safety::kGuidedMode).success || !vehicle->arm().success) {
        return nullptr;
    }
    // The firmware round trips above take time, so the VIO observation is
    // refreshed afterwards: a scenario with a short VIO timeout must still be
    // able to start its stream.
    if (!vehicle->update_vio(true, 1.0F).success || !vehicle->set_velocity(kStreamedVelocity).success) {
        return nullptr;
    }
    if (!wait_until([&] { return vehicle->velocity_control_active(); }, std::chrono::seconds(2))) {
        return nullptr;
    }
    return vehicle;
}

nomad::safety::WatchdogPolicy policy_for(std::string_view scenario) {
    nomad::safety::WatchdogPolicy policy{};
    policy.poll_interval = std::chrono::milliseconds(5);
    // Both gates start far away so each scenario isolates the one it proves.
    policy.command_timeout = std::chrono::seconds(30);
    policy.vio_timeout = std::chrono::seconds(30);
    if (scenario == "command-timeout") {
        policy.command_timeout = std::chrono::milliseconds(150);
    } else if (scenario == "vio-stale") {
        policy.vio_timeout = std::chrono::milliseconds(300);
    }
    return policy;
}

void report(const char *scenario, nomad::safety::WatchdogReason reason, bool active) {
    std::printf("scenario=%s reason=%s active=%s\n", scenario, nomad::safety::watchdog_reason_name(reason),
                active ? "true" : "false");
}

// The stream stops, so the watchdog — not the caller — must zero the vehicle.
int run_command_timeout_scenario(nomad::mavlink::MavlinkConnection &connection,
                                 const nomad::safety::WatchdogPolicy &policy) {
    auto vehicle = start_streaming(connection, policy);
    if (vehicle == nullptr) {
        std::printf("start=fail\n");
        return 1;
    }
    stream_for(*vehicle, std::chrono::milliseconds(300));
    wait_for_velocity_stop(*vehicle, true);
    const auto reason = vehicle->last_velocity_stop_reason();
    report("command-timeout", reason, vehicle->velocity_control_active());
    return reason == nomad::safety::WatchdogReason::command_timeout && !vehicle->velocity_control_active() ? 0 : 1;
}

// The caller stops the stream explicitly.
int run_caller_stop_scenario(nomad::mavlink::MavlinkConnection &connection,
                             const nomad::safety::WatchdogPolicy &policy) {
    auto vehicle = start_streaming(connection, policy);
    if (vehicle == nullptr) {
        std::printf("start=fail\n");
        return 1;
    }
    stream_for(*vehicle, std::chrono::milliseconds(200));
    const bool stopped = vehicle->stop_velocity().success;
    report("caller-stop", vehicle->last_velocity_stop_reason(), vehicle->velocity_control_active());
    return stopped && !vehicle->velocity_control_active() ? 0 : 1;
}

// Destroying a vehicle mid-stream must zero it through the destructor.
int run_destructor_scenario(nomad::mavlink::MavlinkConnection &connection,
                            const nomad::safety::WatchdogPolicy &policy) {
    bool destroyed_while_streaming = false;
    {
        auto vehicle = start_streaming(connection, policy);
        if (vehicle == nullptr) {
            std::printf("start=fail\n");
            return 1;
        }
        stream_for(*vehicle, std::chrono::milliseconds(200));
        destroyed_while_streaming = vehicle->velocity_control_active();
    }
    const bool active = connection.is_velocity_active();
    report("vehicle-destructor", nomad::safety::WatchdogReason::none, active);
    return destroyed_while_streaming && !active ? 0 : 1;
}

// The heartbeat dies while the stream is live: the core may not keep steering.
int run_link_loss_scenario(nomad::mavlink::MavlinkConnection &connection,
                           const nomad::safety::WatchdogPolicy &policy) {
    auto vehicle = start_streaming(connection, policy);
    if (vehicle == nullptr) {
        std::printf("start=fail\n");
        return 1;
    }
    wait_for_velocity_stop(*vehicle, true);
    const auto reason = vehicle->last_velocity_stop_reason();
    report("link-loss", reason, vehicle->velocity_control_active());
    return reason == nomad::safety::WatchdogReason::heartbeat_stale && !vehicle->velocity_control_active() ? 0 : 1;
}

// The VIO feed dies while the command stream is live and the link is healthy.
int run_vio_stale_scenario(nomad::mavlink::MavlinkConnection &connection,
                           const nomad::safety::WatchdogPolicy &policy) {
    auto vehicle = start_streaming(connection, policy);
    if (vehicle == nullptr) {
        std::printf("start=fail\n");
        return 1;
    }
    stream_for(*vehicle, std::chrono::milliseconds(200));
    wait_for_velocity_stop(*vehicle, false);
    const auto reason = vehicle->last_velocity_stop_reason();
    report("vio-stale", reason, vehicle->velocity_control_active());
    return reason == nomad::safety::WatchdogReason::vio_stale && !vehicle->velocity_control_active() ? 0 : 1;
}

int run_scenario_probe(int argc, char **argv) {
    if (argc != 5) {
        std::fprintf(stderr, "usage: %s --zero-delivery <endpoint> <system-id> <scenario>\n", argv[0]);
        return 2;
    }
    const std::string endpoint = argv[2];
    const auto system_id = static_cast<std::uint8_t>(std::stoi(argv[3]));
    const std::string scenario = argv[4];

    auto connection = nomad::mavlink::make_mavsdk_connection(endpoint, system_id, kConnectTimeout);
    if (connection == nullptr || !connection->connect()) {
        std::printf("connect=fail\n");
        return 1;
    }
    const auto policy = policy_for(scenario);
    if (scenario == "command-timeout") {
        return run_command_timeout_scenario(*connection, policy);
    }
    if (scenario == "caller-stop") {
        return run_caller_stop_scenario(*connection, policy);
    }
    if (scenario == "vehicle-destructor") {
        return run_destructor_scenario(*connection, policy);
    }
    if (scenario == "link-loss") {
        return run_link_loss_scenario(*connection, policy);
    }
    if (scenario == "vio-stale") {
        return run_vio_stale_scenario(*connection, policy);
    }
    std::fprintf(stderr, "unknown scenario: %s\n", scenario.c_str());
    return 2;
}

// With no reachable peer nothing may claim a velocity stream: the gate rejects
// the command and no watchdog reports a stop it never made.
void test_velocity_stream_is_refused_without_a_link() {
    nomad::safety::WatchdogPolicy policy{};
    policy.command_timeout = std::chrono::milliseconds(100);
    auto connection =
        nomad::mavlink::make_mavsdk_connection("udpout:127.0.0.1:14655", kSystemId, std::chrono::milliseconds(50));
    CHECK(connection != nullptr);
    CHECK(!connection->connect());

    nomad::vehicle::VehicleConfig config{};
    config.watchdog = policy;
    nomad::vehicle::Vehicle vehicle(*connection, config);
    CHECK(vehicle.update_vio(true, 1.0F).success);
    CHECK(!vehicle.set_velocity(kStreamedVelocity).success);
    CHECK(!vehicle.velocity_control_active());
    CHECK(vehicle.last_velocity_stop_reason() == nomad::safety::WatchdogReason::none);
}

} // namespace

int main(int argc, char **argv) {
    if (argc >= 2 && std::string_view(argv[1]) == "--zero-delivery") {
        return run_scenario_probe(argc, argv);
    }
    const int result = nomad::test::run_tests([] { test_velocity_stream_is_refused_without_a_link(); });
    if (result == 0) {
        std::printf("MAVSDK zero-delivery failure paths passed\n");
    }
    return result;
}
