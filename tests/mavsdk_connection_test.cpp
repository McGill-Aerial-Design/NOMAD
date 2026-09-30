// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
//
// Contract tests for the MAVSDK-backed MavlinkConnection that do not
// need a live peer: configuration validation and fail-closed behavior. The
// peer-driven accepted/denied/timeout cases run from the Python fixture
// (scripts/dev/mavsdk_connection_fixture.py), which drives the real CLI.

#include "nomad/mavlink/connection.hpp"
#include "nomad/mavlink/mavsdk_transport.hpp"
#include "nomad/safety/geofence.hpp"
#include "nomad/vehicle/vehicle.hpp"
#include "support/test_harness.hpp"

#ifdef NDEBUG
#undef NDEBUG
#endif

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

namespace {

using nomad::mavlink::Command;
using nomad::mavlink::FencePlanItem;
using nomad::mavlink::FencePoint;
using nomad::mavlink::MavlinkConnection;
using nomad::mavlink::VelocitySetpoint;
using nomad::safety::GlobalPoint;

constexpr std::uint8_t kSystemId = 1;
constexpr std::uint16_t kArmCommand = 400;

bool connect_with(const std::string &endpoint, std::uint8_t system_id, std::chrono::milliseconds timeout) {
    const auto connection = nomad::mavlink::make_mavsdk_connection(endpoint, system_id, timeout);
    return connection != nullptr && connection->connect();
}

void test_invalid_configuration_is_refused() {
    // TCP is not a supported NOMAD transport alias, system 0 is not a valid
    // target, and a non-positive discovery window cannot select a peer.
    CHECK(!connect_with("tcp:127.0.0.1:5760", kSystemId, std::chrono::milliseconds(100)));
    CHECK(!connect_with("udpin:0.0.0.0:0", kSystemId, std::chrono::milliseconds(100)));
    CHECK(!connect_with("udpout:127.0.0.1:14655", 0, std::chrono::milliseconds(100)));
    CHECK(!connect_with("udpout:127.0.0.1:14655", kSystemId, std::chrono::milliseconds(0)));
}

void test_unusable_endpoint_is_reported_as_a_link_failure() {
    // An endpoint this core cannot open is a configuration problem, not a
    // missing vehicle, and the caller prints the matching diagnostic.
    const auto connection =
        nomad::mavlink::make_mavsdk_connection("tcp:127.0.0.1:5760", kSystemId, std::chrono::milliseconds(100));
    CHECK(connection != nullptr);
    CHECK(!connection->connect());
    CHECK(connection->get_connect_failure() == nomad::mavlink::ConnectFailure::LinkUnavailable);
}

void test_absent_peer_fails_closed() {
    // No peer replies on this endpoint, so discovery times out and every
    // operation must report failure rather than a fabricated success.
    auto connection = nomad::mavlink::make_mavsdk_connection("udpout:127.0.0.1:14655", kSystemId,
                                                             std::chrono::milliseconds(200));
    CHECK(connection != nullptr);
    CHECK(!connection->connect());
    CHECK(!connection->is_connected());
    // The endpoint opened, so the caller must report a heartbeat timeout rather
    // than an endpoint failure.
    CHECK(connection->get_connect_failure() == nomad::mavlink::ConnectFailure::NoAutopilot);

    CHECK(!connection->wait_for_heartbeat(std::chrono::milliseconds(50)).has_value());
    CHECK(!connection->wait_for_state(std::chrono::milliseconds(50)).has_value());

    const auto state = connection->get_state();
    CHECK(!state.connected);
    CHECK(!state.heartbeat_fresh);
    CHECK(!state.position_valid);

    const Command arm{kArmCommand, {1, 0, 0, 0, 0, 0, 0}};
    CHECK(!connection->send_command(arm, std::chrono::milliseconds(100)).has_value());

    const auto fence = connection->upload_fence_plan({FencePlanItem{}});
    CHECK(!fence);
    CHECK(!connection->send_fence_point(FencePoint{}, 0, 1));
    CHECK(!connection->request_fence_point(0));
    CHECK(!connection->wait_for_fence_point(std::chrono::milliseconds(10)).has_value());
    CHECK(!connection->download_fence_plan(std::chrono::milliseconds(10)).has_value());
    CHECK(!connection->read_param("FENCE_ENABLE", std::chrono::milliseconds(10)).has_value());
}

void test_velocity_with_no_peer_fails_closed() {
    // A velocity setpoint needs a live, latched peer: with no peer at all it
    // must report failure rather than a success NOMAD cannot check. The peer
    // fixture proves the healthy velocity path and the zero setpoint on
    // disconnect, which need a live link.
    auto connection = nomad::mavlink::make_mavsdk_connection("udpout:127.0.0.1:14655", kSystemId,
                                                             std::chrono::milliseconds(50));
    CHECK(connection != nullptr);
    CHECK(!connection->send_velocity(VelocitySetpoint{1.0F, 0.0F, 0.0F, 0.0F}));
    CHECK(!connection->is_velocity_active());
}

// Peer-driven probe used by scripts/dev/mavsdk_connection_fixture.py:
// --probe <endpoint> <system-id> <command-id> <long|int> [timeout-ms]. It
// connects, waits for a heartbeat, sends the command and prints the MAVLink
// result code so the fixture can prove wire handling and caller deadlines.
int run_probe(int argc, char **argv) {
    if (argc != 6 && argc != 7) {
        std::fprintf(
            stderr, "usage: %s --probe <endpoint> <system-id> <command-id> <long|int> [timeout-ms]\n", argv[0]);
        return 2;
    }
    const std::string endpoint = argv[2];
    const auto system_id = static_cast<std::uint8_t>(std::stoi(argv[3]));
    const auto command_id = static_cast<std::uint16_t>(std::stoi(argv[4]));
    const bool use_command_int = std::string_view(argv[5]) == "int";
    const auto command_timeout = argc == 7 ? std::chrono::milliseconds(std::stoll(argv[6]))
                                           : std::chrono::seconds(2);

    auto connection = nomad::mavlink::make_mavsdk_connection(endpoint, system_id, std::chrono::seconds(3));
    if (connection == nullptr || !connection->connect()) {
        std::printf("connect=fail\n");
        return 1;
    }
    if (!connection->wait_for_heartbeat(std::chrono::seconds(3)).has_value()) {
        std::printf("heartbeat=fail\n");
        return 1;
    }
    if (use_command_int) {
        if (command_id != 192) {
            std::fprintf(stderr, "the int probe only supports relative goto\n");
            return 2;
        }
        const bool accepted = connection->goto_location_relative(45.5, -73.6, 10.0F, command_timeout);
        std::printf(accepted ? "ack=0\n" : "ack=none\n");
        return accepted ? 0 : 1;
    }

    const Command command{command_id, {0, 1, 0, 0, 45.5F, -73.6F, 10.0F}};
    const auto ack = connection->send_command(command, command_timeout);
    if (!ack.has_value()) {
        std::printf("ack=none\n");
        return 1;
    }
    std::printf("ack=%d\n", static_cast<int>(ack->result));
    return ack->result == 0 ? 0 : 1;
}

// Peer-driven probe used by scripts/dev/mavsdk_connection_fixture.py:
// --velocity <endpoint> <system-id> <vx> <vy> <vz> <yaw-rate>. It connects,
// streams one setpoint a few times, and reports whether the transport holds
// velocity active until it disconnects. The peer decodes the resulting frames,
// so the fixture asserts the wire bytes and the zero setpoint the disconnect
// must send (SR-LNK-03) rather than this probe's own report.
int run_velocity_probe(int argc, char **argv) {
    if (argc != 8) {
        std::fprintf(stderr, "usage: %s --velocity <endpoint> <system-id> <vx> <vy> <vz> <yaw-rate>\n", argv[0]);
        return 2;
    }
    const std::string endpoint = argv[2];
    const auto system_id = static_cast<std::uint8_t>(std::stoi(argv[3]));
    const VelocitySetpoint setpoint{std::stof(argv[4]), std::stof(argv[5]), std::stof(argv[6]),
                                    std::stof(argv[7])};

    auto connection = nomad::mavlink::make_mavsdk_connection(endpoint, system_id, std::chrono::seconds(3));
    if (connection == nullptr || !connection->connect()) {
        std::printf("connect=fail\n");
        return 1;
    }
    if (!connection->wait_for_heartbeat(std::chrono::seconds(3)).has_value()) {
        std::printf("heartbeat=fail\n");
        return 1;
    }

    // Stream the setpoint like the watchdog-driven core does, not once: a
    // single send would not show the wire what a live stream looks like.
    bool sent = true;
    for (int attempt = 0; attempt < 3; ++attempt) {
        sent = connection->send_velocity(setpoint) && sent;
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }
    const bool active_before = connection->is_velocity_active();

    // A non-finite rate must be refused while the link is live: the peer proves
    // no such frame reached the wire, which a check that only ran on a dead
    // link could not distinguish from an unenforced one.
    const bool refused_invalid = !connection->send_velocity(VelocitySetpoint{std::nanf(""), 0.0F, 0.0F, 0.0F});

    connection->disconnect();
    const bool active_after = connection->is_velocity_active();

    std::printf("sent=%d refused_invalid=%d active_before=%d active_after=%d\n", sent ? 1 : 0, refused_invalid ? 1 : 0,
                active_before ? 1 : 0, active_after ? 1 : 0);
    return sent && refused_invalid && active_before && !active_after ? 0 : 1;
}

// Peer-driven probe used by scripts/dev/mavsdk_connection_fixture.py:
// --staleness <endpoint> <system-id> <peer-lifetime-seconds>. The peer streams
// for that lifetime and then falls silent while its socket stays open, which is
// the worst case a link-loss response must cover. The probe connects, confirms
// the link is live, and then requires the connection to stop reporting it as
// fresh and connected once the heartbeat times out. It exits 0 only after
// observing both transitions, so a connection that keeps a dead link "fresh"
// fails here rather than steering a vehicle it can no longer see.
int run_staleness_probe(int argc, char **argv) {
    if (argc != 5) {
        std::fprintf(stderr, "usage: %s --staleness <endpoint> <system-id> <peer-lifetime-seconds>\n", argv[0]);
        return 2;
    }
    const std::string endpoint = argv[2];
    const auto system_id = static_cast<std::uint8_t>(std::stoi(argv[3]));
    const auto peer_lifetime = std::chrono::seconds(std::stoi(argv[4]));

    auto connection = nomad::mavlink::make_mavsdk_connection(endpoint, system_id, std::chrono::seconds(3));
    if (connection == nullptr || !connection->connect()) {
        std::printf("connect=fail\n");
        return 1;
    }
    if (!connection->wait_for_heartbeat(std::chrono::seconds(2)).has_value()) {
        std::printf("heartbeat=fail\n");
        return 1;
    }
    const bool live_before = connection->get_state().heartbeat_fresh;

    // The heartbeat timeout is 3 s, so the peer's lifetime plus a margin must
    // be enough for the staleness to become observable.
    const auto deadline = std::chrono::steady_clock::now() + peer_lifetime + std::chrono::seconds(6);
    while (std::chrono::steady_clock::now() < deadline) {
        const auto state = connection->get_state();
        if (!state.heartbeat_fresh && !state.connected) {
            std::printf("live_before=%d fresh=false connected=false\n", live_before ? 1 : 0);
            return live_before ? 0 : 1;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    std::printf("still_fresh=true\n");
    return 1;
}

struct FenceChecks {
    bool upload_ok{false};
    bool verify_ok{false};
    bool bad_count_refused{false};
    bool bad_point_refused{false};
    bool mismatch_refused{false};
};

// Upload one boundary, read it back, and probe the refusal paths: a boundary
// below the three-vertex minimum, a non-finite vertex, and an expected boundary
// the autopilot does not hold. The expected boundary here must match
// FENCE_BOUNDARY in scripts/dev/mavsdk_link_fixture.py.
FenceChecks run_fence_checks(nomad::vehicle::Vehicle &vehicle) {
    const std::vector<GlobalPoint> boundary{
        {45.5000000, -73.6000000},
        {45.5100000, -73.6000000},
        {45.5050000, -73.5900000},
    };
    FenceChecks checks;
    checks.upload_ok = vehicle.upload_fence(boundary).success;
    checks.verify_ok = vehicle.verify_fence_uploaded(boundary).success;
    checks.bad_count_refused = !vehicle.upload_fence({{45.5, -73.6}, {45.51, -73.6}}).success;
    const std::vector<GlobalPoint> nonfinite{{45.5, -73.6}, {45.51, -73.6}, {std::nan(""), -73.59}};
    checks.bad_point_refused = !vehicle.upload_fence(nonfinite).success;
    const std::vector<GlobalPoint> other{
        {45.5000000, -73.6000000},
        {45.5100000, -73.6000000},
        {45.5200000, -73.5900000},
    };
    checks.mismatch_refused = !vehicle.verify_fence_uploaded(other).success;
    return checks;
}

// Peer-driven probe used by scripts/dev/mavsdk_connection_fixture.py:
// --fence <endpoint> <system-id> <enabled|disabled>. It drives the real core
// fence path (Vehicle::upload_fence, then Vehicle::verify_fence_uploaded) over
// the MAVSDK transport against a peer that stores the fence mission it is sent.
// The fixture asserts the polygon the peer decoded, so a verified result cannot
// come from the transport's own bookkeeping.
int run_fence_probe(int argc, char **argv) {
    if (argc != 5) {
        std::fprintf(stderr, "usage: %s --fence <endpoint> <system-id> <enabled|disabled>\n", argv[0]);
        return 2;
    }
    const std::string endpoint = argv[2];
    const auto system_id = static_cast<std::uint8_t>(std::stoi(argv[3]));
    const bool expect_enabled = std::string_view(argv[4]) == "enabled";

    auto connection = nomad::mavlink::make_mavsdk_connection(endpoint, system_id, std::chrono::seconds(3));
    if (connection == nullptr || !connection->connect()) {
        std::printf("connect=fail\n");
        return 1;
    }
    if (!connection->wait_for_heartbeat(std::chrono::seconds(3)).has_value()) {
        std::printf("heartbeat=fail\n");
        return 1;
    }

    nomad::vehicle::Vehicle vehicle(*connection);
    const auto checks = run_fence_checks(vehicle);
    const bool matches_expectation = checks.verify_ok == expect_enabled;
    std::printf("upload_ok=%d verify_ok=%d matches_expectation=%d bad_count_refused=%d bad_point_refused=%d "
                "mismatch_refused=%d\n",
                checks.upload_ok ? 1 : 0, checks.verify_ok ? 1 : 0, matches_expectation ? 1 : 0,
                checks.bad_count_refused ? 1 : 0, checks.bad_point_refused ? 1 : 0,
                checks.mismatch_refused ? 1 : 0);
    // With the fence disabled the mismatch call already fails on FENCE_ENABLE,
    // so only the enabled run can distinguish a mismatch from a disabled fence.
    const bool mismatch_holds = checks.mismatch_refused || !expect_enabled;
    return checks.upload_ok && matches_expectation && checks.bad_count_refused && checks.bad_point_refused &&
                   mismatch_holds
               ? 0
               : 1;
}

// Peer-driven probe used by the parameter timeout fixture:
// --param <endpoint> <system-id> <param-id> <timeout-ms>. An unknown parameter
// makes the peer stay silent, so a nonzero result must arrive within the
// caller's budget rather than MAVSDK's default transfer timeout.
int run_param_probe(int argc, char **argv) {
    if (argc != 6) {
        std::fprintf(stderr, "usage: %s --param <endpoint> <system-id> <param-id> <timeout-ms>\n", argv[0]);
        return 2;
    }
    const std::string endpoint = argv[2];
    const auto system_id = static_cast<std::uint8_t>(std::stoi(argv[3]));
    const std::string param_id = argv[4];
    const auto timeout = std::chrono::milliseconds(std::stoll(argv[5]));

    auto connection = nomad::mavlink::make_mavsdk_connection(endpoint, system_id, std::chrono::seconds(3));
    if (connection == nullptr || !connection->connect()) {
        std::printf("connect=fail\n");
        return 1;
    }
    if (!connection->wait_for_heartbeat(std::chrono::seconds(3)).has_value()) {
        std::printf("heartbeat=fail\n");
        return 1;
    }
    const auto value = connection->read_param(param_id, timeout);
    std::printf("param=%s\n", value.has_value() ? "value" : "none");
    return value.has_value() ? 0 : 1;
}

int run_qland_probe(int argc, char **argv) {
    if (argc != 4) {
        std::fprintf(stderr, "usage: %s --qland <endpoint> <system-id>\n", argv[0]);
        return 2;
    }
    auto connection = nomad::mavlink::make_mavsdk_connection(
        argv[2], static_cast<std::uint8_t>(std::stoi(argv[3])), std::chrono::seconds(3));
    if (connection == nullptr || !connection->connect() ||
        !connection->wait_for_heartbeat(std::chrono::seconds(3)).has_value()) {
        std::printf("connect=fail\n");
        return 1;
    }
    const auto landed_state_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    auto state = connection->get_state();
    while ((!state.landed_state_valid || state.landed_state != nomad::telemetry::LandedState::OnGround) &&
           std::chrono::steady_clock::now() < landed_state_deadline) {
        static_cast<void>(connection->wait_for_state(std::chrono::milliseconds(50)));
        state = connection->get_state();
    }
    if (!state.landed_state_valid || state.landed_state != nomad::telemetry::LandedState::OnGround) {
        std::printf("landed_state=missing\n");
        return 1;
    }
    const auto version = connection->read_autopilot_version(std::chrono::seconds(3));
    if (!version) {
        std::printf("version=none\n");
        return 1;
    }
    const Command qland{176, {1.0F, 20.0F, 0, 0, 0, 0, 0}};
    const auto ack = connection->send_command(qland, connection->get_state().session_id, std::chrono::seconds(2));
    const auto landed_state_name = nomad::telemetry::landed_state_name(state.landed_state);
    std::printf("version=%u.%u.%u hash=%s landed_state=%.*s ack=%d\n", version->major, version->minor,
                version->patch, version->git_hash.c_str(), static_cast<int>(landed_state_name.size()),
                landed_state_name.data(), ack ? static_cast<int>(ack->result) : -1);
    return version->major == 4 && version->minor == 7 && version->patch == 1 &&
                   version->git_hash == "dbe79216" && state.landed_state == nomad::telemetry::LandedState::OnGround &&
                   ack && ack->command == 176 && ack->result == 0
               ? 0
               : 1;
}

} // namespace

int main(int argc, char **argv) {
    if (argc >= 2 && std::string_view(argv[1]) == "--probe") {
        return run_probe(argc, argv);
    }
    if (argc >= 2 && std::string_view(argv[1]) == "--staleness") {
        return run_staleness_probe(argc, argv);
    }
    if (argc >= 2 && std::string_view(argv[1]) == "--velocity") {
        return run_velocity_probe(argc, argv);
    }
    if (argc >= 2 && std::string_view(argv[1]) == "--fence") {
        return run_fence_probe(argc, argv);
    }
    if (argc >= 2 && std::string_view(argv[1]) == "--param") {
        return run_param_probe(argc, argv);
    }
    if (argc >= 2 && std::string_view(argv[1]) == "--qland") {
        return run_qland_probe(argc, argv);
    }
    const int result = nomad::test::run_tests([] {
        test_invalid_configuration_is_refused();
        test_unusable_endpoint_is_reported_as_a_link_failure();
        test_absent_peer_fails_closed();
        test_velocity_with_no_peer_fails_closed();
    });
    if (result == 0) {
        std::printf("MAVSDK connection contract tests passed\n");
    }
    return result;
}
