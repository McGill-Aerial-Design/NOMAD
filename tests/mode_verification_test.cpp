// SPDX-License-Identifier: Apache-2.0
#include "nomad/vehicle/vehicle.hpp"
#include "support/fake_connection.hpp"
#include "support/test_harness.hpp"

#include <chrono>
#include <string_view>
#include <thread>

namespace {
using Outcome = nomad::vehicle::CommandOutcome;
enum class Path { CopterMode, CopterGuided, QuadGuided };
enum class Observation {
    Fresh, Session, System, Component, Class, Autopilot, VehicleType, Disconnected,
    Cached, ZeroTime, FutureTime, StaleTime, StaleFlag
};

class ModeConnection final : public FakeConnection {
  public:
    ModeConnection(Path path, Observation value) : observation(value) {
        auto_stamp_fresh_fields = false;
        acknowledgement = nomad::mavlink::CommandAck{176, 0};
        if (path == Path::QuadGuided) {
            state->identity = nomad::telemetry::identify_vehicle(
                nomad::telemetry::kArduPilotAutopilot, nomad::telemetry::kVtolQuadrotor);
        }
    }

    std::optional<nomad::mavlink::CommandAck> send_command(
        const nomad::mavlink::Command &command, std::chrono::milliseconds timeout) override {
        const auto result = FakeConnection::send_command(command, timeout);
        return result && result->status != nomad::mavlink::CommandAck::Status::AdmissionCancelled ?
            acknowledgement : result;
    }

    std::optional<nomad::telemetry::VehicleState> wait_for_state(std::chrono::milliseconds) override {
        ++polls;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        std::lock_guard lock(state_mutex);
        if (observation != Observation::Cached) {
            state->heartbeat_updated_at = std::chrono::steady_clock::now();
        }
        switch (observation) {
        case Observation::Session: ++state->session_id; break;
        case Observation::System: ++state->system_id; break;
        case Observation::Component: ++state->component_id; break;
        case Observation::Class: state->identity.aircraft_class = nomad::telemetry::AircraftClass::Plane; break;
        case Observation::Autopilot: ++state->identity.autopilot_type; break;
        case Observation::VehicleType: ++state->identity.vehicle_type; break;
        case Observation::Disconnected: state->connected = false; break;
        case Observation::ZeroTime: state->heartbeat_updated_at = {}; break;
        case Observation::FutureTime: state->heartbeat_updated_at += std::chrono::seconds(10); break;
        case Observation::StaleTime: state->heartbeat_updated_at -= std::chrono::seconds(10); break;
        case Observation::StaleFlag: state->heartbeat_fresh = false; break;
        default: break;
        }
        if (polls == 2) {
            state->connected = false;
        }
        return *state;
    }

    Observation observation;
    int polls{};
};

nomad::vehicle::CommandResult request_mode(nomad::vehicle::Vehicle &vehicle, Path path) {
    return path == Path::CopterMode ? vehicle.set_mode(4) : vehicle.set_guided_mode();
}

void test_mode_rejects_replaced_context(Path path, Observation observation) {
    ModeConnection connection(path, observation);
    CHECK(connection.connect());
    nomad::vehicle::Vehicle vehicle(connection);
    const auto result = request_mode(vehicle, path);
    CHECK(!result.success);
    CHECK(result.acknowledged);
    CHECK(result.outcome == Outcome::Interrupted);
    CHECK(connection.polls == 1);
    CHECK(connection.command_history.size() == 1);
}

void test_mode_requires_post_ack_sample(Path path, Observation observation) {
    ModeConnection connection(path, observation);
    CHECK(connection.connect());
    nomad::vehicle::Vehicle vehicle(connection);
    const auto result = request_mode(vehicle, path);
    CHECK(!result.success);
    CHECK(result.acknowledged);
    CHECK(result.outcome == Outcome::Interrupted);
    CHECK(connection.polls == 2);
    CHECK(connection.command_history.size() == 1);
}

void test_mode_final_send_context(Path path, bool change_class) {
    ModeConnection connection(path, Observation::Fresh);
    CHECK(connection.connect());
    connection.before_command_send = [&] {
        auto state = connection.get_state();
        if (change_class) {
            state.identity.aircraft_class = nomad::telemetry::AircraftClass::Plane;
        } else {
            ++state.session_id;
        }
        connection.set_state(state);
    };
    nomad::vehicle::Vehicle vehicle(connection);
    const auto result = request_mode(vehicle, path);
    CHECK(connection.command_history.empty());
    CHECK(!result.success);
    CHECK(!result.acknowledged);
    CHECK(result.outcome == Outcome::Interrupted);
}

void test_mode_initial_context(Path path) {
    for (int fault = 0; fault < 9; ++fault) {
        ModeConnection connection(path, Observation::Fresh);
        CHECK(connection.connect());
        switch (fault) {
        case 0: connection.state->session_id = 0; break;
        case 1: connection.state->system_id = 0; break;
        case 2: connection.state->component_id = 0; break;
        case 3: connection.state->connected = false; break;
        case 4: connection.state->heartbeat_fresh = false; break;
        case 5: connection.state->heartbeat_updated_at = {}; break;
        case 6: connection.state->heartbeat_updated_at += std::chrono::seconds(10); break;
        case 7: connection.state->heartbeat_updated_at -= std::chrono::seconds(10); break;
        case 8: connection.state->identity.autopilot_type = 0; break;
        }
        nomad::vehicle::Vehicle vehicle(connection);
        const auto result = request_mode(vehicle, path);
        CHECK(!result.success);
        CHECK(!result.acknowledged);
        CHECK(result.outcome == Outcome::Rejected);
        CHECK(connection.command_history.empty());
    }
}

void test_mode_accepts_new_same_context_sample(Path path) {
    ModeConnection connection(path, Observation::Fresh);
    CHECK(connection.connect());
    nomad::vehicle::Vehicle vehicle(connection);
    const auto result = request_mode(vehicle, path);
    CHECK(result.success);
    CHECK(result.acknowledged);
    CHECK(result.outcome == Outcome::Success);
    CHECK(connection.polls == 1);
    CHECK(connection.command_history.size() == 1);
    CHECK(connection.last_command.id == 176);
    CHECK(connection.last_command.parameters[0] == 1.0F);
    CHECK(connection.last_command.parameters[1] == (path == Path::QuadGuided ? 15.0F : 4.0F));
}

void test_mode_ack_faults(Path path) {
    for (int fault = 0; fault < 3; ++fault) {
        ModeConnection connection(path, Observation::Fresh);
        CHECK(connection.connect());
        connection.acknowledgement = fault == 0 ? std::nullopt :
            std::optional{nomad::mavlink::CommandAck{static_cast<std::uint16_t>(fault == 1 ? 177 : 176),
                                                   static_cast<std::uint8_t>(fault == 2 ? 4 : 0)}};
        nomad::vehicle::Vehicle vehicle(connection);
        const auto result = request_mode(vehicle, path);
        CHECK(!result.success);
        CHECK(result.acknowledged == (fault == 2));
        CHECK(result.outcome == (fault == 2 ? Outcome::Failed : Outcome::Unknown));
        CHECK(connection.polls == 0);
        CHECK(connection.command_history.size() == 1);
    }
}

void test_mode_eligibility_is_unchanged() {
    for (const auto aircraft_class : {nomad::telemetry::AircraftClass::Plane,
                                      nomad::telemetry::AircraftClass::Unknown}) {
        ModeConnection connection(Path::CopterMode, Observation::Fresh);
        CHECK(connection.connect());
        connection.state->identity.aircraft_class = aircraft_class;
        nomad::vehicle::Vehicle vehicle(connection);
        CHECK(!vehicle.set_mode(4).success);
        CHECK(!vehicle.set_guided_mode().success);
        CHECK(connection.command_history.empty());
    }
    ModeConnection connection(Path::QuadGuided, Observation::Fresh);
    CHECK(connection.connect());
    nomad::vehicle::Vehicle vehicle(connection);
    CHECK(!vehicle.set_mode(15).success);
    CHECK(connection.command_history.empty());
}
}

int main(int argc, char **argv) {
    return nomad::test::run_tests([argc, argv] {
        if (argc == 2 && std::string_view(argv[1]) == "--mode-session") {
            nomad::test::run_scenario("mode_replaced_session", [] {
                test_mode_rejects_replaced_context(Path::CopterMode, Observation::Session);
            });
            return;
        }
        if (argc == 2 && std::string_view(argv[1]) == "--mode-cached") {
            nomad::test::run_scenario("mode_cached_sample", [] {
                test_mode_requires_post_ack_sample(Path::CopterGuided, Observation::Cached);
            });
            return;
        }
        if (argc == 2 && std::string_view(argv[1]) == "--mode-send-context") {
            nomad::test::run_scenario("mode_final_send_context", [] {
                test_mode_final_send_context(Path::QuadGuided, true);
            });
            return;
        }
        for (const auto path : {Path::CopterMode, Path::CopterGuided, Path::QuadGuided}) {
            test_mode_initial_context(path);
            test_mode_accepts_new_same_context_sample(path);
            for (const auto observation : {Observation::Session, Observation::System, Observation::Component,
                    Observation::Class, Observation::Autopilot, Observation::VehicleType, Observation::Disconnected}) {
                test_mode_rejects_replaced_context(path, observation);
            }
            for (const auto observation : {Observation::Cached, Observation::ZeroTime, Observation::FutureTime,
                    Observation::StaleTime, Observation::StaleFlag}) {
                test_mode_requires_post_ack_sample(path, observation);
            }
            test_mode_final_send_context(path, false);
            test_mode_final_send_context(path, true);
            test_mode_ack_faults(path);
        }
        test_mode_eligibility_is_unchanged();
    });
}
