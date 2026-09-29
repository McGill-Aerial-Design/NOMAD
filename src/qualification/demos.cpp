// SPDX-License-Identifier: Apache-2.0
// The demo verbs: a canned mission, a fence upload/readback, and a payload
// release. They exist to exercise the core against a vehicle; the SITL
// scenarios in tests/sitl drive the same verbs.
#include "commands.hpp"

#include "nomad/mission/executor.hpp"

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <vector>

int run_mission_demo(nomad::vehicle::Vehicle &vehicle) {
    const nomad::mission::Mission mission{
        nomad::mission::Action{"guided"}, nomad::mission::Action{"arm"}, nomad::mission::Takeoff{5.0F},
        nomad::mission::ReturnToLaunch{}, nomad::mission::Land{},        nomad::mission::Action{"wait_disarmed"},
    };
    nomad::mission::MissionExecutor executor(vehicle);
    const auto result = executor.execute(mission);
    std::cout << "mission_success=" << (result.success ? "true" : "false")
              << " completed_steps=" << result.completed_steps << " message=" << result.message << '\n';
    return result.success ? EXIT_SUCCESS : EXIT_FAILURE;
}

int run_fence_demo(nomad::vehicle::Vehicle &vehicle) {
    const auto state = vehicle.wait_for_telemetry(
        std::chrono::seconds(5), [](const nomad::telemetry::VehicleState &sample) { return sample.position_valid; });
    if (!state.has_value() || !state->position_valid) {
        std::cerr << "fence-demo requires authoritative position telemetry\n";
        return EXIT_FAILURE;
    }
    const auto &position = state->position;
    constexpr double offset = 0.0001;
    const std::vector<nomad::safety::GlobalPoint> boundary{
        {position.latitude_deg - offset, position.longitude_deg - offset},
        {position.latitude_deg - offset, position.longitude_deg + offset},
        {position.latitude_deg + offset, position.longitude_deg + offset},
        {position.latitude_deg + offset, position.longitude_deg - offset},
    };
    const auto upload = vehicle.upload_fence(boundary);
    if (!upload.success) {
        std::cerr << upload.message << '\n';
        return EXIT_FAILURE;
    }
    const auto verification = vehicle.verify_fence_uploaded(boundary);
    std::cout << verification.message << '\n';
    return verification.success ? EXIT_SUCCESS : EXIT_FAILURE;
}

int run_payload_demo(nomad::vehicle::Vehicle &vehicle, int relay_number, float duration_seconds) {
    const auto armed = vehicle.arm_payload();
    if (!armed.success) {
        std::cerr << armed.message << '\n';
        return EXIT_FAILURE;
    }
    return print_result(vehicle.release_payload(relay_number, duration_seconds));
}
