// SPDX-License-Identifier: Apache-2.0
// The demo verbs: a canned mission, a fence upload/readback, and a payload
// release. They exist to exercise the core against a vehicle; the SITL
// scenarios in tests/sitl drive the same verbs.
#include "commands.hpp"


#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <thread>
#include <vector>

int run_mission_demo(nomad::vehicle::Vehicle &vehicle) {
    std::size_t completed = 0;
    const auto step = [&completed](const nomad::vehicle::CommandResult &result) {
        if (!result.success) {
            std::cerr << "mission failed after " << completed << " steps: " << result.message << '\n';
            return false;
        }
        ++completed;
        return true;
    };
    if (!step(vehicle.set_guided_mode())) {
        return EXIT_FAILURE;
    }
    if (!step(vehicle.arm())) {
        return EXIT_FAILURE;
    }
    if (!step(vehicle.takeoff(5.0F))) {
        return EXIT_FAILURE;
    }
    if (!step(vehicle.return_to_launch())) {
        return EXIT_FAILURE;
    }
    if (!step(vehicle.land())) {
        return EXIT_FAILURE;
    }
    if (!step(vehicle.wait_until_disarmed(std::chrono::seconds(90)))) {
        return EXIT_FAILURE;
    }
    std::cout << "mission_success=true completed_steps=" << completed << " message=mission completed\n";
    return EXIT_SUCCESS;
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

// Qualification-only output scenario; production actuator behavior is owned by runtime IPC.
int run_payload_demo(nomad::vehicle::Vehicle &vehicle, int relay_number, float duration_seconds) {
    if (!std::isfinite(duration_seconds)) {
        return print_result({false, "pulse duration must be finite"});
    }
    const auto on = vehicle.set_relay(relay_number, true);
    if (!on.success) {
        return print_result(on);
    }
    std::this_thread::sleep_for(std::chrono::duration<float>(std::clamp(duration_seconds, 0.05F, 5.0F)));
    return print_result(vehicle.set_relay(relay_number, false));
}
