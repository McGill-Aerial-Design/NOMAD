// SPDX-License-Identifier: Apache-2.0
// The velocity verbs: a timed setpoint stream and a short fixed demo. Both
// prove that the watchdog, not the caller, stops the vehicle when the stream
// ends.
#include "commands.hpp"

#include "nomad/safety/watchdog.hpp"

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <thread>

namespace {

// A velocity setpoint means nothing unless the vehicle is armed in GUIDED mode
// with a live link, so both verbs start here.
int require_armed_guided(nomad::vehicle::Vehicle &vehicle, const char *verb) {
    const auto state = vehicle.wait_for_state(std::chrono::seconds(3));
    if (!state.has_value()) {
        std::cerr << "timed out waiting for ArduPilot telemetry\n";
        return EXIT_FAILURE;
    }
    if (!state->armed || !nomad::telemetry::is_guided_mode(state->identity.aircraft_class, state->custom_mode)) {
        std::cerr << verb << " requires an armed vehicle in its aircraft GUIDED mode\n";
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}

// The shared verdict: velocity control must be off and the watchdog must report
// the command timeout, not a failure it discovered on its own.
int report_velocity_stop(nomad::vehicle::Vehicle &vehicle) {
    const auto reason = vehicle.last_velocity_stop_reason();
    const bool stopped = !vehicle.velocity_control_active();
    std::cout << "velocity_active=" << (vehicle.velocity_control_active() ? "true" : "false")
              << " watchdog_reason=" << nomad::safety::watchdog_reason_name(reason) << '\n';
    return stopped && reason == nomad::safety::WatchdogReason::command_timeout ? EXIT_SUCCESS : EXIT_FAILURE;
}

} // namespace

int run_velocity(nomad::vehicle::Vehicle &vehicle, float vx, float vy, float vz, float yaw_rate,
                 float duration_seconds) {
    if (require_armed_guided(vehicle, "velocity") != EXIT_SUCCESS) {
        return EXIT_FAILURE;
    }
    const nomad::safety::VelocityCommand command{vx, vy, vz, yaw_rate};
    if (!std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(vz) || !std::isfinite(yaw_rate) ||
        !std::isfinite(duration_seconds) || duration_seconds <= 0.0F) {
        std::cerr << "velocity arguments must be finite and duration must be positive\n";
        return EXIT_FAILURE;
    }

    // Stream setpoints through the core, refreshing the VIO feed so the
    // velocity gate stays open for the whole run. The watchdog owns stopping
    // once the stream ends (command timeout).
    int accepted = 0;
    int rejected = 0;
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(static_cast<int>(duration_seconds * 1000.0F));
    while (std::chrono::steady_clock::now() < deadline) {
        vehicle.update_vio(true, 1.0F);
        // Pump the link so heartbeats keep the velocity gate fresh; nothing
        // else receives while this loop streams setpoints.
        vehicle.wait_for_state(std::chrono::milliseconds(1));
        const auto result = vehicle.set_velocity(command);
        if (!result.success) {
            rejected += 1;
            std::cerr << result.message << '\n';
            break;
        }
        accepted += 1;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    std::cout << "velocity_accepted=" << accepted << " velocity_rejected=" << rejected << '\n';
    if (rejected > 0) {
        return EXIT_FAILURE;
    }

    // Let the watchdog observe the stopped stream and stop the vehicle.
    std::this_thread::sleep_for(std::chrono::seconds(2));
    return report_velocity_stop(vehicle);
}

int run_velocity_demo(nomad::vehicle::Vehicle &vehicle) {
    if (require_armed_guided(vehicle, "velocity-demo") != EXIT_SUCCESS) {
        return EXIT_FAILURE;
    }
    const auto vio_result = vehicle.update_vio(true, 1.0F);
    if (!vio_result.success) {
        std::cerr << vio_result.message << '\n';
        return EXIT_FAILURE;
    }
    const auto velocity_result = vehicle.set_velocity({0.2F, 0.0F, 0.0F, 0.0F});
    if (!velocity_result.success) {
        std::cerr << velocity_result.message << '\n';
        return EXIT_FAILURE;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(700));
    return report_velocity_stop(vehicle);
}
