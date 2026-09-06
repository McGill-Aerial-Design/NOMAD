// SPDX-License-Identifier: Apache-2.0
#include "cli_commands.hpp"

#include "nomad/safety/velocity.hpp"
#include "nomad/safety/watchdog.hpp"
#include "nomad/telemetry/state.hpp"

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <thread>

void print_state(const nomad::telemetry::VehicleState &state) {
    std::cout << "system=" << static_cast<int>(state.system_id) << " component=" << static_cast<int>(state.component_id)
              << " connected=" << (state.connected ? "true" : "false") << " armed=" << (state.armed ? "true" : "false")
              << " mode=" << state.custom_mode << '\n';
    if (state.position_valid) {
        std::cout << "position=" << state.position.latitude_deg << ',' << state.position.longitude_deg
                  << " altitude_m=" << state.position.altitude_m
                  << " relative_altitude_m=" << state.position.relative_altitude_m << '\n';
    }
    if (state.battery_valid) {
        std::cout << "battery_v=" << state.battery.voltage_v << " remaining_percent=" << state.battery.remaining_percent
                  << '\n';
    }
    if (state.gps_valid) {
        std::cout << "gps_fix=" << static_cast<int>(state.gps.fix_type)
                  << " satellites=" << static_cast<int>(state.gps.satellites) << '\n';
    }
}

int run_status(nomad::mavlink::UdpMavlinkConnection &connection) {
    // Wait for the first heartbeat so the target system is known. The MAVProxy
    // link already forwards ArduPilot's default streams, so no explicit stream
    // requests are needed; requesting them only loads the link ahead of the
    // next command's acknowledgement.
    if (!connection.wait_for_heartbeat(std::chrono::seconds(3)).has_value()) {
        std::cerr << "timed out waiting for ArduPilot heartbeat\n";
        return EXIT_FAILURE;
    }

    // Poll until position and GPS telemetry have been accumulated, or time out.
    // wait_for_state returns as soon as any telemetry arrives, so loop to give
    // the lower-rate GPS/position messages time to be processed.
    nomad::vehicle::Vehicle vehicle(connection);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    nomad::telemetry::VehicleState state{};
    bool got_any = false;
    while (std::chrono::steady_clock::now() < deadline) {
        const auto partial = vehicle.wait_for_state(std::chrono::milliseconds(500));
        if (partial.has_value()) {
            got_any = true;
            state = *partial;
            if (state.position_valid && state.gps_valid) {
                break;
            }
        }
    }
    if (!got_any) {
        std::cerr << "timed out waiting for ArduPilot telemetry\n";
        return EXIT_FAILURE;
    }
    print_state(state);
    return EXIT_SUCCESS;
}

int run_velocity(nomad::vehicle::Vehicle &vehicle, float vx, float vy, float vz, float yaw_rate,
                 float duration_seconds) {
    const auto state = vehicle.wait_for_state(std::chrono::seconds(3));
    if (!state.has_value()) {
        std::cerr << "timed out waiting for ArduPilot telemetry\n";
        return EXIT_FAILURE;
    }
    if (!state->armed || state->custom_mode != nomad::safety::kGuidedMode) {
        std::cerr << "velocity requires an armed vehicle in GUIDED mode\n";
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
    const auto reason = vehicle.last_velocity_stop_reason();
    const bool stopped = !vehicle.velocity_control_active();
    std::cout << "velocity_active=" << (vehicle.velocity_control_active() ? "true" : "false")
              << " watchdog_reason=" << nomad::safety::watchdog_reason_name(reason) << '\n';
    return stopped && reason == nomad::safety::WatchdogReason::command_timeout ? EXIT_SUCCESS : EXIT_FAILURE;
}

int run_velocity_demo(nomad::vehicle::Vehicle &vehicle) {
    const auto state = vehicle.wait_for_state(std::chrono::seconds(3));
    if (!state.has_value()) {
        std::cerr << "timed out waiting for ArduPilot telemetry\n";
        return EXIT_FAILURE;
    }
    if (!state->armed || state->custom_mode != nomad::safety::kGuidedMode) {
        std::cerr << "velocity-demo requires an armed vehicle in GUIDED mode\n";
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
    const auto reason = vehicle.last_velocity_stop_reason();
    const bool stopped = !vehicle.velocity_control_active();
    std::cout << "velocity_active=" << (vehicle.velocity_control_active() ? "true" : "false")
              << " watchdog_reason=" << nomad::safety::watchdog_reason_name(reason) << '\n';
    return stopped && reason == nomad::safety::WatchdogReason::command_timeout ? EXIT_SUCCESS : EXIT_FAILURE;
}
