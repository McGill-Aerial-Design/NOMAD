// SPDX-License-Identifier: Apache-2.0
// The telemetry readout and the shared result printer.
//
// The verb surface, its usage text and the API-key boundary live in
// cli_command_table.hpp; argument parsing in cli_arguments.cpp; the velocity
// and demo verbs in cli_velocity.cpp and cli_demos.cpp.
#include "commands.hpp"

#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <iostream>

namespace {

using Clock = std::chrono::steady_clock;

long long age_milliseconds(Clock::time_point timestamp) {
    return std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - timestamp).count();
}

void print_state(const nomad::telemetry::VehicleState &state) {
    std::cout << "system=" << static_cast<int>(state.system_id) << " component=" << static_cast<int>(state.component_id)
              << " connected=" << (state.connected ? "true" : "false") << " armed=" << (state.armed ? "true" : "false")
              << " mode=" << state.custom_mode << '\n';
    std::cout << "heartbeat_fresh=" << (state.heartbeat_fresh ? "true" : "false")
              << " autopilot_type=" << static_cast<int>(state.identity.autopilot_type)
              << " vehicle_type=" << static_cast<int>(state.identity.vehicle_type)
              << " aircraft_class=" << nomad::telemetry::aircraft_class_name(state.identity.aircraft_class) << '\n';
    if (state.position_valid) {
        std::cout << "position=" << state.position.latitude_deg << ',' << state.position.longitude_deg
                  << " altitude_m=" << state.position.altitude_m
                  << " relative_altitude_m=" << state.position.relative_altitude_m
                  << " position_age_ms=" << age_milliseconds(state.position_updated_at) << '\n';
    }
    if (state.battery_valid) {
        std::cout << "battery_v=" << state.battery.voltage_v << " remaining_percent=" << state.battery.remaining_percent
                  << '\n';
    }
    if (state.gps_valid) {
        std::cout << "gps_fix=" << static_cast<int>(state.gps.fix_type)
                  << " satellites=" << static_cast<int>(state.gps.satellites)
                  << " gps_age_ms=" << age_milliseconds(state.gps_updated_at) << '\n';
    }
    if (state.velocity_updated_at != std::chrono::steady_clock::time_point{}) {
        std::cout << "groundspeed_mps=" << state.velocity.groundspeed_mps
                  << " climb_rate_mps=" << state.velocity.climb_rate_mps
                  << " velocity_age_ms=" << age_milliseconds(state.velocity_updated_at) << '\n';
    }
    if (state.attitude_valid) {
        std::cout << "attitude=" << state.attitude.roll_deg << ',' << state.attitude.pitch_deg << ','
                  << state.attitude.yaw_deg << " attitude_age_ms=" << age_milliseconds(state.attitude_updated_at)
                  << '\n';
    }
    if (state.vtol_state_valid) {
        std::cout << "vtol_state=" << nomad::telemetry::vtol_state_name(state.vtol_state)
                  << " vtol_state_age_ms=" << age_milliseconds(state.vtol_state_updated_at) << '\n';
    }
    if (state.landed_state_valid) {
        std::cout << "landed_state=" << nomad::telemetry::landed_state_name(state.landed_state)
                  << " landed_state_age_ms=" << age_milliseconds(state.landed_state_updated_at) << '\n';
    }
}

} // namespace

int print_result(const nomad::vehicle::CommandResult &result) {
    std::cout << result.message << '\n';
    return result.success ? EXIT_SUCCESS : EXIT_FAILURE;
}

int run_status(nomad::mavlink::MavlinkConnection &connection) {
    // Wait for the first heartbeat so the target system is known. The MAVProxy
    // link already forwards ArduPilot's default streams, so no explicit stream
    // requests are needed; requesting them only loads the link ahead of the
    // next command's acknowledgement.
    if (!connection.wait_for_heartbeat(std::chrono::seconds(6)).has_value()) {
        std::cerr << "timed out waiting for ArduPilot heartbeat\n";
        return EXIT_FAILURE;
    }

    // Give the lower-rate GPS/position messages time to arrive, then print the
    // freshest sample even when the complete set never did.
    nomad::vehicle::Vehicle vehicle(connection);
    const auto state = vehicle.wait_for_telemetry(
        std::chrono::seconds(5), [](const nomad::telemetry::VehicleState &sample) {
            return sample.heartbeat_fresh && sample.position_valid && sample.gps_valid && sample.attitude_valid;
        });
    if (!state.has_value()) {
        std::cerr << "timed out waiting for ArduPilot telemetry\n";
        return EXIT_FAILURE;
    }
    print_state(*state);
    return EXIT_SUCCESS;
}
