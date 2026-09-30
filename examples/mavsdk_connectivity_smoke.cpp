// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors

#include "nomad/mavlink/mavlink_observation.hpp"

#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <string>
#include <string_view>

namespace {

constexpr std::string_view kDefaultEndpoint = "udpin:0.0.0.0:14550";
constexpr std::string_view kDefaultSystemId = "1";
constexpr auto kDiscoveryTimeout = std::chrono::seconds(5);
constexpr auto kTelemetryTimeout = std::chrono::seconds(8);

void print_usage(const char *program) {
    std::cerr << "Usage: " << program << " <connect|status> [udpin:host:port] [expected-system-id]\n";
}

void print_status(std::uint8_t system_id, const nomad::mavlink::MavlinkObservationSnapshot &snapshot) {
    const auto &values = snapshot.values;
    std::cout << "connected=true system=" << static_cast<int>(system_id)
              << " samples=" << snapshot.position_updates << " observation_ms=" << snapshot.observation_ms
              << " age_ms=" << snapshot.age_ms << '\n';
    std::cout << "position=" << values.latitude_deg << ',' << values.longitude_deg
              << " relative_altitude_m=" << values.relative_altitude_m << '\n';
    std::cout << "battery_v=" << values.battery_voltage_v
              << " remaining_percent=" << values.battery_remaining_percent << '\n';
    std::cout << "gps_fix=" << values.gps_fix_type << " satellites=" << values.satellites
              << " flight_mode=" << values.flight_mode << '\n';
}

int report_connection_failure(const nomad::mavlink::MavlinkObservation &connection) {
    std::cerr << nomad::mavlink::mavlink_observation_error_message(connection.last_error()) << '\n';
    return EXIT_FAILURE;
}

int run_status(nomad::mavlink::MavlinkObservation &connection) {
    const auto status = connection.wait_for_status(kTelemetryTimeout);
    if (!status) {
        const auto snapshot = connection.get_status();
        std::cerr << "telemetry qualification failed: connected="
                  << (connection.is_connected() ? "true" : "false")
                  << " samples=" << snapshot.position_updates << " observation_ms=" << snapshot.observation_ms
                  << " age_ms=" << snapshot.age_ms << '\n';
        return EXIT_FAILURE;
    }
    print_status(connection.system_id(), *status);
    return EXIT_SUCCESS;
}

} // namespace

int main(int argc, char **argv) {
    if (argc < 2 || argc > 4) {
        print_usage(argv[0]);
        return EXIT_FAILURE;
    }
    const auto command = std::string_view(argv[1]);
    if (command != "connect" && command != "status") {
        print_usage(argv[0]);
        return EXIT_FAILURE;
    }

    const auto endpoint = std::string(argc >= 3 ? argv[2] : kDefaultEndpoint);
    const auto expected_id = nomad::mavsdk_phase_a::parse_system_id(argc == 4 ? argv[3] : kDefaultSystemId);
    if (!expected_id) {
        std::cerr << "invalid MAVLink UDP input endpoint or expected system ID\n";
        return EXIT_FAILURE;
    }

    nomad::mavlink::MavlinkObservation connection{{endpoint, *expected_id, kDiscoveryTimeout}};
    if (!connection.connect()) {
        return report_connection_failure(connection);
    }
    if (command == "connect") {
        std::cout << "connected=true system=" << static_cast<int>(connection.system_id()) << '\n';
        return EXIT_SUCCESS;
    }
    return run_status(connection);
}
