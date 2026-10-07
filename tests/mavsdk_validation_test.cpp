// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors

#include "nomad/mavlink/mavsdk_validation.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string_view>
#include <vector>

namespace {

int failures = 0;

void check(bool condition, std::string_view message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        ++failures;
    }
}

nomad::mavsdk_phase_a::StatusValues valid_status() {
    return {45.5, -73.6, 8.0, 3, 10, 12.6, 75.0, 1};
}

void test_endpoints() {
    using nomad::mavsdk_phase_a::canonicalize_udp_endpoint;
    check(canonicalize_udp_endpoint("udpin:0.0.0.0:14550") == "udpin://0.0.0.0:14550", "canonical udpin");
    check(canonicalize_udp_endpoint("udpout://127.0.0.1:65535") == "udpout://127.0.0.1:65535", "udpout");
    check(!canonicalize_udp_endpoint("udp:0.0.0.0:14550"), "ambiguous udp alias rejected");
    check(!canonicalize_udp_endpoint("tcp:127.0.0.1:5760"), "tcp rejected");
    check(!canonicalize_udp_endpoint("udpin:127.0.0.1:0"), "zero port rejected");
    check(!canonicalize_udp_endpoint("udpin:bad host:14550"), "whitespace host rejected");
    check(!canonicalize_udp_endpoint("udpout:0.0.0.0:14550"), "wildcard output rejected");
    using nomad::mavsdk_phase_a::canonicalize_udp_input_endpoint;
    check(canonicalize_udp_input_endpoint("udpin:0.0.0.0:14550") == "udpin://0.0.0.0:14550",
          "observer accepts UDP input");
    check(!canonicalize_udp_input_endpoint("udpout:127.0.0.1:14550"), "observer rejects UDP output");
}

void test_system_identity() {
    using nomad::mavsdk_phase_a::classify_system_ids;
    using nomad::mavsdk_phase_a::parse_system_id;
    using nomad::mavsdk_phase_a::SystemSelection;
    check(parse_system_id("1") == 1, "system one accepted");
    check(parse_system_id("255") == 255, "system 255 accepted");
    check(!parse_system_id("0") && !parse_system_id("256") && !parse_system_id("one"), "bad IDs rejected");
    check(classify_system_ids({}, 1) == SystemSelection::NoAutopilot, "no autopilot");
    check(classify_system_ids({1}, 1) == SystemSelection::Selected, "expected autopilot");
    check(classify_system_ids({2}, 1) == SystemSelection::WrongPeer, "wrong autopilot");
    check(classify_system_ids({257}, 1) == SystemSelection::WrongPeer, "wide system ID must not alias system 1");
    check(classify_system_ids({255}, 255) == SystemSelection::Selected, "maximum MAVLink system ID");
    check(classify_system_ids({1, 2}, 1) == SystemSelection::Ambiguous, "multiple autopilots");
}

void test_status_validation() {
    using nomad::mavsdk_phase_a::has_valid_battery;
    using nomad::mavsdk_phase_a::has_valid_gps;
    using nomad::mavsdk_phase_a::has_valid_position;
    using nomad::mavsdk_phase_a::has_valid_status;
    auto status = valid_status();
    check(has_valid_status(status), "complete status accepted");
    check(has_valid_position(status), "global position accepted");
    check(has_valid_gps(status), "3D GPS status accepted");
    check(has_valid_battery(status), "battery status accepted");
    status.latitude_deg = 91.0;
    check(!has_valid_status(status), "latitude range checked");
    check(!has_valid_position(status), "latitude range checked independently");
    status = valid_status();
    status.gps_fix_type = 2;
    check(!has_valid_status(status), "3D GPS fix required");
    check(!has_valid_gps(status), "GPS validity is available independently");
    status = valid_status();
    status.battery_voltage_v = NAN;
    check(!has_valid_status(status), "finite battery required");
    check(!has_valid_battery(status), "battery validity is available independently");
    status = valid_status();
    status.flight_mode = 0;
    check(!has_valid_status(status), "known flight mode required");
    status = valid_status();
    status.absolute_altitude_m = NAN;
    check(!has_valid_position(status), "finite absolute altitude required");
}

void test_freshness() {
    using nomad::mavsdk_phase_a::has_fresh_telemetry;
    using nomad::mavsdk_phase_a::has_fresh_position_stream;
    check(has_fresh_position_stream(3, 1000, 0), "sustained fresh stream accepted");
    check(!has_fresh_position_stream(2, 1000, 0), "multiple updates required");
    check(!has_fresh_position_stream(3, 999, 0), "observation duration required");
    check(!has_fresh_position_stream(3, 1000, 1501), "stale sample rejected");
    check(!has_fresh_position_stream(3, 1000, -1), "negative age rejected");
    check(has_fresh_telemetry(1500), "fresh field sample accepted at the age limit");
    check(!has_fresh_telemetry(1501), "stale field sample rejected");
    check(!has_fresh_telemetry(-1), "future field sample rejected");
    check(!has_fresh_telemetry(0, -1), "negative maximum age rejected");
}

} // namespace

int main() {
    test_endpoints();
    test_system_identity();
    test_status_validation();
    test_freshness();
    if (failures == 0) {
        std::cout << "MAVSDK validation tests passed\n";
        return EXIT_SUCCESS;
    }
    return EXIT_FAILURE;
}
