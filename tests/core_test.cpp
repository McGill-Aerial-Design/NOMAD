// SPDX-License-Identifier: Apache-2.0
#include "fake_connection.hpp"
#include "nomad/mavlink/protocol.hpp"
#include "nomad/mission/executor.hpp"
#include "nomad/vehicle/vehicle.hpp"

#ifdef NDEBUG
#undef NDEBUG
#endif

#include <cassert>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

// A failing assert on Windows opens a dialog that blocks unattended CI runs,
// so main() runs the tests inside a try/catch and reports failures on stderr.
void check_impl(bool ok, const char *condition, int line) {
    if (!ok) {
        throw std::runtime_error(std::string("check failed at line ") + std::to_string(line) + ": " + condition);
    }
}

#define CHECK(condition) check_impl(static_cast<bool>(condition), #condition, __LINE__)

void test_state_is_available_through_vehicle() {
    FakeConnection connection;
    connection.connect();
    nomad::vehicle::Vehicle vehicle(connection);

    const auto state = vehicle.wait_for_state(std::chrono::seconds(1));

    CHECK(state.has_value());
    CHECK(state->connected);
    CHECK(state->system_id == 1);
}

void test_state_requires_connection() {
    FakeConnection connection;
    nomad::vehicle::Vehicle vehicle(connection);

    const auto state = vehicle.wait_for_state(std::chrono::seconds(1));

    CHECK(!state.has_value());
}

void test_arm_sends_arm_command() {
    FakeConnection connection;
    connection.connect();
    nomad::vehicle::Vehicle vehicle(connection);
    connection.acknowledgement = nomad::mavlink::CommandAck{400, 0};

    const auto result = vehicle.arm();

    CHECK(result.success);
    CHECK(result.message == "arm verified");
    CHECK(connection.last_command.id == 400);
    CHECK(connection.last_command.parameters[0] == 1.0F);
}

void test_takeoff_rejects_invalid_altitude() {
    FakeConnection connection;
    connection.connect();
    nomad::vehicle::Vehicle vehicle(connection);

    const auto result = vehicle.takeoff(0.0F);

    CHECK(!result.success);
    CHECK(connection.last_command.id == 0);
}

void test_mode_and_takeoff_are_verified() {
    FakeConnection connection;
    connection.connect();
    nomad::vehicle::Vehicle vehicle(connection);
    connection.acknowledgement = nomad::mavlink::CommandAck{176, 0};

    const auto mode_result = vehicle.set_mode(4);
    CHECK(mode_result.success);
    CHECK(mode_result.message == "set mode verified");

    connection.acknowledgement = nomad::mavlink::CommandAck{22, 0};
    const auto takeoff_result = vehicle.takeoff(10.0F);
    CHECK(takeoff_result.success);
    CHECK(takeoff_result.message == "takeoff verified");
}

void test_land_and_rtl_are_verified() {
    FakeConnection connection;
    connection.connect();
    nomad::vehicle::Vehicle vehicle(connection);

    connection.acknowledgement = nomad::mavlink::CommandAck{21, 0};
    CHECK(vehicle.land().success);
    connection.acknowledgement = nomad::mavlink::CommandAck{20, 0};
    CHECK(vehicle.return_to_launch().success);
}

void test_disarm_is_verified() {
    FakeConnection connection;
    connection.connect();
    connection.state->armed = true;
    nomad::vehicle::Vehicle vehicle(connection);
    connection.acknowledgement = nomad::mavlink::CommandAck{400, 0};

    const auto result = vehicle.disarm();

    CHECK(result.success);
    CHECK(!connection.state->armed);
}

void test_goto_location_validates_and_verifies() {
    FakeConnection connection;
    connection.connect();
    nomad::vehicle::Vehicle vehicle(connection);
    connection.acknowledgement = nomad::mavlink::CommandAck{192, 0};

    const auto result = vehicle.goto_location({45.5, -73.6, 10.0F});

    CHECK(result.success);
    CHECK(connection.last_command.id == 192);
    CHECK(connection.last_command.parameters[4] == 45.5F);
    CHECK(connection.last_command.parameters[5] == -73.6F);
    // ArduPilot only accepts MAV_CMD_DO_REPOSITION as command_int.
    CHECK(connection.last_command.use_command_int);
}

void test_goto_location_rejects_invalid_coordinates() {
    FakeConnection connection;
    connection.connect();
    nomad::vehicle::Vehicle vehicle(connection);
    connection.acknowledgement = nomad::mavlink::CommandAck{192, 0};

    const auto result = vehicle.goto_location({91.0, 0.0, 10.0F});

    CHECK(!result.success);
    CHECK(connection.last_command.id == 0);
}

void test_command_rejects_failed_acknowledgement() {
    FakeConnection connection;
    connection.connect();
    nomad::vehicle::Vehicle vehicle(connection);
    connection.acknowledgement = nomad::mavlink::CommandAck{21, 4};

    const auto result = vehicle.land();

    CHECK(!result.success);
}

void test_command_requires_connection() {
    FakeConnection connection;
    nomad::vehicle::Vehicle vehicle(connection);

    const auto result = vehicle.land();

    CHECK(!result.success);
}

void test_mission_executor_runs_steps_and_reports_progress() {
    FakeConnection connection;
    connection.connect();
    nomad::vehicle::Vehicle vehicle(connection);
    nomad::mission::MissionExecutor executor(vehicle);

    const nomad::mission::Mission mission{
        nomad::mission::Action{"guided"},
        nomad::mission::Action{"arm"},
        nomad::mission::Land{},
        nomad::mission::Action{"wait_disarmed"},
    };
    const auto result = executor.execute(mission);

    CHECK(result.success);
    CHECK(result.completed_steps == 4);
}

void test_mission_executor_rejects_unknown_action() {
    FakeConnection connection;
    connection.connect();
    nomad::vehicle::Vehicle vehicle(connection);
    nomad::mission::MissionExecutor executor(vehicle);

    const auto result = executor.execute({nomad::mission::Action{"unknown"}});

    CHECK(!result.success);
    CHECK(result.completed_steps == 0);
}

void test_command_frame_has_expected_header() {
    const auto frame =
        nomad::mavlink::encode_command_long(7, 255, 190, 1, 1, nomad::mavlink::Command{400, {1, 0, 0, 0, 0, 0, 0}});

    CHECK(frame.size() == 41);
    CHECK(frame[0] == 0xfe);
    CHECK(frame[1] == 33);
    CHECK(frame[2] == 7);
    CHECK(frame[3] == 255);
    CHECK(frame[4] == 190);
    CHECK(frame[5] == 76);
    CHECK(nomad::mavlink::decode_message(frame).has_value());
}

void test_velocity_frame_uses_expected_wire_layout() {
    const auto frame = nomad::mavlink::encode_velocity_setpoint(
        1, 255, 190, 1, 1, nomad::mavlink::VelocitySetpoint{1.0F, -2.0F, 0.5F, 0.25F});
    CHECK(frame.size() == 61);
    const auto message = nomad::mavlink::decode_message(frame);
    CHECK(message.has_value());
    CHECK(message->message_id == 84);
    CHECK(message->payload[48] == 0xc7);
    CHECK(message->payload[49] == 0x07);
    CHECK(message->payload[50] == 1);
    CHECK(message->payload[51] == 1);
    CHECK(message->payload[52] == 9);
}

void test_heartbeat_filter_accepts_vehicle_only() {
    const nomad::mavlink::Heartbeat vehicle{1, 1, 0, 2, 3, 0};
    const nomad::mavlink::Heartbeat gcs{255, 1, 0, 6, 3, 0};
    const nomad::mavlink::Heartbeat other_vehicle{2, 1, 0, 2, 3, 0};

    CHECK(nomad::mavlink::accepts_heartbeat(vehicle, 0));
    CHECK(nomad::mavlink::accepts_heartbeat(vehicle, 1));
    CHECK(!nomad::mavlink::accepts_heartbeat(gcs, 0));
    CHECK(!nomad::mavlink::accepts_heartbeat(other_vehicle, 1));
}

void test_v2_heartbeat_updates_state() {
    std::vector<std::uint8_t> payload(9, 0);
    payload[4] = 2;
    payload[5] = 3;
    payload[6] = 0x80;
    payload[8] = 3;
    const auto frame = nomad::mavlink::encode_message(1, 1, 1, 0, payload, true);
    CHECK(frame.has_value());

    const auto message = nomad::mavlink::decode_message(*frame);
    CHECK(message.has_value());
    nomad::telemetry::VehicleState state;
    CHECK(nomad::mavlink::update_vehicle_state(*message, state));
    CHECK(state.connected);
    CHECK(state.armed);
    CHECK(state.system_id == 1);
}

void test_global_position_updates_state() {
    std::vector<std::uint8_t> payload(28, 0);
    payload[4] = 0x00;
    payload[5] = 0x80;
    payload[8] = 0x00;
    payload[9] = 0x40;
    payload[12] = 0x40;
    payload[16] = 0x20;
    const auto frame = nomad::mavlink::encode_message(1, 1, 1, 33, payload, true);
    CHECK(frame.has_value());
    const auto message = nomad::mavlink::decode_message(*frame);
    CHECK(message.has_value());
    nomad::telemetry::VehicleState state;
    CHECK(nomad::mavlink::update_vehicle_state(*message, state));
    CHECK(state.position_valid);
    CHECK(state.position.latitude_deg > 0.0);
}

void test_vfr_hud_updates_groundspeed_and_climb_rate() {
    // Wire order from the ardupilotmega dialect: airspeed, groundspeed, alt,
    // climb, heading, throttle (alt at byte 8, climb at byte 12).
    std::vector<std::uint8_t> payload(20, 0);
    const float groundspeed = 4.5F;
    const float altitude = 12.0F;
    const float climb_rate = -0.5F;
    std::memcpy(payload.data() + 4, &groundspeed, sizeof(groundspeed));
    std::memcpy(payload.data() + 8, &altitude, sizeof(altitude));
    std::memcpy(payload.data() + 12, &climb_rate, sizeof(climb_rate));
    const auto frame = nomad::mavlink::encode_message(1, 1, 1, 74, payload, true);
    CHECK(frame.has_value());
    const auto message = nomad::mavlink::decode_message(*frame);
    CHECK(message.has_value());
    nomad::telemetry::VehicleState state;
    CHECK(nomad::mavlink::update_vehicle_state(*message, state));
    CHECK(state.velocity.groundspeed_mps == groundspeed);
    CHECK(state.position.altitude_m == altitude);
    CHECK(state.velocity.climb_rate_mps == climb_rate);
}

void test_trimmed_v2_payload_is_zero_padded() {
    // MAVLink2 senders drop trailing zero bytes; a level-flight GPI frame
    // with vz and hdg both zero arrives 24 bytes long. The decoder must pad
    // it back to 28 so the position still updates.
    std::vector<std::uint8_t> payload(28, 0);
    payload[0] = 1;
    payload[1] = 2;
    payload[2] = 3;
    payload[3] = 4;
    const auto lat = static_cast<std::int32_t>(45.0 * 1e7);
    std::memcpy(payload.data() + 4, &lat, sizeof(lat));
    const auto lon = static_cast<std::int32_t>(-73.0 * 1e7);
    std::memcpy(payload.data() + 8, &lon, sizeof(lon));
    const auto alt = static_cast<std::int32_t>(18570);
    std::memcpy(payload.data() + 12, &alt, sizeof(alt));
    const auto relative = static_cast<std::int32_t>(5000);
    std::memcpy(payload.data() + 16, &relative, sizeof(relative));
    payload.resize(24);  // drop the trailing zero vz and hdg bytes

    const auto frame = nomad::mavlink::encode_message(1, 1, 1, 33, payload, true);
    CHECK(frame.has_value());
    const auto message = nomad::mavlink::decode_message(*frame);
    CHECK(message.has_value());
    CHECK(message->payload.size() == 28);
    nomad::telemetry::VehicleState state;
    state.gps_valid = true;
    CHECK(nomad::mavlink::update_vehicle_state(*message, state));
    CHECK(state.position_valid);
    CHECK(state.position.latitude_deg == 45.0);
    CHECK(state.position.longitude_deg == -73.0);
    CHECK(state.position.relative_altitude_m == 5.0F);
}

void test_bad_crc_is_rejected() {
    const auto frame = nomad::mavlink::encode_message(1, 1, 1, 0, std::vector<std::uint8_t>(9), true);
    CHECK(frame.has_value());
    auto damaged = *frame;
    damaged.back() ^= 0xff;
    CHECK(!nomad::mavlink::decode_message(damaged).has_value());
}

void test_coalesced_datagram_decodes_every_frame() {
    // MAVProxy packs several MAVLink frames into one UDP datagram. Each call
    // must decode one frame and report how many bytes it consumed, so the
    // caller can decode the next frame of the same datagram.
    const auto heartbeat = nomad::mavlink::encode_message(1, 1, 1, 0, std::vector<std::uint8_t>(9), true);
    const auto telemetry = nomad::mavlink::encode_message(2, 1, 1, 30, std::vector<std::uint8_t>(28), true);
    const auto acknowledgement = nomad::mavlink::encode_message(3, 1, 1, 77, std::vector<std::uint8_t>(3), true);
    CHECK(heartbeat.has_value() && telemetry.has_value() && acknowledgement.has_value());

    std::vector<std::uint8_t> datagram;
    datagram.insert(datagram.end(), heartbeat->begin(), heartbeat->end());
    datagram.insert(datagram.end(), telemetry->begin(), telemetry->end());
    datagram.insert(datagram.end(), acknowledgement->begin(), acknowledgement->end());

    std::span<const std::uint8_t> remaining(datagram);
    std::size_t consumed = 0;
    const auto first = nomad::mavlink::decode_datagram(remaining, consumed);
    CHECK(first.has_value() && first->message_id == 0);
    remaining = remaining.subspan(consumed);
    const auto second = nomad::mavlink::decode_datagram(remaining, consumed);
    CHECK(second.has_value() && second->message_id == 30);
    remaining = remaining.subspan(consumed);
    const auto third = nomad::mavlink::decode_datagram(remaining, consumed);
    CHECK(third.has_value() && third->message_id == 77);
    remaining = remaining.subspan(consumed);
    CHECK(remaining.empty());
    CHECK(!nomad::mavlink::decode_datagram(remaining, consumed).has_value());
}

void test_gps_raw_int_updates_fix_type() {
    // ArduPilot packs the ardupilotmega GPS_RAW_INT layout: fix_type at byte
    // 28 (after vel/cog), satellites_visible at 29. The common.xml layout
    // (fix_type at 8) is what the live Copter 4.7.0 stream does not use.
    std::vector<std::uint8_t> payload(30, 0);
    payload[28] = 3;
    payload[29] = 12;
    const auto frame = nomad::mavlink::encode_message(1, 1, 1, 24, payload, true);
    CHECK(frame.has_value());
    const auto message = nomad::mavlink::decode_message(*frame);
    CHECK(message.has_value());
    nomad::telemetry::VehicleState state;
    CHECK(nomad::mavlink::update_vehicle_state(*message, state));
    CHECK(state.gps_valid);
    CHECK(state.gps.fix_type == 3);
    CHECK(state.gps.satellites == 12);
}

} // namespace

int main() {
    try {
    test_state_is_available_through_vehicle();
    test_state_requires_connection();
    test_arm_sends_arm_command();
    test_takeoff_rejects_invalid_altitude();
    test_mode_and_takeoff_are_verified();
    test_land_and_rtl_are_verified();
    test_disarm_is_verified();
    test_goto_location_validates_and_verifies();
    test_goto_location_rejects_invalid_coordinates();
    test_command_rejects_failed_acknowledgement();
    test_command_requires_connection();
    test_mission_executor_runs_steps_and_reports_progress();
    test_mission_executor_rejects_unknown_action();
    test_command_frame_has_expected_header();
    test_velocity_frame_uses_expected_wire_layout();
    test_heartbeat_filter_accepts_vehicle_only();
    test_v2_heartbeat_updates_state();
    test_global_position_updates_state();
    test_vfr_hud_updates_groundspeed_and_climb_rate();
    test_bad_crc_is_rejected();
    test_coalesced_datagram_decodes_every_frame();
    test_gps_raw_int_updates_fix_type();
    test_trimmed_v2_payload_is_zero_padded();
    } catch (const std::exception &error) {
        std::fprintf(stderr, "FAILED: %s\n", error.what());
        return 1;
    }
    return 0;
}
