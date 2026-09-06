// SPDX-License-Identifier: Apache-2.0
// Golden-frame codec tests: every reference frame here was produced with
// pymavlink 2.4.49 (ardupilotmega dialect), so the NOMAD wire format matches
// what MAVLink and ArduPilot expect. Keep the frames in sync with any codec
// change. Split from core_test.cpp to keep files under the 500-line policy.
#include "nomad/mavlink/protocol.hpp"

#include <chrono>
#include <cstdint>
#include <cstdio>
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

void run_codec_golden_tests();

// Golden frames below were produced with pymavlink 2.4.49 (ardupilotmega
// dialect) so the fence mission handshake matches the MAVLink wire format
// ArduPilot expects. Keep them in sync with any codec change.

// The GCS heartbeat is the frame that opens heartbeat-gated relay legs, so a
// single wrong byte (type, autopilot, or crc_extra) silently leaves the link
// dead. Pin the exact wire bytes: MAVLink2, sysid 255, compid 190, len 9
// payload = MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID, base_mode 0, custom_mode 0,
// MAV_STATE_ACTIVE, MAVLink version 3, CRC extra 50. Frame produced with
// pymavlink 2.4.49 (ardupilotmega, MAVLink_heartbeat_message.pack, v2).
void test_gcs_heartbeat_encoder_matches_mavlink_reference() {
    const std::vector<std::uint8_t> expected{
        0xfd, 0x09, 0x00, 0x00, 0x00, 0xff, 0xbe, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x06, 0x08, 0x00, 0x04, 0x03, 0x3d, 0x48};

    const auto frame = nomad::mavlink::encode_gcs_heartbeat(0, 255, 190);

    CHECK(frame == expected);
}

void test_gcs_heartbeat_frame_round_trips_through_decoder() {
    const auto frame = nomad::mavlink::encode_gcs_heartbeat(7, 255, 190);

    const auto message = nomad::mavlink::decode_message(frame);

    CHECK(message.has_value());
    CHECK(message->system_id == 255);
    CHECK(message->component_id == 190);
    CHECK(message->message_id == 0);
    CHECK(message->payload.size() >= 9);
    CHECK(message->payload[4] == 6); // MAV_TYPE_GCS
    CHECK(message->payload[5] == 8); // MAV_AUTOPILOT_INVALID
}

void test_mission_count_encoder_matches_mavlink_reference() {
    const std::vector<std::uint8_t> expected{
        0xfd, 0x05, 0x00, 0x00, 0x00, 0xff, 0x00, 0x2c, 0x00, 0x00, 0x04, 0x00, 0x01, 0x01, 0x01, 0x11, 0x86};

    const auto frame = nomad::mavlink::encode_mission_count(0, 255, 0, 1, 1, 4, 1);

    CHECK(frame == expected);
}

void test_mission_item_int_encoder_matches_mavlink_reference() {
    const std::vector<std::uint8_t> expected{
        0xfd, 0x26, 0x00, 0x00, 0x00, 0xff, 0x00, 0x49, 0x00, 0x00, 0x00, 0x00, 0x80, 0x40, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x59, 0x73, 0x07, 0x60, 0xdd,
        0x95, 0xeb, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x89, 0x13, 0x01, 0x01, 0x05, 0x00, 0x00, 0x01,
        0x3e, 0x21};
    const nomad::mavlink::FencePlanItem item{{12.5F, -34.25F}, 3, 5001, 4.0F};

    const auto frame = nomad::mavlink::encode_mission_item_int(0, 255, 0, 1, 1, item, 1);

    CHECK(frame == expected);
}

void test_mission_request_frames_match_mavlink_reference() {
    const std::vector<std::uint8_t> expected_request_int{
        0xfd, 0x05, 0x00, 0x00, 0x00, 0xff, 0x00, 0x33, 0x00, 0x00, 0x07, 0x00, 0x01, 0x01, 0x01, 0x90, 0x6e};
    const std::vector<std::uint8_t> expected_request_list{
        0xfd, 0x03, 0x00, 0x00, 0x00, 0xff, 0x00, 0x2b, 0x00, 0x00, 0x01, 0x01, 0x01, 0x40, 0x0a};

    const auto request_int = nomad::mavlink::encode_mission_request_int(0, 255, 0, 1, 1, 7, 1);
    const auto request_list = nomad::mavlink::encode_mission_request_list(0, 255, 0, 1, 1, 1);

    CHECK(request_int == expected_request_int);
    CHECK(request_list == expected_request_list);
}

void test_command_int_encoder_matches_mavlink_reference() {
    // Reference: MAVLink2 COMMAND_INT (id 75) with frame
    // GLOBAL_RELATIVE_ALT_INT (6), x/y as float32(lat)*1e7 truncated toward
    // zero (matching pymavlink int(x * 1e7) semantics on the float parameter),
    // z=5.0f, seq 0, src 255/190, tgt 1/1, current/autocontinue 0, CRC over
    // the full header+payload span with crc_extra 158 (pymavlink 2.4.49
    // field order + canonical x25crc). Two earlier versions were rejected by
    // Copter 4.7.0: crc_extra 24 (GPS_RAW_INT's value) made MAVProxy drop the
    // frame, and frame 2 (MAV_FRAME_MISSION) was answered
    // "Unknown mavlink coordinate frame" with MAV_RESULT_DENIED.
    const std::vector<std::uint8_t> expected{
        0xfd, 0x23, 0x00, 0x00, 0x00, 0xff, 0xbe, 0x4b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x80, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x9a, 0x2b, 0x44, 0x19, 0xf2, 0xbc,
        0x97, 0xd5, 0x00, 0x00, 0xa0, 0x40, 0xc0, 0x00, 0x01, 0x01, 0x06, 0x00, 0x00, 0x99, 0x1a};
    const nomad::mavlink::Command command{192, {0, 1, 0, 0, 42.3898F, -71.1476F, 5.0F}};

    const auto frame = nomad::mavlink::encode_command_int(0, 255, 190, 1, 1, command);

    CHECK(frame == expected);
}

void test_mission_handshake_decoders_read_reference_frames() {
    const std::vector<std::uint8_t> request{
        0xfd, 0x05, 0x00, 0x00, 0x00, 0x01, 0x01, 0x28, 0x00, 0x00, 0x07, 0x00, 0xff, 0x00, 0x01, 0x47, 0x10};
    const std::vector<std::uint8_t> request_int{
        0xfd, 0x05, 0x00, 0x00, 0x00, 0x01, 0x01, 0x33, 0x00, 0x00, 0x07, 0x00, 0xff, 0x00, 0x01, 0x0e, 0x44};
    const std::vector<std::uint8_t> count{
        0xfd, 0x05, 0x00, 0x00, 0x00, 0x01, 0x01, 0x2c, 0x00, 0x00, 0x04, 0x00, 0xff, 0x00, 0x01, 0x8f, 0xac};
    const std::vector<std::uint8_t> ack_ok{
        0xfd, 0x04, 0x00, 0x00, 0x00, 0x01, 0x01, 0x2f, 0x00, 0x00, 0xff, 0x00, 0x00, 0x01, 0x7e, 0x26};
    const std::vector<std::uint8_t> ack_cancelled{
        0xfd, 0x04, 0x00, 0x00, 0x00, 0x01, 0x01, 0x2f, 0x00, 0x00, 0xff, 0x00, 0x05, 0x01, 0xc3, 0x1f};

    const auto request_message = nomad::mavlink::decode_message(request);
    const auto request_int_message = nomad::mavlink::decode_message(request_int);
    CHECK(request_message.has_value());
    CHECK(request_int_message.has_value());
    CHECK(request_message->system_id == 1);
    CHECK(nomad::mavlink::decode_mission_sequence(*request_message).value_or(999) == 7);
    CHECK(nomad::mavlink::decode_mission_sequence(*request_int_message).value_or(999) == 7);

    const auto count_message = nomad::mavlink::decode_message(count);
    CHECK(count_message.has_value());
    CHECK(nomad::mavlink::decode_mission_count(*count_message).value_or(999) == 4);

    const auto ok_message = nomad::mavlink::decode_message(ack_ok);
    const auto cancelled_message = nomad::mavlink::decode_message(ack_cancelled);
    CHECK(ok_message.has_value());
    CHECK(cancelled_message.has_value());
    CHECK(nomad::mavlink::decode_mission_ack(*ok_message).value_or(0xff) == 0);
    CHECK(nomad::mavlink::decode_mission_ack(*cancelled_message).value_or(0xff) == 5);
}


void run_codec_golden_tests() {
    test_gcs_heartbeat_encoder_matches_mavlink_reference();
    test_gcs_heartbeat_frame_round_trips_through_decoder();
    test_mission_count_encoder_matches_mavlink_reference();
    test_mission_item_int_encoder_matches_mavlink_reference();
    test_mission_request_frames_match_mavlink_reference();
    test_command_int_encoder_matches_mavlink_reference();
    test_mission_handshake_decoders_read_reference_frames();
}

} // namespace

int main() {
    try {
        run_codec_golden_tests();
    } catch (const std::exception &error) {
        std::fprintf(stderr, "FAILED: %s\n", error.what());
        return 1;
    }
    return 0;
}
