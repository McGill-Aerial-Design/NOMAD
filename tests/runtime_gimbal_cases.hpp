// SPDX-License-Identifier: Apache-2.0
#pragma once

Json gimbal_target_request(std::string id, double pitch_deg, double roll_deg,
                           std::string client = "test-client") {
    auto request = base_request(std::move(id), "set_gimbal_target", client);
    request["pitch_deg"] = pitch_deg;
    request["roll_deg"] = roll_deg;
    bind_authority(request);
    request["command_source"] = client;
    return request;
}

Json gimbal_config_target_request(std::string id, int mount_mode, double pitch_deg, double roll_deg,
                                  std::string client = "test-client") {
    auto request = base_request(std::move(id), "configure_gimbal_target", client);
    request["mount_mode"] = mount_mode;
    request["pitch_deg"] = pitch_deg;
    request["roll_deg"] = roll_deg;
    bind_authority(request);
    request["command_source"] = client;
    return request;
}

void test_gimbal_target_request_validation(Client &client, FakeConnection &connection, const Json &hello) {
    CHECK(std::find(hello["capabilities"].begin(), hello["capabilities"].end(), "set_gimbal_target") !=
          hello["capabilities"].end());
    read_authority(client);
    const auto denied = client.request(gimbal_target_request("startup-gimbal-denied", 0.0, 0.0));
    CHECK(denied["error"]["code"] == "not_authoritative");
    CHECK(connection.command_count() == 0);

    auto missing_pitch = base_request("missing-gimbal-pitch", "set_gimbal_target");
    missing_pitch["roll_deg"] = 0.0;
    CHECK(client.request(missing_pitch)["error"]["code"] == "invalid_request");
    auto string_pitch = base_request("string-gimbal-pitch", "set_gimbal_target");
    string_pitch["pitch_deg"] = "0";
    string_pitch["roll_deg"] = 0.0;
    CHECK(client.request(string_pitch)["error"]["code"] == "invalid_request");

    CHECK(std::find(hello["capabilities"].begin(), hello["capabilities"].end(), "configure_gimbal_target") !=
          hello["capabilities"].end());
    const auto combined_denied = client.request(gimbal_config_target_request("startup-gimbal-pair-denied", 2, 0, 0));
    CHECK(combined_denied["error"]["code"] == "not_authoritative");
    CHECK(connection.command_count() == 0);

    auto missing_mode = base_request("missing-gimbal-pair-mode", "configure_gimbal_target");
    missing_mode["pitch_deg"] = 0.0;
    missing_mode["roll_deg"] = 0.0;
    CHECK(client.request(missing_mode)["error"]["code"] == "invalid_request");
}

void test_gimbal_config_target_dispatch_and_dedupe(std::uint16_t port, FakeConnection &connection);

void test_gimbal_target_dispatch_and_dedupe(std::uint16_t port, FakeConnection &connection) {
    connection.acknowledgement = nomad::mavlink::CommandAck{205, 0};
    Client client(port);
    const auto before = connection.command_count();
    const auto request = gimbal_target_request("gimbal-target-1", -20.5, 12.25);
    const auto result = client.request(request);
    CHECK(result["command_result"]["success"] == true);
    CHECK(connection.command_count() == before + 1);
    CHECK(connection.last_command.id == 205);
    CHECK(connection.last_command.parameters[0] == -20.5F);
    CHECK(connection.last_command.parameters[1] == 12.25F);
    CHECK(connection.last_command.parameters[2] == 0.0F);
    CHECK(connection.last_command.parameters[6] == 2.0F);

    CHECK(client.request(request) == result);
    CHECK(connection.command_count() == before + 1);
    const auto reused_id = gimbal_target_request("gimbal-target-1", 10.0, 5.0);
    CHECK(client.request(reused_id)["error"]["code"] == "request_id_conflict");
    CHECK(connection.command_count() == before + 1);

    const auto invalid = client.request(gimbal_target_request("gimbal-target-out-of-range", 90.01, 0.0));
    CHECK(invalid["command_result"]["success"] == false);
    CHECK(connection.command_count() == before + 1);

    connection.acknowledgement = nomad::mavlink::CommandAck{205, 4};
    const auto rejected = client.request(gimbal_target_request("gimbal-target-rejected", 10.0, 5.0));
    CHECK(rejected["command_result"]["success"] == false);
    CHECK(rejected["command_result"]["message"].get<std::string>().find("rejected") != std::string::npos);
    CHECK(connection.command_count() == before + 2);

    connection.acknowledgement = nomad::mavlink::CommandAck{183, 0};
    test_gimbal_config_target_dispatch_and_dedupe(port, connection);
}

void test_gimbal_config_target_dispatch_and_dedupe(std::uint16_t port, FakeConnection &connection) {
    Client client(port);
    const auto before = connection.command_count();
    const auto request = gimbal_config_target_request("gimbal-config-target-1", 2, -20.5, 12.25);
    const auto result = client.request(request);
    CHECK(result["command_result"]["success"] == true);
    CHECK(connection.command_count() == before + 2);
    CHECK(connection.command_history[before].id == 204);
    CHECK(connection.command_history[before + 1].id == 205);
    CHECK(connection.command_history[before + 1].parameters[0] == -20.5F);
    CHECK(connection.command_history[before + 1].parameters[1] == 12.25F);

    CHECK(client.request(request) == result);
    CHECK(connection.command_count() == before + 2);
    const auto reused_id = gimbal_config_target_request("gimbal-config-target-1", 2, 10.0, 5.0);
    CHECK(client.request(reused_id)["error"]["code"] == "request_id_conflict");
    CHECK(connection.command_count() == before + 2);

    const auto invalid_target = gimbal_config_target_request("gimbal-config-target-invalid", 2, 90.01, 0.0);
    CHECK(client.request(invalid_target)["command_result"]["success"] == false);
    CHECK(connection.command_count() == before + 2);

    connection.acknowledgement = nomad::mavlink::CommandAck{183, 4};
    const auto rejected_mode = gimbal_config_target_request("gimbal-config-target-mode-rejected", 2, 10.0, 5.0);
    const auto rejected_result = client.request(rejected_mode);
    CHECK(rejected_result["command_result"]["success"] == false);
    CHECK(connection.command_count() == before + 3);
    CHECK(connection.command_history[before + 2].id == 204);
    connection.acknowledgement = nomad::mavlink::CommandAck{183, 0};
}
