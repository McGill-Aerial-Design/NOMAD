// SPDX-License-Identifier: Apache-2.0
#pragma once

void test_disconnect_does_not_cancel_or_replay(std::uint16_t port, FakeConnection &connection) {
    Client disconnected(port);
    const auto request = servo_request("lost-response", 1700);
    disconnected.send(request);
    disconnected.disconnect();
    wait_until([&connection] { return connection.command_count() == 3; });

    Client reconnect(port);
    CHECK(reconnect.request(request)["command_result"]["success"] == true);
    CHECK(connection.command_count() == 3);
}

void test_evicted_replay_and_wrong_source(std::uint16_t port, FakeConnection &connection) {
    Client client(port);
    const auto first = servo_request("eviction-first", 1500);
    CHECK(client.request(first)["command_result"]["success"] == true);
    for (int index = 0; index < 257; ++index) {
        CHECK(client.request(servo_request("eviction-" + std::to_string(index), 1500))
                  ["command_result"]["success"] == true);
    }
    const auto before = connection.command_count();
    CHECK(client.request(first)["error"]["code"] == "stale_request");
    CHECK(connection.command_count() == before);
    CHECK(client.request(servo_request("wrong-source", 1500, "other-client"))["error"]["code"] ==
          "not_authoritative");
    CHECK(connection.command_count() == before);
}

void test_revoke_and_handback(std::uint16_t port, FakeConnection &connection) {
    Client client(port);
    const auto old = servo_request("queued-old", 1500);
    const auto old_generation = authority.generation;
    const auto revoked = client.request(authority_request("revoke", "revoke_authority", "operator"));
    CHECK(revoked["ok"] == true);
    authority.generation = revoked["authority_generation"].get<std::uint64_t>();
    const auto before = connection.command_count();
    CHECK(client.request(old)["error"]["code"] == "stale_authority");
    CHECK(connection.command_count() == before);
    CHECK(client.request(servo_request("reconnect-no-reclaim", 1500))["error"]["code"] ==
          "not_authoritative");
    CHECK(connection.command_count() == before);
    const auto handback = client.request(authority_request("handback", "handback_authority"));
    CHECK(handback["ok"] == true);
    authority.generation = handback["authority_generation"].get<std::uint64_t>();
    CHECK(authority.generation > old_generation);
    CHECK(client.request(old)["error"]["code"] == "stale_authority");
    CHECK(client.request(servo_request("after-handback", 1500))["command_result"]["success"] == true);
    CHECK(connection.command_count() == before + 1);
}

void test_expired_and_delayed_request(std::uint16_t port, FakeConnection &connection) {
    Client client(port);
    auto expired = servo_request("expired", 1500);
    expired["expires_at_ms"] = 1;
    const auto before = connection.command_count();
    CHECK(client.request(expired)["error"]["code"] == "expired_request");
    CHECK(connection.command_count() == before);
}

void test_revoke_during_operation(std::uint16_t port, FakeConnection &connection) {
    connection.command_started = false;
    connection.command_delay = std::chrono::milliseconds(300);
    Client first(port);
    const auto before = connection.command_count();
    first.send(servo_request("in-flight-revoke", 1500));
    wait_until([&connection] { return connection.command_started.load(); });
    Client operator_client(port);
    const auto revoked = operator_client.request(authority_request("during-revoke", "revoke_authority", "operator"));
    CHECK(revoked["ok"] == true);
    authority.generation = revoked["authority_generation"].get<std::uint64_t>();
    CHECK(first.receive()["error"]["code"] == "authority_interrupted");
    CHECK(connection.command_count() == before);
    CHECK(operator_client.request(servo_request("after-revoke", 1500))["error"]["code"] ==
          "not_authoritative");
    CHECK(connection.command_count() == before);
    connection.command_delay = std::chrono::milliseconds(0);
}

void test_reconnect_is_observation_only(std::uint16_t port, FakeConnection &connection) {
    connection.set_connected(false);
    Client client(port);
    wait_until([&client] {
        return client.request(base_request("observe-loss", "status"))["status"]["authority_owner"].is_null();
    });
    connection.set_connected(true);
    read_authority(client);
    const auto before = connection.command_count();
    CHECK(client.request(servo_request("reconnected-old-owner", 1500))["error"]["code"] ==
          "not_authoritative");
    CHECK(connection.command_count() == before);
    const auto handback = client.request(authority_request("reconnected-handback", "handback_authority"));
    CHECK(handback["ok"] == true);
    authority.generation = handback["authority_generation"].get<std::uint64_t>();
}
