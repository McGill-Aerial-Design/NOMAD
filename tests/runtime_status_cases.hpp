// SPDX-License-Identifier: Apache-2.0

void check_flight_sample_age(const Json &age, std::chrono::steady_clock::time_point sampled_at,
                            std::int64_t minimum_age_ms) {
    const auto maximum_age = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - sampled_at).count();
    CHECK(age.is_number_integer());
    CHECK(age.get<std::int64_t>() >= minimum_age_ms);
    CHECK(age.get<std::int64_t>() <= maximum_age);
}

void test_missing_flight_status(Client &client, FakeConnection &connection) {
    auto state = connection.get_state();
    state.heartbeat_updated_at = {};
    state.heartbeat_fresh = false;
    state.vtol_state = nomad::telemetry::VtolState::Undefined;
    state.vtol_state_valid = false;
    state.vtol_state_updated_at = {};
    state.landed_state = nomad::telemetry::LandedState::Unknown;
    state.landed_state_valid = false;
    state.landed_state_updated_at = {};
    connection.set_state(state);

    const auto telemetry = client.request(base_request("missing-flight-state", "status"))["status"]["telemetry"];
    CHECK(telemetry["heartbeat_fresh"] == false);
    CHECK(telemetry["heartbeat_age_ms"].is_null());
    CHECK(telemetry["vtol_state"] == "undefined");
    CHECK(telemetry["vtol_state_valid"] == false);
    CHECK(telemetry["vtol_state_age_ms"].is_null());
    CHECK(telemetry["landed_state"] == "unknown");
    CHECK(telemetry["landed_state_valid"] == false);
    CHECK(telemetry["landed_state_age_ms"].is_null());
}

void test_observed_flight_status(Client &client, FakeConnection &connection) {
    using nomad::telemetry::VtolState;
    using nomad::telemetry::LandedState;
    const std::array samples{
        std::pair{VtolState::TransitionToFixedWing, "transition_to_fixed_wing"},
        std::pair{VtolState::TransitionToMulticopter, "transition_to_multicopter"},
        std::pair{VtolState::Multicopter, "multicopter"},
        std::pair{VtolState::FixedWing, "fixed_wing"},
    };
    const std::array landed_samples{
        std::pair{LandedState::OnGround, "on_ground"}, std::pair{LandedState::InAir, "in_air"},
        std::pair{LandedState::TakingOff, "taking_off"}, std::pair{LandedState::Landing, "landing"},
    };
    for (std::size_t index = 0; index < samples.size(); ++index) {
        auto state = connection.get_state();
        const auto age = index == 0 ? std::chrono::seconds(0) : std::chrono::seconds(30);
        const auto sampled_at = std::chrono::steady_clock::now() - age;
        state.heartbeat_updated_at = sampled_at;
        state.heartbeat_fresh = index == 0;
        state.vtol_state = samples[index].first;
        state.vtol_state_valid = true;
        state.vtol_state_updated_at = sampled_at - std::chrono::seconds(2);
        state.landed_state = landed_samples[index].first;
        state.landed_state_valid = true;
        state.landed_state_updated_at = sampled_at - std::chrono::seconds(4);
        connection.set_state(state);
        const auto telemetry = client.request(base_request("observed-flight-state", "status"))["status"]["telemetry"];
        CHECK(telemetry["vtol_state"] == samples[index].second);
        CHECK(telemetry["landed_state"] == landed_samples[index].second);
        CHECK(telemetry["vtol_state_valid"] == true);
        CHECK(telemetry["landed_state_valid"] == true);
        CHECK(telemetry["heartbeat_fresh"] == (index == 0));
        check_flight_sample_age(telemetry["heartbeat_age_ms"], sampled_at, age.count() * 1000);
        check_flight_sample_age(telemetry["vtol_state_age_ms"], state.vtol_state_updated_at, age.count() * 1000 + 2000);
        check_flight_sample_age(telemetry["landed_state_age_ms"], state.landed_state_updated_at,
                               age.count() * 1000 + 4000);
    }
}

void test_invalid_flight_status(Client &client, FakeConnection &connection) {
    auto state = connection.get_state();
    state.vtol_state = nomad::telemetry::VtolState::Undefined;
    state.vtol_state_valid = false;
    state.vtol_state_updated_at = std::chrono::steady_clock::now();
    state.landed_state = nomad::telemetry::LandedState::Unknown;
    state.landed_state_valid = false;
    state.landed_state_updated_at = state.vtol_state_updated_at;
    connection.set_state(state);

    const auto telemetry = client.request(base_request("invalid-flight-state", "status"))["status"]["telemetry"];
    CHECK(telemetry["vtol_state"] == "undefined");
    CHECK(telemetry["vtol_state_valid"] == false);
    CHECK(telemetry["vtol_state_age_ms"].is_number_integer());
    CHECK(telemetry["landed_state"] == "unknown");
    CHECK(telemetry["landed_state_valid"] == false);
    CHECK(telemetry["landed_state_age_ms"].is_number_integer());
}

void test_flight_status_observations(std::uint16_t port, FakeConnection &connection) {
    const auto original = connection.get_state();
    Client client(port);
    test_missing_flight_status(client, connection);
    test_observed_flight_status(client, connection);
    test_invalid_flight_status(client, connection);
    connection.set_state(original);
}
