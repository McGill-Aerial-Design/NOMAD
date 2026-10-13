// SPDX-License-Identifier: Apache-2.0

void test_supported_flight_operations(Client &client, FakeConnection &connection) {
    using nomad::telemetry::AircraftClass;
    using nomad::telemetry::identify_vehicle;
    const std::array identities{
        nomad::telemetry::VehicleIdentity{}, identify_vehicle(3, 2), identify_vehicle(3, 1),
        identify_vehicle(3, 21),
        // A Plane heartbeat can resolve as QuadPlane through Q_ENABLE.
        nomad::telemetry::VehicleIdentity{3, 1, AircraftClass::QuadPlane}, identify_vehicle(12, 2),
    };
    const std::array expected_operations{
        Json(nullptr), Json::array({"land"}), Json::array(), Json::array(), Json::array(), Json(nullptr),
    };
    const auto commands_before = connection.command_count();
    for (std::size_t index = 0; index < identities.size(); ++index) {
        auto state = connection.get_state();
        state.identity = identities[index];
        connection.set_state(state);

        const auto status = client.request(base_request("flight-operation-support", "status"))["status"];

        CHECK(status["supported_flight_operations"] == expected_operations[index]);
        CHECK(status["authority_owner"].is_null());
    }
    const auto hello = client.request(base_request("flight-operation-verbs", "hello"));
    CHECK(std::find(hello["capabilities"].begin(), hello["capabilities"].end(), "land") !=
          hello["capabilities"].end());
    CHECK(connection.command_count() == commands_before);
}

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
        state.identity = nomad::telemetry::identify_vehicle(3, 2);
        state.connected = index == 0;
        state.heartbeat_updated_at = sampled_at;
        state.heartbeat_fresh = index == 0;
        state.vtol_state = samples[index].first;
        state.vtol_state_valid = true;
        state.vtol_state_updated_at = sampled_at - std::chrono::seconds(2);
        state.landed_state = landed_samples[index].first;
        state.landed_state_valid = true;
        state.landed_state_updated_at = sampled_at - std::chrono::seconds(4);
        connection.set_state(state);
        const auto status = client.request(base_request("observed-flight-state", "status"))["status"];
        const auto &telemetry = status["telemetry"];
        CHECK(status["supported_flight_operations"] == Json::array({"land"}));
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

void test_missing_position_status(Client &client, FakeConnection &connection) {
    auto state = connection.get_state();
    state.position = {45.5, -73.5, 125.0F, 25.0F};
    state.position_valid = false;
    state.position_updated_at = {};
    connection.set_state(state);

    const auto telemetry = client.request(base_request("missing-position", "status"))["status"]["telemetry"];
    CHECK(telemetry.contains("position"));
    CHECK(telemetry["position"].is_null());
    CHECK(telemetry["position_valid"] == false);
    CHECK(telemetry["position_age_ms"].is_null());
}

void test_observed_position_status(Client &client, FakeConnection &connection) {
    const std::array positions{
        nomad::telemetry::Position{0.0, 0.0, 0.0F, 0.0F},
        nomad::telemetry::Position{45.5, -73.5, 125.0F, 25.0F},
        nomad::telemetry::Position{-90.0, -180.0, -100.0F, -10.0F},
        nomad::telemetry::Position{90.0, 180.0, 1000.0F, 500.0F},
    };
    const std::array expected_positions{
        Json{{"latitude_deg", 0.0}, {"longitude_deg", 0.0},
             {"absolute_altitude_m", 0.0}, {"relative_altitude_m", 0.0}},
        Json{{"latitude_deg", 45.5}, {"longitude_deg", -73.5},
             {"absolute_altitude_m", 125.0}, {"relative_altitude_m", 25.0}},
        Json{{"latitude_deg", -90.0}, {"longitude_deg", -180.0},
             {"absolute_altitude_m", -100.0}, {"relative_altitude_m", -10.0}},
        Json{{"latitude_deg", 90.0}, {"longitude_deg", 180.0},
             {"absolute_altitude_m", 1000.0}, {"relative_altitude_m", 500.0}},
    };
    for (std::size_t index = 0; index < positions.size(); ++index) {
        auto state = connection.get_state();
        const auto age = index == 0 ? std::chrono::seconds(0) : std::chrono::seconds(30);
        state.position = positions[index];
        state.position_valid = true;
        state.position_updated_at = std::chrono::steady_clock::now() - age;
        // A fresh heartbeat must not hide an independently stale position.
        state.connected = index <= 1;
        state.heartbeat_fresh = index <= 1;
        state.heartbeat_updated_at = std::chrono::steady_clock::now() -
            (state.heartbeat_fresh ? std::chrono::seconds(0) : age);
        connection.set_state(state);

        const auto telemetry = client.request(base_request("observed-position", "status"))["status"]["telemetry"];
        CHECK(telemetry["position"] == expected_positions[index]);
        CHECK(telemetry["position_valid"] == true);
        CHECK(telemetry["heartbeat_fresh"] == (index <= 1));
        check_flight_sample_age(telemetry["heartbeat_age_ms"], state.heartbeat_updated_at,
                               state.heartbeat_fresh ? 0 : age.count() * 1000);
        check_flight_sample_age(telemetry["position_age_ms"], state.position_updated_at, age.count() * 1000);
    }
}

void test_invalid_position_status(Client &client, FakeConnection &connection) {
    auto state = connection.get_state();
    state.position = {91.0, -181.0, 125.0F, 25.0F};
    state.position_valid = false;
    state.position_updated_at = std::chrono::steady_clock::now();
    connection.set_state(state);

    const auto invalid = client.request(base_request("invalid-position", "status"))["status"]["telemetry"];
    const Json expected{{"latitude_deg", 91.0}, {"longitude_deg", -181.0},
                        {"absolute_altitude_m", 125.0}, {"relative_altitude_m", 25.0}};
    CHECK(invalid["position"] == expected);
    CHECK(invalid["position_valid"] == false);
    CHECK(invalid["position_age_ms"].is_number_integer());

    // The transport's sample flag does not guarantee finite numeric values.
    state.position = {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity(),
                      -std::numeric_limits<float>::infinity(), std::numeric_limits<float>::quiet_NaN()};
    state.position_valid = true;
    connection.set_state(state);
    const auto nonfinite = client.request(base_request("nonfinite-position", "status"))["status"]["telemetry"];
    const Json unavailable{{"latitude_deg", nullptr}, {"longitude_deg", nullptr},
                           {"absolute_altitude_m", nullptr}, {"relative_altitude_m", nullptr}};
    CHECK(nonfinite["position"] == unavailable);
    CHECK(nonfinite["position_valid"] == true);
    CHECK(nonfinite["position_age_ms"].is_number_integer());

    state.position.longitude_deg = -73.5;
    state.position.relative_altitude_m = 25.0F;
    connection.set_state(state);
    const auto partial = client.request(base_request("partial-position", "status"))["status"]["telemetry"];
    CHECK(partial["position"]["latitude_deg"].is_null());
    CHECK(partial["position"]["longitude_deg"] == -73.5);
    CHECK(partial["position"]["absolute_altitude_m"].is_null());
    CHECK(partial["position"]["relative_altitude_m"] == 25.0);
}

void test_flight_status_observations(std::uint16_t port, FakeConnection &connection) {
    const auto original = connection.get_state();
    const auto commands_before = connection.command_count();
    Client client(port);
    test_supported_flight_operations(client, connection);
    connection.set_state(original);
    test_missing_flight_status(client, connection);
    test_observed_flight_status(client, connection);
    test_invalid_flight_status(client, connection);
    test_missing_position_status(client, connection);
    test_observed_position_status(client, connection);
    test_invalid_position_status(client, connection);
    CHECK(connection.command_count() == commands_before);
    connection.set_state(original);
}
