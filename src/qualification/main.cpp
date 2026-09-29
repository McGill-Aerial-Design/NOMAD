// SPDX-License-Identifier: Apache-2.0
// Non-installed test driver for direct SITL and MAVSDK transport qualification.
#include "cli_command_table.hpp"
#include "arguments.hpp"
#include "commands.hpp"
#include "nomad/mavlink/mavsdk_transport.hpp"
#include "runtime/cli_client.hpp"
#include "nomad/safety/fence_config.hpp"
#include "nomad/safety/velocity_config.hpp"
#include "nomad/vehicle/vehicle.hpp"

#include <chrono>
#include <cstddef>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace {

bool api_key_configured() {
    const char *key = std::getenv("NOMAD_API_KEY");
    return key != nullptr && key[0] != '\0';
}

std::optional<bool> integrated_flight_enabled() {
    const char *configured = std::getenv("NOMAD_INTEGRATED_FLIGHT");
    if (configured == nullptr || configured[0] == '\0') {
        return false;
    }

    std::string value(configured);
    for (auto &character : value) {
        if (character >= 'A' && character <= 'Z') {
            character = static_cast<char>(character - 'A' + 'a');
        }
    }
    if (value == "1" || value == "true" || value == "yes") {
        return true;
    }
    if (value == "0" || value == "false" || value == "no") {
        return false;
    }
    return std::nullopt;
}

void audit_command(std::string_view command, std::string_view result, std::string_view auth,
                   std::string_view reason) {
    std::cerr << "audit command=" << command << " result=" << result << " auth=" << auth;
    if (!reason.empty()) std::cerr << " reason=" << reason;
    std::cerr << '\n';
}

// The transport reports which half of the link failed, so an operator keeps the
// diagnostic it needs: a link that never opened is a connect failure, while an
// open link that no autopilot answered on is a heartbeat timeout.
void print_connect_failure(const nomad::mavlink::MavlinkConnection &connection, const std::string &endpoint) {
    if (connection.get_connect_failure() == nomad::mavlink::ConnectFailure::NoAutopilot) {
        std::cerr << "timed out waiting for ArduPilot heartbeat\n";
        return;
    }
    std::cerr << "could not connect to qualification MAVSDK endpoint " << endpoint
              << "; stop the NOMAD runtime before running direct qualification\n";
}

int run_command(nomad::mavlink::MavlinkConnection &connection, const Arguments &arguments,
                const std::string &endpoint) {
    if (is_actuation_command(arguments.command)) {
        if (runtime_endpoint_is_open()) {
            audit_command(arguments.command, "refused", "none", "runtime_owner_active");
            std::cerr << "error: direct actuation is inhibited while the NOMAD runtime is listening\n";
            return EXIT_FAILURE;
        }
        const auto integrated = integrated_flight_enabled();
        if (!integrated.has_value()) {
            audit_command(arguments.command, "refused", "none", "invalid_integrated_flight_setting");
            std::cerr << "error: NOMAD_INTEGRATED_FLIGHT must be a boolean value\n";
            return EXIT_FAILURE;
        }
        if (*integrated) {
            audit_command(arguments.command, "refused", "none", "runtime_owner_required");
            std::cerr << "error: direct actuation is inhibited in integrated flight mode\n";
            return EXIT_FAILURE;
        }
        if (!api_key_configured()) {
            audit_command(arguments.command, "refused", "none", "missing_api_key");
            std::cerr << "error: actuation command refused: NOMAD_API_KEY is not set\n";
            return EXIT_FAILURE;
        }
        audit_command(arguments.command, "accepted", "api-key", "");
    }

    if (arguments.command == "goto" && arguments.latitude.has_value() && arguments.longitude.has_value() &&
        arguments.altitude.has_value()) {
        // The NOMAD-side projected fence (SR-FEN-02) rejects an out-of-fence
        // target before any socket work; a malformed fence fails closed.
        const auto fence_policy = nomad::safety::load_fence_policy(std::getenv("NOMAD_FENCE_POLYGON"),
                                                                   std::getenv("NOMAD_FENCE_MARGIN_M"));
        const nomad::safety::GlobalPoint target{*arguments.latitude, *arguments.longitude};
        const auto decision = nomad::safety::evaluate_global_position(fence_policy, target);
        if (!decision.allowed) {
            std::cerr << "error: " << decision.message << '\n';
            return EXIT_FAILURE;
        }
    }

    if (!connection.connect()) {
        print_connect_failure(connection, endpoint);
        return EXIT_FAILURE;
    }

    if (arguments.command == "status") {
        return run_status(connection);
    }
    if (arguments.command == "connect") {
        const auto heartbeat = connection.wait_for_heartbeat(std::chrono::seconds(6));
        if (!heartbeat.has_value()) {
            std::cerr << "timed out waiting for ArduPilot heartbeat\n";
            return EXIT_FAILURE;
        }
        std::cout << "connected system=" << static_cast<int>(heartbeat->system_id)
                  << " component=" << static_cast<int>(heartbeat->component_id) << '\n';
        return EXIT_SUCCESS;
    }

    // Ensure the target system is known before sending any command. Commands
    // deliberately do not request high-rate telemetry streams: SITL's default
    // streams carry the heartbeats and state the command verification needs,
    // and a burst of stream requests can saturate a lossy link right before
    // the acknowledgement arrives.
    if (!connection.wait_for_heartbeat(std::chrono::seconds(3)).has_value()) {
        std::cerr << "timed out waiting for ArduPilot heartbeat\n";
        return EXIT_FAILURE;
    }

    // The NOMAD-side projected fence (SR-FEN-02) gates every position target
    // before transmission; a malformed configured fence fails closed.
    const auto fence_policy = nomad::safety::load_fence_policy(std::getenv("NOMAD_FENCE_POLYGON"),
                                                               std::getenv("NOMAD_FENCE_MARGIN_M"));
    const auto velocity_limits = nomad::safety::load_velocity_limits(
        std::getenv("NOMAD_VELOCITY_MAX_XY"), std::getenv("NOMAD_VELOCITY_MAX_Z"),
        std::getenv("NOMAD_VELOCITY_MAX_YAW_RATE"));
    nomad::vehicle::VehicleConfig vehicle_config{};
    vehicle_config.watchdog = {};
    vehicle_config.fence = fence_policy;
    vehicle_config.velocity = velocity_limits;
    nomad::vehicle::Vehicle vehicle(connection, vehicle_config);
    if (arguments.command == "arm") {
        return print_result(vehicle.arm());
    }
    if (arguments.command == "disarm") {
        return print_result(vehicle.disarm());
    }
    if (arguments.command == "mode" && arguments.mode.has_value()) {
        return print_result(vehicle.set_mode(*arguments.mode));
    }
    if (arguments.command == "takeoff" && arguments.altitude.has_value()) {
        return print_result(vehicle.takeoff(*arguments.altitude));
    }
    if (arguments.command == "vtol-takeoff" && arguments.altitude.has_value()) {
        return print_result(vehicle.vtol_takeoff(*arguments.altitude));
    }
    if (arguments.command == "transition-to-fixed-wing") {
        return print_result(vehicle.transition_to_fixed_wing());
    }
    if (arguments.command == "fixed-wing-route" && arguments.fixed_wing_route_values.size() == 6) {
        const auto &values = arguments.fixed_wing_route_values;
        const std::vector<nomad::vehicle::RouteWaypoint> route{
            {values[0], values[1], static_cast<float>(values[2])},
            {values[3], values[4], static_cast<float>(values[5])},
        };
        return print_result(vehicle.fixed_wing_route(route));
    }
    if (arguments.command == "fixed-wing-recovery" && arguments.fixed_wing_recovery_values.size() == 3) {
        const auto &values = arguments.fixed_wing_recovery_values;
        const nomad::vehicle::RecoveryPoint point{values[0], values[1], static_cast<float>(values[2])};
        return print_result(vehicle.fixed_wing_recovery(point));
    }
    if (arguments.command == "transition-to-vtol" && arguments.transition_to_vtol_values.size() == 3) {
        const auto &values = arguments.transition_to_vtol_values;
        const nomad::vehicle::RecoveryPoint point{values[0], values[1], static_cast<float>(values[2])};
        return print_result(vehicle.transition_to_vtol(point));
    }
    if (arguments.command == "quadplane-vtol-land" && arguments.quadplane_landing_values.size() == 2) {
        const auto &values = arguments.quadplane_landing_values;
        const nomad::vehicle::LandingPoint point{values[0], values[1]};
        return print_result(vehicle.quadplane_vtol_land(point));
    }
    if (arguments.command == "goto" && arguments.latitude.has_value() && arguments.longitude.has_value() &&
        arguments.altitude.has_value()) {
        const nomad::vehicle::Location target{*arguments.latitude, *arguments.longitude, *arguments.altitude};
        return print_result(vehicle.goto_location(target));
    }
    if (arguments.command == "land") {
        return print_result(vehicle.land());
    }
    if (arguments.command == "rtl") {
        return print_result(vehicle.return_to_launch());
    }
    if (arguments.command == "servo" && arguments.channel.has_value() && arguments.pwm_microseconds.has_value()) {
        return print_result(vehicle.set_servo(*arguments.channel, *arguments.pwm_microseconds));
    }
    if (arguments.command == "relay" && arguments.relay_number.has_value() && arguments.relay_on.has_value()) {
        return print_result(vehicle.set_relay(*arguments.relay_number, *arguments.relay_on));
    }
    if (arguments.command == "motor-test" && arguments.motor_instance.has_value() &&
        arguments.pwm_microseconds.has_value() && arguments.timeout_seconds.has_value()) {
        return print_result(
            vehicle.motor_test(*arguments.motor_instance, *arguments.pwm_microseconds, *arguments.timeout_seconds));
    }
    if (arguments.command == "gimbal-config" && arguments.mount_mode.has_value()) {
        return print_result(vehicle.configure_gimbal(*arguments.mount_mode));
    }
    if (arguments.command == "mission-demo") {
        return run_mission_demo(vehicle);
    }
    if (arguments.command == "velocity-demo") {
        return run_velocity_demo(vehicle);
    }
    if (arguments.command == "velocity" && arguments.velocity_vx.has_value() &&
        arguments.duration_seconds.has_value()) {
        return run_velocity(vehicle, *arguments.velocity_vx, arguments.velocity_vy.value_or(0.0F),
                            arguments.velocity_vz.value_or(0.0F), arguments.velocity_yaw_rate.value_or(0.0F),
                            *arguments.duration_seconds);
    }
    if (arguments.command == "fence-demo") {
        return run_fence_demo(vehicle);
    }
    if (arguments.command == "payload-demo" && arguments.relay_number.has_value() &&
        arguments.duration_seconds.has_value()) {
        return run_payload_demo(vehicle, *arguments.relay_number, *arguments.duration_seconds);
    }

    // Every verb in the table is dispatched above, so reaching this point means
    // a verb was added to the table without a handler. Report it instead of
    // exiting silently the way the old fall-through did.
    audit_command(arguments.command, "failed", "api-key", "verb_not_dispatched");
    std::cerr << "error: " << arguments.command << " has no handler in this build\n";
    return EXIT_FAILURE;
}

} // namespace

void print_qualification_usage() {
    const auto commands = cli_commands();
    std::cout << "Non-installed NOMAD qualification driver. Usage: nomad-qualification <";
    for (std::size_t index = 0; index < commands.size(); ++index) {
        std::cout << (index == 0 ? "" : "|") << commands[index].name;
    }
    std::cout << "> [value] [--endpoint udpin:host:port] [--system-id id]\n";
}

int main(int argc, char **argv) {
    const auto arguments = parse_qualification_arguments(argc, argv);
    if (!arguments.has_value()) {
        print_qualification_usage();
        return EXIT_FAILURE;
    }
    const auto connection =
        nomad::mavlink::make_mavsdk_connection(arguments->endpoint, arguments->system_id, std::chrono::seconds(6));
    return run_command(*connection, arguments->command, arguments->endpoint);
}
