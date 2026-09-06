// SPDX-License-Identifier: Apache-2.0
#include "cli_commands.hpp"
#include "nomad/mavlink/udp_connection.hpp"
#include "nomad/mission/executor.hpp"
#include "nomad/safety/fence_config.hpp"
#include "nomad/safety/watchdog.hpp"
#include "nomad/vehicle/vehicle.hpp"

#include <charconv>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <optional>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

namespace {

constexpr std::string_view kDefaultEndpoint = "udpin:0.0.0.0:14550";

struct Arguments {
    std::string command;
    std::string endpoint{kDefaultEndpoint};
    std::optional<float> altitude;
    std::optional<float> latitude;
    std::optional<float> longitude;
    std::optional<float> velocity_vx;
    std::optional<float> velocity_vy;
    std::optional<float> velocity_vz;
    std::optional<float> velocity_yaw_rate;
    std::optional<std::uint32_t> mode;
    std::optional<int> relay_number;
    std::optional<float> duration_seconds;
    std::optional<int> channel;
    std::optional<int> pwm_microseconds;
    std::optional<bool> relay_on;
    std::optional<int> motor_instance;
    std::optional<float> timeout_seconds;
    std::optional<int> mount_mode;
    std::optional<std::array<float, 7>> user_parameters;
    int user_parameter_count{0};
};

void print_usage() {
    std::cout << "Usage: nomad <connect|status|arm|disarm|mode|takeoff|goto|land|rtl|servo|relay|motor-test|gimbal-config|user-command|mission-demo|velocity|velocity-demo|fence-demo|payload-demo> "
                 "[value] [--endpoint udpin:host:port]\n";
    std::cout << "goto requires: <latitude> <longitude> <altitude_m>\n";
    std::cout << "servo <channel> <pwm_us> | relay <number> <0|1> | motor-test <instance> <pwm_us> <timeout_s> | "
                 "gimbal-config <mount_mode> | user-command <p1> <p2> <p3> <p4> <p5> <p6> <p7>\n";
    std::cout << "velocity requires: --vx <m_s> [--vy --vz --yaw-rate] --duration <seconds>\n";
    std::cout << "Actuation commands require the NOMAD_API_KEY environment variable.\n";
}

std::optional<float> parse_float(std::string_view value) {
    std::string copy(value);
    char *end = nullptr;
    const auto parsed = std::strtof(copy.c_str(), &end);
    if (end == copy.c_str() || *end != '\0') {
        return std::nullopt;
    }
    return parsed;
}

std::optional<std::uint32_t> parse_mode(std::string_view value) {
    std::uint32_t mode{};
    const auto result = std::from_chars(value.data(), value.data() + value.size(), mode);
    if (result.ec != std::errc{} || result.ptr != value.data() + value.size()) {
        return std::nullopt;
    }
    return mode;
}

// Non-negative integer for the output verbs, bounded by the caller.
std::optional<int> parse_output_int(std::string_view value, std::uint32_t maximum) {
    const auto parsed = parse_mode(value);
    if (!parsed.has_value() || *parsed > maximum) {
        return std::nullopt;
    }
    return static_cast<int>(*parsed);
}

bool is_supported_command(std::string_view command) {
    return command == "connect" || command == "status" || command == "arm" || command == "disarm" ||
           command == "mode" || command == "takeoff" || command == "goto" || command == "land" ||
           command == "rtl" || command == "servo" || command == "relay" || command == "motor-test" ||
           command == "gimbal-config" || command == "user-command" || command == "mission-demo" ||
           command == "velocity" || command == "velocity-demo" || command == "fence-demo" ||
           command == "payload-demo";
}

std::optional<Arguments> parse_arguments(int argc, char **argv) {
    if (argc < 2 || !is_supported_command(argv[1])) {
        return std::nullopt;
    }

    Arguments arguments{argv[1]};
    for (int index = 2; index < argc; ++index) {
        const std::string_view value(argv[index]);
        if (value == "--endpoint" && index + 1 < argc) {
            arguments.endpoint = argv[++index];
            continue;
        }
        if (arguments.command == "velocity" && value == "--vx" && index + 1 < argc) {
            arguments.velocity_vx = parse_float(argv[++index]);
            if (!arguments.velocity_vx.has_value()) {
                return std::nullopt;
            }
            continue;
        }
        if (arguments.command == "velocity" && value == "--vy" && index + 1 < argc) {
            arguments.velocity_vy = parse_float(argv[++index]);
            if (!arguments.velocity_vy.has_value()) {
                return std::nullopt;
            }
            continue;
        }
        if (arguments.command == "velocity" && value == "--vz" && index + 1 < argc) {
            arguments.velocity_vz = parse_float(argv[++index]);
            if (!arguments.velocity_vz.has_value()) {
                return std::nullopt;
            }
            continue;
        }
        if (arguments.command == "velocity" && value == "--yaw-rate" && index + 1 < argc) {
            arguments.velocity_yaw_rate = parse_float(argv[++index]);
            if (!arguments.velocity_yaw_rate.has_value()) {
                return std::nullopt;
            }
            continue;
        }
        if (arguments.command == "velocity" && value == "--duration" && index + 1 < argc) {
            arguments.duration_seconds = parse_float(argv[++index]);
            if (!arguments.duration_seconds.has_value()) {
                return std::nullopt;
            }
            continue;
        }
        if (arguments.command == "takeoff") {
            if (arguments.altitude.has_value()) return std::nullopt;
            arguments.altitude = parse_float(value);
            if (!arguments.altitude.has_value()) return std::nullopt;
            continue;
        }
        if (arguments.command == "goto") {
            if (arguments.latitude && arguments.longitude && arguments.altitude) return std::nullopt;
            std::optional<float> *slot = !arguments.latitude ? &arguments.latitude
                                         : !arguments.longitude ? &arguments.longitude
                                                                 : &arguments.altitude;
            *slot = parse_float(value);
            if (!slot->has_value()) return std::nullopt;
            continue;
        }
        if (arguments.command == "mode" && !arguments.mode.has_value()) {
            arguments.mode = parse_mode(value);
            if (!arguments.mode.has_value()) return std::nullopt;
            continue;
        }
        if (arguments.command == "payload-demo") {
            if (!arguments.relay_number.has_value()) {
                const auto parsed = parse_output_int(value, 15);
                if (!parsed.has_value()) {
                    return std::nullopt;
                }
                arguments.relay_number = *parsed;
                continue;
            }
            if (!arguments.duration_seconds.has_value()) {
                arguments.duration_seconds = parse_float(value);
                if (!arguments.duration_seconds.has_value()) {
                    return std::nullopt;
                }
                continue;
            }
        }
        if (arguments.command == "servo") {
            const auto parsed = parse_output_int(value, 65535);
            if (!parsed.has_value()) {
                return std::nullopt;
            }
            if (!arguments.channel.has_value()) {
                arguments.channel = *parsed;
            } else if (!arguments.pwm_microseconds.has_value()) {
                arguments.pwm_microseconds = *parsed;
            } else {
                return std::nullopt;
            }
            continue;
        }
        if (arguments.command == "relay") {
            if (!arguments.relay_number.has_value()) {
                const auto parsed = parse_output_int(value, 15);
                if (!parsed.has_value()) {
                    return std::nullopt;
                }
                arguments.relay_number = *parsed;
            } else if (!arguments.relay_on.has_value()) {
                const auto parsed = parse_output_int(value, 1);
                if (!parsed.has_value()) {
                    return std::nullopt;
                }
                arguments.relay_on = *parsed == 1;
            } else {
                return std::nullopt;
            }
            continue;
        }
        if (arguments.command == "motor-test") {
            if (!arguments.motor_instance.has_value()) {
                const auto parsed = parse_output_int(value, 65535);
                if (!parsed.has_value()) {
                    return std::nullopt;
                }
                arguments.motor_instance = *parsed;
            } else if (!arguments.pwm_microseconds.has_value()) {
                const auto parsed = parse_output_int(value, 65535);
                if (!parsed.has_value()) {
                    return std::nullopt;
                }
                arguments.pwm_microseconds = *parsed;
            } else if (!arguments.timeout_seconds.has_value()) {
                arguments.timeout_seconds = parse_float(value);
                if (!arguments.timeout_seconds.has_value()) {
                    return std::nullopt;
                }
            } else {
                return std::nullopt;
            }
            continue;
        }
        if (arguments.command == "gimbal-config") {
            if (arguments.mount_mode.has_value()) {
                return std::nullopt;
            }
            const auto parsed = parse_output_int(value, 4);
            if (!parsed.has_value()) {
                return std::nullopt;
            }
            arguments.mount_mode = *parsed;
            continue;
        }
        if (arguments.command == "user-command") {
            const auto parsed = parse_float(value);
            if (!parsed.has_value() || arguments.user_parameter_count >= 7) {
                return std::nullopt;
            }
            if (!arguments.user_parameters.has_value()) {
                arguments.user_parameters = std::array<float, 7>{};
            }
            (*arguments.user_parameters)[arguments.user_parameter_count++] = *parsed;
            continue;
        }
        return std::nullopt;
    }
    const bool has_required =
        (arguments.command != "servo" ||
         (arguments.channel.has_value() && arguments.pwm_microseconds.has_value())) &&
        (arguments.command != "relay" || (arguments.relay_number.has_value() && arguments.relay_on.has_value())) &&
        (arguments.command != "motor-test" || (arguments.motor_instance.has_value() &&
                                                arguments.pwm_microseconds.has_value() &&
                                                arguments.timeout_seconds.has_value())) &&
        (arguments.command != "gimbal-config" || arguments.mount_mode.has_value()) &&
        (arguments.command != "user-command" ||
         (arguments.user_parameters.has_value() && arguments.user_parameter_count == 7)) &&
        (arguments.command != "velocity" ||
         (arguments.velocity_vx.has_value() && arguments.duration_seconds.has_value())) &&
        (arguments.command != "goto" || (arguments.latitude.has_value() && arguments.longitude.has_value() &&
                                          arguments.altitude.has_value()));
    if (!has_required) {
        return std::nullopt;
    }
    return arguments;
}

// Telemetry verbs stay available without a key (local console fallback);
// actuation verbs always require the boundary credential, mirroring the
// transitional REST policy where command paths never open unauthenticated.
bool is_actuation_command(std::string_view command) {
    return command != "connect" && command != "status";
}

bool api_key_configured() {
    const char *key = std::getenv("NOMAD_API_KEY");
    return key != nullptr && key[0] != '\0';
}

void audit_command(std::string_view command, std::string_view result, std::string_view auth,
                   std::string_view reason) {
    std::cerr << "audit command=" << command << " result=" << result << " auth=" << auth;
    if (!reason.empty()) std::cerr << " reason=" << reason;
    std::cerr << '\n';
}

int print_result(const nomad::vehicle::CommandResult &result) {
    std::cout << result.message << '\n';
    return result.success ? EXIT_SUCCESS : EXIT_FAILURE;
}

int run_mission_demo(nomad::vehicle::Vehicle &vehicle) {
    const nomad::mission::Mission mission{
        nomad::mission::Action{"guided"}, nomad::mission::Action{"arm"}, nomad::mission::Takeoff{5.0F},
        nomad::mission::ReturnToLaunch{}, nomad::mission::Land{},        nomad::mission::Action{"wait_disarmed"},
    };
    nomad::mission::MissionExecutor executor(vehicle);
    const auto result = executor.execute(mission);
    std::cout << "mission_success=" << (result.success ? "true" : "false")
              << " completed_steps=" << result.completed_steps << " message=" << result.message << '\n';
    return result.success ? EXIT_SUCCESS : EXIT_FAILURE;
}

int run_fence_demo(nomad::vehicle::Vehicle &vehicle) {
    nomad::telemetry::VehicleState state{};
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (std::chrono::steady_clock::now() < deadline) {
        const auto partial = vehicle.wait_for_state(std::chrono::milliseconds(500));
        if (partial.has_value()) {
            state = *partial;
            if (state.position_valid) {
                break;
            }
        }
    }
    if (!state.position_valid) {
        std::cerr << "fence-demo requires authoritative position telemetry\n";
        return EXIT_FAILURE;
    }
    const auto &position = state.position;
    constexpr double offset = 0.0001;
    const std::vector<nomad::safety::GlobalPoint> boundary{
        {position.latitude_deg - offset, position.longitude_deg - offset},
        {position.latitude_deg - offset, position.longitude_deg + offset},
        {position.latitude_deg + offset, position.longitude_deg + offset},
        {position.latitude_deg + offset, position.longitude_deg - offset},
    };
    const auto upload = vehicle.upload_fence(boundary);
    if (!upload.success) {
        std::cerr << upload.message << '\n';
        return EXIT_FAILURE;
    }
    const auto verification = vehicle.verify_fence_uploaded(boundary);
    std::cout << verification.message << '\n';
    return verification.success ? EXIT_SUCCESS : EXIT_FAILURE;
}

int run_payload_demo(nomad::vehicle::Vehicle &vehicle, int relay_number, float duration_seconds) {
    const auto armed = vehicle.arm_payload();
    if (!armed.success) {
        std::cerr << armed.message << '\n';
        return EXIT_FAILURE;
    }
    return print_result(vehicle.release_payload(relay_number, duration_seconds));
}

int run_command(const Arguments &arguments) {
    if (is_actuation_command(arguments.command)) {
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

    nomad::mavlink::UdpMavlinkConnection connection(arguments.endpoint);
    if (!connection.connect()) {
        std::cerr << "could not connect to " << arguments.endpoint << '\n';
        return EXIT_FAILURE;
    }

    if (arguments.command == "status") {
        return run_status(connection);
    }
    if (arguments.command == "connect") {
        const auto heartbeat = connection.wait_for_heartbeat(std::chrono::seconds(3));
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
    nomad::vehicle::Vehicle vehicle(connection, {}, fence_policy);
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
    if (arguments.command == "user-command" && arguments.user_parameters.has_value()) {
        return print_result(vehicle.send_user_command(*arguments.user_parameters));
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
    if (arguments.command == "payload-demo" && arguments.relay_number.has_value() && arguments.duration_seconds.has_value()) {
        return run_payload_demo(vehicle, *arguments.relay_number, *arguments.duration_seconds);
    }
    return EXIT_FAILURE;
}

} // namespace

int main(int argc, char **argv) {
    const auto arguments = parse_arguments(argc, argv);
    if (!arguments.has_value()) {
        print_usage();
        return EXIT_FAILURE;
    }
    return run_command(*arguments);
}
