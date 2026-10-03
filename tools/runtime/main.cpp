// SPDX-License-Identifier: Apache-2.0
#include "nomad/mavlink/mavsdk_transport.hpp"
#include "nomad/mavlink/mavsdk_validation.hpp"
#include "nomad/runtime/runtime.hpp"
#include "client_auth.hpp"
#include "lifecycle.hpp"
#include "service_config.hpp"
#include "nomad/safety/fence_config.hpp"
#include "nomad/safety/velocity_config.hpp"

#include <array>
#include <charconv>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <thread>
#include <utility>

namespace {

struct Arguments {
    std::string endpoint{"udpin:127.0.0.1:14601"};
    std::uint16_t port{14611};
    std::uint8_t system_id{1};
};

std::optional<unsigned int> parse_unsigned(std::string_view value) {
    unsigned int parsed{};
    const auto result = std::from_chars(value.data(), value.data() + value.size(), parsed);
    if (result.ec != std::errc{} || result.ptr != value.data() + value.size()) {
        return std::nullopt;
    }
    return parsed;
}

bool assign_env_port(Arguments &arguments) {
    const char *value = std::getenv("NOMAD_RUNTIME_IPC_PORT");
    if (value == nullptr || value[0] == '\0') {
        return true;
    }
    const auto parsed = parse_unsigned(value);
    if (!parsed || *parsed == 0 || *parsed > 65535) {
        return false;
    }
    arguments.port = static_cast<std::uint16_t>(*parsed);
    return true;
}

bool assign_env_endpoint(Arguments &arguments) {
    const char *value = std::getenv("NOMAD_MAVLINK_ENDPOINT");
    if (value != nullptr && value[0] != '\0') {
        arguments.endpoint = value;
    }
    return true;
}

bool assign_flag(Arguments &arguments, int argc, char **argv, int &index) {
    const std::string_view flag(argv[index]);
    if (flag != "--endpoint" && flag != "--ipc-port" && flag != "--system-id") {
        return false;
    }
    if (index + 1 >= argc) {
        return false;
    }
    const std::string_view value(argv[++index]);
    if (flag == "--endpoint") {
        arguments.endpoint = value;
        return !arguments.endpoint.empty();
    }
    const auto parsed = parse_unsigned(value);
    if (!parsed || *parsed == 0 || *parsed > (flag == "--ipc-port" ? 65535U : 255U)) {
        return false;
    }
    if (flag == "--ipc-port") {
        arguments.port = static_cast<std::uint16_t>(*parsed);
        return true;
    }
    arguments.system_id = static_cast<std::uint8_t>(*parsed);
    return true;
}

std::optional<Arguments> parse_arguments(int argc, char **argv) {
    Arguments arguments;
    if (!assign_env_port(arguments) || !assign_env_endpoint(arguments)) {
        return std::nullopt;
    }
    for (int index = 1; index < argc; ++index) {
        const std::string_view token(argv[index]);
        if (token == "--service") {
            continue;
        }
        if (token == "--config" && index + 1 < argc) {
            ++index;
            continue;
        }
        if (token == "--help" || token == "-h") {
            return arguments;
        }
        if (!assign_flag(arguments, argc, argv, index)) {
            return std::nullopt;
        }
    }
    return arguments;
}

void print_usage() {
    std::cout << "Usage: nomad-runtime [--endpoint udpin:127.0.0.1:14601] [--ipc-port 14611] [--system-id 1]\n";
    std::cout << "IPC binds only to 127.0.0.1. NOMAD_RUNTIME_IPC_PORT and NOMAD_MAVLINK_ENDPOINT may set defaults.\n";
    std::cout << "--config <protected-json> loads service environment; --service uses Windows SCM.\n";
}

bool api_key_configured() {
    const char *key = std::getenv("NOMAD_API_KEY");
    return key != nullptr && key[0] != '\0';
}

int fail_runtime(const char *message, int code = nomad::runtime::process::kConfigurationFailure) {
    std::cerr << message << '\n';
    nomad::runtime::process::publish_error(message);
    return code;
}

bool load_process_configuration(int argc, char **argv) {
    bool loaded = false;
    bool service = false;
    for (int index = 1; index < argc; ++index) {
        service = service || std::string_view(argv[index]) == "--service";
        if (std::string_view(argv[index]) == "--config") {
            if (loaded || ++index >= argc || !load_service_environment(argv[index])) {
                return false;
            }
            loaded = true;
        }
    }
    return !service || loaded;
}

std::optional<nomad::runtime::RuntimeConfig> load_runtime_config(const Arguments &arguments) {
    nomad::runtime::RuntimeConfig config;
    config.ipc_port = arguments.port;
#ifdef NOMAD_VERSION
    config.version = NOMAD_VERSION;
#endif
    config.actuation_enabled = api_key_configured();
    const char *credentials = std::getenv("NOMAD_CLIENT_CREDENTIALS_FILE");
    const char *audit = std::getenv("NOMAD_AUDIT_DIRECTORY");
    if (credentials == nullptr || !nomad::runtime::detail::load_credentials(credentials, config.client_credentials)) {
        return std::nullopt;
    }
    config.audit_directory = audit == nullptr ? "" : audit;
    config.fence_policy = nomad::safety::load_fence_policy(std::getenv("NOMAD_FENCE_POLYGON"),
                                                            std::getenv("NOMAD_FENCE_MARGIN_M"));
    config.velocity_limits = nomad::safety::load_velocity_limits(
        std::getenv("NOMAD_VELOCITY_MAX_XY"), std::getenv("NOMAD_VELOCITY_MAX_Z"),
        std::getenv("NOMAD_VELOCITY_MAX_YAW_RATE"));
    return config;
}

int wait_runtime(nomad::runtime::Runtime &runtime, std::uint16_t port) {
    std::string error;
    if (!runtime.start(error)) {
        std::cerr << "runtime startup failed: " << error << '\n';
        return fail_runtime("runtime startup failed: audit/lock/IPC initialization failed");
    }
    struct Publication {
        ~Publication() { nomad::runtime::process::publish_stopping(); }
    } publication;
    nomad::runtime::process::publish_ready(runtime);
    std::cout << "READY protocol=nomad-core version=1 ipc=127.0.0.1:" << port << '\n';
    while (!nomad::runtime::process::stop_requested()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    const int result = nomad::runtime::process::shutdown_runtime(runtime);
    if (result != 0) {
        return fail_runtime("runtime shutdown audit failure", result);
    }
    return EXIT_SUCCESS;
}

int run_runtime(int argc, char **argv) {
    for (int index = 1; index < argc; ++index) {
        const std::string_view token(argv[index]);
        if (token == "--help" || token == "-h") {
            print_usage();
            return EXIT_SUCCESS;
        }
    }
    if (!load_process_configuration(argc, argv)) {
        return fail_runtime("runtime configuration error: valid protected service configuration required");
    }
    const auto arguments = parse_arguments(argc, argv);
    if (!arguments) {
        return fail_runtime("runtime configuration error: invalid endpoint/IPC/system arguments");
    }
    const auto endpoint = nomad::mavsdk_phase_a::canonicalize_udp_endpoint(arguments->endpoint);
    if (!endpoint) {
        return fail_runtime(
            "runtime configuration error: NOMAD_MAVLINK_ENDPOINT must be a supported UDP MAVSDK endpoint");
    }
    auto config = load_runtime_config(*arguments);
    if (!config) {
        return fail_runtime("runtime configuration error: valid protected client credentials file required");
    }
    auto connection = nomad::mavlink::make_mavsdk_connection(
        *endpoint, arguments->system_id, config->discovery_timeout);
    nomad::runtime::Runtime runtime(std::move(connection), std::move(*config));
    return wait_runtime(runtime, arguments->port);
}

} // namespace

int main(int argc, char **argv) {
    if (argc == 2 && std::string_view(argv[1]) == "--version") {
        std::cout << NOMAD_VERSION << '\n';
        return 0;
    }
    for (int index = 1; index < argc; ++index) {
        if (std::string_view(argv[index]) == "--service") {
            return nomad::runtime::process::run_service(argc, argv, run_runtime);
        }
    }
    nomad::runtime::process::initialize_console();
    try {
        return run_runtime(argc, argv);
    } catch (...) {
        std::cerr << "runtime process failure\n";
        return EXIT_FAILURE;
    }
}
