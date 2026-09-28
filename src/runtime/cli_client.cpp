// SPDX-License-Identifier: Apache-2.0
#include "cli_client.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <climits>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <random>
#include <string>
#include <string_view>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <cerrno>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>
#endif

namespace {

using Json = nlohmann::json;

#ifdef _WIN32
using NativeSocket = SOCKET;
constexpr NativeSocket kInvalidSocket = INVALID_SOCKET;
#else
using NativeSocket = int;
constexpr NativeSocket kInvalidSocket = -1;
#endif

constexpr std::size_t kMaximumMessageBytes = 64 * 1024;
constexpr std::string_view kProtocolName = "nomad-core";
constexpr int kProtocolVersion = 1;

bool initialize_sockets() {
#ifdef _WIN32
    static const bool initialized = [] {
        WSADATA data{};
        return WSAStartup(MAKEWORD(2, 2), &data) == 0;
    }();
    return initialized;
#else
    return true;
#endif
}

void close_socket(NativeSocket socket) {
    if (socket == kInvalidSocket) {
        return;
    }
#ifdef _WIN32
    closesocket(socket);
#else
    ::close(socket);
#endif
}

bool set_socket_timeouts(NativeSocket socket) {
#ifdef _WIN32
    const DWORD receive_timeout = 120000;
    const DWORD send_timeout = 3000;
    return setsockopt(socket, SOL_SOCKET, SO_RCVTIMEO, reinterpret_cast<const char *>(&receive_timeout),
                      sizeof(receive_timeout)) == 0 &&
           setsockopt(socket, SOL_SOCKET, SO_SNDTIMEO, reinterpret_cast<const char *>(&send_timeout),
                      sizeof(send_timeout)) == 0;
#else
    const timeval receive_timeout{120, 0};
    const timeval send_timeout{3, 0};
    return setsockopt(socket, SOL_SOCKET, SO_RCVTIMEO, &receive_timeout, sizeof(receive_timeout)) == 0 &&
           setsockopt(socket, SOL_SOCKET, SO_SNDTIMEO, &send_timeout, sizeof(send_timeout)) == 0;
#endif
}

bool wait_for_socket(NativeSocket socket, bool writing, std::chrono::milliseconds timeout) {
    fd_set ready;
    FD_ZERO(&ready);
    FD_SET(socket, &ready);
    const auto seconds = std::chrono::duration_cast<std::chrono::seconds>(timeout);
    const auto remainder = std::chrono::duration_cast<std::chrono::microseconds>(timeout - seconds);
    timeval wait{static_cast<long>(seconds.count()), static_cast<long>(remainder.count())};
    return select(
#ifdef _WIN32
        0,
#else
        socket + 1,
#endif
        writing ? nullptr : &ready, writing ? &ready : nullptr, nullptr, &wait) > 0;
}

NativeSocket connect_loopback(std::uint16_t port) {
    if (!initialize_sockets()) {
        return kInvalidSocket;
    }
    const auto socket = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (socket == kInvalidSocket) {
        return kInvalidSocket;
    }

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(port);
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
#ifdef _WIN32
    u_long nonblocking = 1;
    if (ioctlsocket(socket, FIONBIO, &nonblocking) != 0) {
        close_socket(socket);
        return kInvalidSocket;
    }
    const auto connected = ::connect(socket, reinterpret_cast<const sockaddr *>(&address), sizeof(address));
    if (connected == SOCKET_ERROR &&
        (WSAGetLastError() != WSAEWOULDBLOCK || !wait_for_socket(socket, true, std::chrono::milliseconds(1500)))) {
        close_socket(socket);
        return kInvalidSocket;
    }
#else
    const auto old_flags = fcntl(socket, F_GETFL, 0);
    if (old_flags < 0 || fcntl(socket, F_SETFL, old_flags | O_NONBLOCK) != 0) {
        close_socket(socket);
        return kInvalidSocket;
    }
    const auto connected = ::connect(socket, reinterpret_cast<const sockaddr *>(&address), sizeof(address));
    if (connected != 0 && (errno != EINPROGRESS || !wait_for_socket(socket, true, std::chrono::milliseconds(1500)))) {
        close_socket(socket);
        return kInvalidSocket;
    }
#endif
    int socket_error = 0;
#ifdef _WIN32
    int error_size = sizeof(socket_error);
#else
    socklen_t error_size = sizeof(socket_error);
#endif
    if (getsockopt(socket, SOL_SOCKET, SO_ERROR, reinterpret_cast<char *>(&socket_error), &error_size) != 0 ||
        socket_error != 0) {
        close_socket(socket);
        return kInvalidSocket;
    }
#ifdef _WIN32
    nonblocking = 0;
    if (ioctlsocket(socket, FIONBIO, &nonblocking) != 0) {
        close_socket(socket);
        return kInvalidSocket;
    }
#else
    if (fcntl(socket, F_SETFL, old_flags) != 0) {
        close_socket(socket);
        return kInvalidSocket;
    }
#endif
    if (!set_socket_timeouts(socket)) {
        close_socket(socket);
        return kInvalidSocket;
    }
    return socket;
}

bool send_all(NativeSocket socket, std::string_view message) {
    std::size_t sent = 0;
    while (sent < message.size()) {
#ifdef _WIN32
        const auto count = ::send(socket, message.data() + sent,
                                  static_cast<int>(std::min<std::size_t>(message.size() - sent, INT_MAX)), 0);
        if (count == SOCKET_ERROR || count == 0) {
            return false;
        }
#else
        const auto count = ::send(socket, message.data() + sent, message.size() - sent,
#ifdef MSG_NOSIGNAL
                                  MSG_NOSIGNAL
#else
                                  0
#endif
        );
        if (count <= 0) {
            return false;
        }
#endif
        sent += static_cast<std::size_t>(count);
    }
    return true;
}

bool write_message(NativeSocket socket, const Json &message) {
    const auto bytes = message.dump();
    return bytes.size() <= kMaximumMessageBytes && send_all(socket, bytes + "\n");
}

bool read_message(NativeSocket socket, Json &message) {
    std::string bytes;
    std::array<char, 2048> buffer{};
    while (bytes.size() <= kMaximumMessageBytes) {
#ifdef _WIN32
        const auto count = ::recv(socket, buffer.data(), static_cast<int>(buffer.size()), 0);
        if (count == SOCKET_ERROR || count == 0) {
            return false;
        }
#else
        const auto count = ::recv(socket, buffer.data(), buffer.size(), 0);
        if (count <= 0) {
            return false;
        }
#endif
        const auto newline = std::find(buffer.begin(), buffer.begin() + count, '\n');
        const auto chunk_size = static_cast<std::size_t>(newline - buffer.begin());
        if (bytes.size() + chunk_size > kMaximumMessageBytes) {
            return false;
        }
        bytes.append(buffer.data(), chunk_size);
        if (newline != buffer.begin() + count) {
            message = Json::parse(bytes, nullptr, false);
            return message.is_object();
        }
    }
    return false;
}

std::string new_request_id() {
    std::random_device random;
    constexpr char digits[] = "0123456789abcdef";
    std::string id(32, '0');
    for (auto &digit : id) {
        digit = digits[random() & 0x0fU];
    }
    return id;
}

std::uint16_t runtime_port() {
    const char *configured = std::getenv("NOMAD_RUNTIME_IPC_PORT");
    if (configured == nullptr || configured[0] == '\0') {
        return 14611;
    }
    unsigned int port{};
    const auto *end = configured;
    while (*end >= '0' && *end <= '9') {
        port = port * 10U + static_cast<unsigned int>(*end - '0');
        if (port > 65535U) {
            return 0;
        }
        ++end;
    }
    return *end == '\0' && port > 0 ? static_cast<std::uint16_t>(port) : 0;
}

Json make_request(const std::string &id, const std::string &type) {
    return {{"protocol", kProtocolName}, {"version", kProtocolVersion}, {"client_id", "nomad-cli"},
            {"id", id}, {"type", type}};
}

bool add_typed_arguments(const Arguments &arguments, Json &request) {
    const auto &command = arguments.command;
    if (command == "status") {
        request = make_request(new_request_id(), "status");
        return true;
    }
    if (command == "admit" || command == "revoke" || command == "handback") {
        const auto type = command == "admit" ? "admit_authority" :
                          command == "revoke" ? "revoke_authority" : "handback_authority";
        request = make_request(new_request_id(), type);
        return true;
    }
    if (command == "servo" && arguments.channel && arguments.pwm_microseconds) {
        request = make_request(new_request_id(), "set_servo");
        request["channel"] = *arguments.channel;
        request["pwm_microseconds"] = *arguments.pwm_microseconds;
        return true;
    }
    if (command == "relay" && arguments.relay_number && arguments.relay_on) {
        request = make_request(new_request_id(), "set_relay");
        request["relay_number"] = *arguments.relay_number;
        request["on"] = *arguments.relay_on;
        return true;
    }
    if (command == "motor-test" && arguments.motor_instance && arguments.pwm_microseconds &&
        arguments.timeout_seconds) {
        request = make_request(new_request_id(), "motor_test");
        request["motor_instance"] = *arguments.motor_instance;
        request["pwm_microseconds"] = *arguments.pwm_microseconds;
        request["timeout_seconds"] = *arguments.timeout_seconds;
        return true;
    }
    if (command == "gimbal-config" && arguments.mount_mode) {
        request = make_request(new_request_id(), "configure_gimbal");
        request["mount_mode"] = *arguments.mount_mode;
        return true;
    }
    return false;
}

bool is_mutating(const Json &request) {
    const auto type = request.value("type", "");
    return type != "status";
}

bool bind_authority(Json &request, const Json &hello_response) {
    if (!hello_response.contains("authority") || !hello_response["authority"].is_object() ||
        !hello_response.contains("runtime_incarnation")) {
        return false;
    }
    const auto &authority = hello_response["authority"];
    if (!authority.contains("vehicle_session") || !authority.contains("generation")) {
        return false;
    }
    const auto now = std::chrono::system_clock::now().time_since_epoch();
    request["runtime_incarnation"] = hello_response["runtime_incarnation"];
    request["vehicle_session"] = authority["vehicle_session"];
    request["authority_generation"] = authority["generation"];
    request["command_source"] = "nomad-cli";
    request["sequence"] = std::chrono::duration_cast<std::chrono::microseconds>(now).count();
    request["expires_at_ms"] = std::chrono::duration_cast<std::chrono::milliseconds>(now).count() + 3000;
    return true;
}

bool valid_response(const Json &response, const std::string &id) {
    return response.is_object() && response.value("protocol", "") == kProtocolName &&
           response.value("version", 0) == kProtocolVersion && response.value("id", "") == id &&
           response.contains("ok") && response["ok"].is_boolean();
}

void print_error(const Json &response) {
    const auto error = response.value("error", Json::object());
    std::cerr << "error[" << error.value("code", "protocol_error") << "]: "
              << error.value("message", "runtime rejected the request") << '\n';
}

int receive_result(NativeSocket socket, const std::string &request_id, bool mutation) {
    Json response;
    if (!read_message(socket, response)) {
        if (mutation) {
            std::cerr << "error[unknown_outcome]: runtime connection ended after request send; "
                         "vehicle outcome is unknown\n";
        } else {
            std::cerr << "error[runtime_unavailable]: no complete response from runtime\n";
        }
        return EXIT_FAILURE;
    }
    if (!valid_response(response, request_id)) {
        std::cerr << "error[invalid_response]: runtime response did not match the request\n";
        return EXIT_FAILURE;
    }
    if (!response["ok"].get<bool>()) {
        print_error(response);
        return EXIT_FAILURE;
    }
    if (response.value("type", "") == "command_response") {
        const auto result = response.value("command_result", Json::object());
        const bool success = result.value("success", false);
        std::cout << result.value("message", "vehicle operation returned") << '\n';
        return success ? EXIT_SUCCESS : EXIT_FAILURE;
    }
    std::cout << response.dump(2) << '\n';
    return EXIT_SUCCESS;
}

} // namespace

bool runtime_endpoint_is_open() {
    const auto port = runtime_port();
    if (port == 0) {
        return true;
    }
    const auto socket = connect_loopback(port);
    if (socket == kInvalidSocket) {
        return false;
    }
    close_socket(socket);
    return true;
}

int run_runtime_command(const Arguments &arguments) {
    if (arguments.endpoint_explicit || arguments.system_id_explicit) {
        std::cerr << "error[invalid_configuration]: --runtime uses endpoint and system identity configured by "
                     "nomad-runtime\n";
        return EXIT_FAILURE;
    }
    Json request;
    if (!add_typed_arguments(arguments, request)) {
        std::cerr << "error[unsupported_request]: " << arguments.command
                  << " is not available through runtime protocol v1\n";
        return EXIT_FAILURE;
    }
    const auto port = runtime_port();
    if (port == 0) {
        std::cerr << "error[invalid_configuration]: NOMAD_RUNTIME_IPC_PORT must be between 1 and 65535\n";
        return EXIT_FAILURE;
    }
    const auto socket = connect_loopback(port);
    if (socket == kInvalidSocket) {
        std::cerr << "error[runtime_unavailable]: could not connect to NOMAD runtime at 127.0.0.1:"
                  << port << " within 1500 ms\n";
        return EXIT_FAILURE;
    }

    const auto hello_id = new_request_id();
    const auto hello = make_request(hello_id, "hello");
    if (!write_message(socket, hello)) {
        close_socket(socket);
        std::cerr << "error[runtime_unavailable]: could not send protocol HELLO\n";
        return EXIT_FAILURE;
    }
    Json hello_response;
    if (!read_message(socket, hello_response)) {
        close_socket(socket);
        std::cerr << "error[runtime_unavailable]: runtime did not complete protocol HELLO\n";
        return EXIT_FAILURE;
    }
    if (!valid_response(hello_response, hello_id) || !hello_response["ok"].get<bool>() ||
        hello_response.value("type", "") != "hello_response") {
        print_error(hello_response);
        close_socket(socket);
        return EXIT_FAILURE;
    }

    if (is_mutating(request) && !bind_authority(request, hello_response)) {
        close_socket(socket);
        std::cerr << "error[invalid_response]: runtime did not provide an authority context\n";
        return EXIT_FAILURE;
    }

    const auto request_id = request["id"].get<std::string>();
    const bool mutation = is_mutating(request);
    if (!write_message(socket, request)) {
        close_socket(socket);
        if (mutation) {
            std::cerr << "error[unknown_outcome]: request write failed; vehicle outcome is unknown\n";
        } else {
            std::cerr << "error[runtime_unavailable]: could not send request\n";
        }
        return EXIT_FAILURE;
    }
    const auto result = receive_result(socket, request_id, mutation);
    close_socket(socket);
    return result;
}
