// SPDX-License-Identifier: Apache-2.0
#include "support/fake_connection.hpp"
#include "nomad/runtime/runtime.hpp"
#include "support/test_harness.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <functional>
#include <future>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>
#endif

namespace {

using Json = nlohmann::json;

#ifdef _WIN32
using Socket = SOCKET;
constexpr Socket kInvalidSocket = INVALID_SOCKET;
#else
using Socket = int;
constexpr Socket kInvalidSocket = -1;
#endif

void initialize_sockets() {
#ifdef _WIN32
    static const bool initialized = [] {
        WSADATA data{};
        return WSAStartup(MAKEWORD(2, 2), &data) == 0;
    }();
    CHECK(initialized);
#endif
}

void close_socket(Socket socket) {
#ifdef _WIN32
    closesocket(socket);
#else
    ::close(socket);
#endif
}

void set_timeout(Socket socket) {
#ifdef _WIN32
    const DWORD timeout = 2000;
    setsockopt(socket, SOL_SOCKET, SO_RCVTIMEO, reinterpret_cast<const char *>(&timeout), sizeof(timeout));
#else
    const timeval timeout{2, 0};
    setsockopt(socket, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
#endif
}

std::uint16_t free_port() {
    initialize_sockets();
    const auto socket = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    CHECK(socket != kInvalidSocket);
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    address.sin_port = 0;
    CHECK(::bind(socket, reinterpret_cast<const sockaddr *>(&address), sizeof(address)) == 0);
#ifdef _WIN32
    int length = sizeof(address);
#else
    socklen_t length = sizeof(address);
#endif
    CHECK(getsockname(socket, reinterpret_cast<sockaddr *>(&address), &length) == 0);
    const auto port = ntohs(address.sin_port);
    close_socket(socket);
    return port;
}

class Client {
  public:
    explicit Client(std::uint16_t port) {
        initialize_sockets();
        socket_ = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        CHECK(socket_ != kInvalidSocket);
        sockaddr_in address{};
        address.sin_family = AF_INET;
        address.sin_port = htons(port);
        address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        CHECK(::connect(socket_, reinterpret_cast<const sockaddr *>(&address), sizeof(address)) == 0);
        set_timeout(socket_);
    }

    ~Client() {
        if (socket_ != kInvalidSocket) {
            close_socket(socket_);
        }
    }

    void send_raw(std::string_view line) {
        std::string framed(line);
        framed.push_back('\n');
        std::size_t offset = 0;
        while (offset < framed.size()) {
#ifdef _WIN32
            const int count = ::send(socket_, framed.data() + offset, static_cast<int>(framed.size() - offset), 0);
            CHECK(count > 0);
#else
            const auto count = ::send(socket_, framed.data() + offset, framed.size() - offset, 0);
            CHECK(count > 0);
#endif
            offset += static_cast<std::size_t>(count);
        }
    }

    void send(const Json &request) {
        send_raw(request.dump());
    }

    Json receive() {
        std::string line;
        char character{};
        while (line.size() <= 65536) {
#ifdef _WIN32
            const int count = ::recv(socket_, &character, 1, 0);
#else
            const auto count = ::recv(socket_, &character, 1, 0);
#endif
            CHECK(count == 1);
            if (character == '\n') {
                return Json::parse(line);
            }
            line.push_back(character);
        }
        throw std::runtime_error("response exceeded the test client limit");
    }

    Json request(const Json &body) {
        send(body);
        return receive();
    }

    void send_partial(std::string_view data) {
#ifdef _WIN32
        CHECK(::send(socket_, data.data(), static_cast<int>(data.size()), 0) == static_cast<int>(data.size()));
#else
        CHECK(::send(socket_, data.data(), data.size(), 0) == static_cast<ssize_t>(data.size()));
#endif
    }

    void disconnect() {
        close_socket(socket_);
        socket_ = kInvalidSocket;
    }

  private:
    Socket socket_{kInvalidSocket};
};

Json base_request(std::string id, std::string type, std::string client = "test-client") {
    return {{"protocol", "nomad-core"}, {"version", 1}, {"id", std::move(id)},
            {"client_id", std::move(client)}, {"type", std::move(type)}};
}

Json servo_request(std::string id, int pwm, std::string client = "test-client") {
    auto request = base_request(std::move(id), "set_servo", std::move(client));
    request["channel"] = 8;
    request["pwm_microseconds"] = pwm;
    return request;
}

void wait_until(const std::function<bool()> &predicate) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (std::chrono::steady_clock::now() < deadline) {
        if (predicate()) {
            return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    CHECK(predicate());
}

void test_protocol_and_status(std::uint16_t port, FakeConnection &connection) {
    Client client(port);
    const auto hello = client.request(base_request("1", "hello"));
    CHECK(hello["ok"] == true);
    CHECK(hello["protocol"] == "nomad-core");
    CHECK(hello["version"] == 1);
    CHECK(std::find(hello["capabilities"].begin(), hello["capabilities"].end(), "status") !=
          hello["capabilities"].end());
    CHECK(std::find(hello["capabilities"].begin(), hello["capabilities"].end(), "goto_location") ==
          hello["capabilities"].end());

    const auto status = client.request(base_request("2", "status"));
    CHECK(status["status"]["runtime_ready"] == true);
    CHECK(status["status"]["identity_resolved"] == false);
    CHECK(status["status"]["aircraft_class"] == "Unknown");
    CHECK(status["status"]["vehicle_session_established"] == true);

    connection.set_identity(nomad::telemetry::identify_vehicle(nomad::telemetry::kArduPilotAutopilot,
                                                                nomad::telemetry::kQuadrotor));
    const auto resolved = client.request(base_request("3", "status"));
    CHECK(resolved["status"]["identity_resolved"] == true);
    CHECK(resolved["status"]["aircraft_class"] == "Copter");
    CHECK(resolved["status"]["vehicle_connected"] == true);
    CHECK(client.request(base_request("4", "ping"))["type"] == "pong");

    auto read_status = [port] {
        Client independent_client(port);
        return independent_client.request(base_request("parallel-status", "status"));
    };
    auto first_status = std::async(std::launch::async, read_status);
    auto second_status = std::async(std::launch::async, read_status);
    CHECK(first_status.get()["ok"] == true);
    CHECK(second_status.get()["ok"] == true);
}

void test_protocol_errors(std::uint16_t port) {
    Client client(port);
    const auto incompatible = client.request([] {
        auto request = base_request("v2", "hello");
        request["version"] = 2;
        return request;
    }());
    CHECK(incompatible["error"]["code"] == "incompatible_version");
    Client wrong_protocol(port);
    auto wrong_name = base_request("wrong-name", "hello");
    wrong_name["protocol"] = "nomad-other";
    CHECK(wrong_protocol.request(wrong_name)["error"]["code"] == "incompatible_protocol");
    CHECK(client.request(base_request("unknown", "send_command"))["error"]["code"] == "unsupported_request");
    CHECK(client.request(base_request("no-navigation", "goto_location"))["error"]["code"] ==
          "unsupported_request");
    CHECK(client.request(base_request("no-shell", "execute_shell"))["error"]["code"] == "unsupported_request");
    CHECK(client.request(base_request("no-mavlink", "send_mavlink"))["error"]["code"] ==
          "unsupported_request");

    Client malformed(port);
    malformed.send_raw("{bad");
    CHECK(malformed.receive()["error"]["code"] == "malformed_json");

    Client deeply_nested(port);
    deeply_nested.send_raw(std::string(65, '[') + std::string(65, ']'));
    CHECK(deeply_nested.receive()["error"]["code"] == "malformed_json");

    Client oversized(port);
    oversized.send_raw(std::string(65537, 'x'));
    CHECK(oversized.receive()["error"]["code"] == "message_too_large");
    CHECK(client.request(base_request("after-error", "status"))["ok"] == true);
}

void test_command_dispatch_and_dedupe(std::uint16_t port, FakeConnection &connection) {
    connection.acknowledgement = nomad::mavlink::CommandAck{183, 0};
    const auto request = servo_request("servo-1", 1500);
    Client client(port);
    const auto result = client.request(request);
    CHECK(result["command_result"]["success"] == true);
    CHECK(connection.command_count() == 1);

    const auto duplicate = client.request(request);
    CHECK(duplicate == result);
    CHECK(connection.command_count() == 1);
    auto reused_id = servo_request("servo-1", 1600);
    CHECK(client.request(reused_id)["error"]["code"] == "request_id_conflict");
    CHECK(connection.command_count() == 1);

    CHECK(client.request(servo_request("c", 1500, "a\nb"))["command_result"]["success"] == true);
    CHECK(client.request(servo_request("b\nc", 1500, "a"))["command_result"]["success"] == true);
    CHECK(connection.command_count() == 3);
}

void test_vehicle_admission_is_authoritative(std::uint16_t port, FakeConnection &connection) {
    connection.set_identity(nomad::telemetry::identify_vehicle(nomad::telemetry::kArduPilotAutopilot,
                                                                nomad::telemetry::kVtolTiltrotor));
    Client client(port);
    const auto result = client.request(servo_request("unqualified-servo", 1500));
    CHECK(result["command_result"]["success"] == false);
    CHECK(connection.command_count() == 3);
}

void test_busy_and_slow_client(std::uint16_t port, FakeConnection &connection) {
    connection.set_identity(nomad::telemetry::identify_vehicle(nomad::telemetry::kArduPilotAutopilot,
                                                                nomad::telemetry::kQuadrotor));
    connection.command_started = false;
    connection.command_delay = std::chrono::milliseconds(300);
    Client first(port);
    first.send(servo_request("slow-command", 1500));
    wait_until([&connection] { return connection.command_started.load(); });

    Client second(port);
    CHECK(second.request(servo_request("busy-command", 1600))["error"]["code"] == "busy");
    CHECK(second.request(base_request("while-busy", "status"))["ok"] == true);
    const auto completed = first.receive();
    CHECK(completed["command_result"]["success"] == true);
    CHECK(connection.command_count() == 4);

    Client slow_client(port);
    slow_client.send_partial("{\"protocol\":");
    Client responsive(port);
    CHECK(responsive.request(base_request("responsive", "status"))["ok"] == true);
    connection.command_delay = std::chrono::milliseconds(0);
}

void test_disconnect_does_not_cancel_or_replay(std::uint16_t port, FakeConnection &connection) {
    Client disconnected(port);
    const auto request = servo_request("lost-response", 1700, "stable-client");
    disconnected.send(request);
    disconnected.disconnect();
    wait_until([&connection] { return connection.command_count() == 5; });

    Client reconnect(port);
    CHECK(reconnect.request(request)["command_result"]["success"] == true);
    CHECK(connection.command_count() == 5);
}

void test_runtime_owns_one_connection_and_releases_port() {
    const auto port = free_port();
    auto connection = std::make_unique<FakeConnection>();
    auto *observed = connection.get();
    observed->set_identity({});
    nomad::runtime::RuntimeConfig config;
    config.ipc_port = port;
    config.actuation_enabled = true;
    nomad::runtime::Runtime runtime(std::move(connection), config);
    std::string error;
    CHECK(runtime.start(error));
    wait_until([&runtime] { return runtime.ready(); });
    test_protocol_and_status(port, *observed);
    test_protocol_errors(port);
    test_command_dispatch_and_dedupe(port, *observed);
    test_vehicle_admission_is_authoritative(port, *observed);
    test_busy_and_slow_client(port, *observed);
    test_disconnect_does_not_cancel_or_replay(port, *observed);
    CHECK(observed->connect_count == 1);
    runtime.stop();
    CHECK(!runtime.ready());
}

void test_runtime_restart_and_missing_key() {
    const auto port = free_port();
    nomad::runtime::RuntimeConfig config;
    config.ipc_port = port;
    config.actuation_enabled = false;
    std::string error;
    {
        nomad::runtime::Runtime runtime(std::make_unique<FakeConnection>(), config);
        CHECK(runtime.start(error));
        Client client(port);
        CHECK(client.request(servo_request("no-key", 1500))["error"]["code"] == "missing_api_key");
        runtime.stop();
    }
    {
        nomad::runtime::Runtime runtime(std::make_unique<FakeConnection>(), config);
        CHECK(runtime.start(error));
        Client restarted(port);
        CHECK(restarted.request(base_request("restart-hello", "hello"))["ok"] == true);
        runtime.stop();
    }
}

} // namespace

int main() {
    return nomad::test::run_tests([] {
        test_runtime_owns_one_connection_and_releases_port();
        test_runtime_restart_and_missing_key();
    });
}
