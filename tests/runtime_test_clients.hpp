// SPDX-License-Identifier: Apache-2.0
// Socket client and isolated storage helpers for runtime integration tests.

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

void set_timeout(Socket socket, std::chrono::milliseconds duration = std::chrono::seconds(2)) {
#ifdef _WIN32
    const DWORD timeout = static_cast<DWORD>(duration.count());
    CHECK(setsockopt(socket, SOL_SOCKET, SO_RCVTIMEO, reinterpret_cast<const char *>(&timeout), sizeof(timeout)) == 0);
#else
    const auto seconds = std::chrono::duration_cast<std::chrono::seconds>(duration);
    const timeval timeout{static_cast<long>(seconds.count()),
        static_cast<long>(std::chrono::duration_cast<std::chrono::microseconds>(duration - seconds).count())};
    CHECK(setsockopt(socket, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) == 0);
#endif
}

int last_socket_error() {
#ifdef _WIN32
    return WSAGetLastError();
#else
    return errno;
#endif
}

std::string socket_failure_reason(std::int64_t count, int error) {
    if (count == 0) {
        return "orderly_peer_close";
    }
    if (count > 0) {
        return "incomplete_transfer";
    }
#ifdef _WIN32
    const bool timeout = error == WSAETIMEDOUT || error == WSAEWOULDBLOCK;
#else
    const bool timeout = error == EAGAIN || error == EWOULDBLOCK || error == ETIMEDOUT;
#endif
    return timeout ? "timeout" : "socket_error";
}

std::string safe_request_field(std::string value);

std::string format_location(const std::source_location &location) {
    const auto filename = std::filesystem::path(location.file_name()).filename().string();
    return filename + ":" + std::to_string(location.line()) + " (" + location.function_name() + ")";
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
    explicit Client(std::uint16_t port, std::chrono::milliseconds timeout = std::chrono::seconds(2)) {
        initialize_sockets();
        socket_ = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        CHECK(socket_ != kInvalidSocket);
        sockaddr_in address{};
        address.sin_family = AF_INET;
        address.sin_port = htons(port);
        address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        CHECK(::connect(socket_, reinterpret_cast<const sockaddr *>(&address), sizeof(address)) == 0);
        set_timeout(socket_, timeout);
    }

    ~Client() {
        if (socket_ != kInvalidSocket) {
            close_socket(socket_);
        }
    }

    void send_raw(std::string_view line, std::source_location location = std::source_location::current()) {
        record_request(line, location);
        std::string framed(line);
        framed.push_back('\n');
        std::size_t offset = 0;
        while (offset < framed.size()) {
#ifdef _WIN32
            const int count = ::send(socket_, framed.data() + offset, static_cast<int>(framed.size() - offset), 0);
#else
            const auto count = ::send(socket_, framed.data() + offset, framed.size() - offset, MSG_NOSIGNAL);
#endif
            const int error = count < 0 ? last_socket_error() : 0;
            if (count <= 0) {
                throw_socket_failure("send", count, error, offset, location);
            }
            offset += static_cast<std::size_t>(count);
        }
    }

    void send(const Json &request, std::source_location location = std::source_location::current()) {
        auto wire = request;
        const auto secret = wire.value("credential", "");
        wire.erase("credential");
        if (!secret.empty()) {
            const auto payload = wire.dump();
            wire["auth_payload"] = payload;
            wire["auth_proof"] = nomad::runtime::detail::make_proof(secret, "nomad-core:request:v1:" + payload);
        }
        send_raw(wire.dump(), location);
    }

    Json receive(std::source_location location = std::source_location::current()) {
        std::string line;
        char character{};
        while (line.size() <= 65536) {
#ifdef _WIN32
            const int count = ::recv(socket_, &character, 1, 0);
#else
            const auto count = ::recv(socket_, &character, 1, 0);
#endif
            const int error = count < 0 ? last_socket_error() : 0;
            if (count != 1) {
                throw_socket_failure("receive", count, error, line.size(), location);
            }
            if (character == '\n') {
                return Json::parse(line);
            }
            line.push_back(character);
        }
        throw std::runtime_error("response exceeded the test client limit");
    }

    Json request(const Json &body, std::source_location location = std::source_location::current()) {
        send(body, location);
        return receive(location);
    }

    void send_partial(std::string_view data, std::source_location location = std::source_location::current()) {
        record_request(data, location);
#ifdef _WIN32
        const auto count = ::send(socket_, data.data(), static_cast<int>(data.size()), 0);
#else
        const auto count = ::send(socket_, data.data(), data.size(), MSG_NOSIGNAL);
#endif
        const int error = count < 0 ? last_socket_error() : 0;
        if (count < 0 || static_cast<std::size_t>(count) != data.size()) {
            throw_socket_failure("send_partial", count, error, count > 0 ? count : 0, location);
        }
    }

    bool wait_for_peer_close(std::source_location location = std::source_location::current()) {
        fd_set readable;
        FD_ZERO(&readable);
        FD_SET(socket_, &readable);
        // Observe the server's existing three-second idle policy without guessing a sleep.
        timeval deadline{4, 0};
#ifdef _WIN32
        const int selected = select(0, &readable, nullptr, nullptr, &deadline);
#else
        const int selected = select(socket_ + 1, &readable, nullptr, nullptr, &deadline);
#endif
        const int selection_error = selected < 0 ? last_socket_error() : 0;
        if (selected < 0) {
            throw_socket_failure("select", selected, selection_error, 0, location);
        }
        if (selected == 0) {
            return false;
        }
        char byte{};
        const auto count = ::recv(socket_, &byte, 1, MSG_PEEK);
        const int error = count < 0 ? last_socket_error() : 0;
        if (count < 0) {
            throw_socket_failure("peek", count, error, 0, location);
        }
        return count == 0;
    }

    void disconnect() {
        close_socket(socket_);
        socket_ = kInvalidSocket;
    }

  private:
    void record_request(std::string_view line, std::source_location location) {
        request_location_ = location;
        request_id_ = "<unavailable>";
        request_type_ = "<raw>";
        if (line.size() > 65536) {
            return;
        }
        const auto request = Json::parse(line, nullptr, false);
        if (!request.is_object()) {
            return;
        }
        if (request.contains("id") && request["id"].is_string()) {
            request_id_ = safe_request_field(request["id"].get<std::string>());
        }
        if (request.contains("type") && request["type"].is_string()) {
            request_type_ = safe_request_field(request["type"].get<std::string>());
        }
    }

    [[noreturn]] void throw_socket_failure(std::string_view operation, std::int64_t count, int error,
                                          std::size_t bytes, std::source_location location) const {
        throw std::runtime_error("test client " + std::string(operation) + ": " + socket_failure_reason(count, error) +
            "; code=" + std::to_string(error) + "; count=" + std::to_string(count) +
            "; partial_bytes=" + std::to_string(bytes) + "; id=" + request_id_ + "; type=" + request_type_ +
            "; sent_at=" + format_location(request_location_) + "; observed_at=" + format_location(location));
    }

    Socket socket_{kInvalidSocket};
    std::string request_id_{"<unavailable>"};
    std::string request_type_{"<unavailable>"};
    std::source_location request_location_;
};

const std::map<std::string, std::string> test_credentials{
    {"test-client", std::string(64, 'a')}, {"operator", std::string(64, 'b')},
    {"other-client", std::string(64, 'c')}, {"client-a", std::string(64, 'd')},
    {"client-b", std::string(64, 'e')},
};

std::string safe_request_field(std::string value) {
    value = nomad::runtime::detail::redact_credentials(std::move(value), test_credentials);
    for (auto &character : value) {
        if (static_cast<unsigned char>(character) < 32 || character == 127) {
            character = '?';
        }
    }
    return value.substr(0, 80);
}

class TestStorage {
  public:
    TestStorage() {
        std::random_device random;
        path = std::filesystem::temp_directory_path() / ("nomad-audit-tests-" + std::to_string(random()));
        CHECK(std::filesystem::create_directory(path));
    }
    ~TestStorage() {
        std::error_code ignored;
        std::filesystem::remove_all(path, ignored);
    }
    std::filesystem::path path;
    unsigned next{};
};

TestStorage storage;

nomad::runtime::RuntimeConfig test_config() {
    nomad::runtime::RuntimeConfig config;
    config.client_credentials = test_credentials;
    config.audit_directory = (storage.path / std::to_string(++storage.next)).string();
    return config;
}
