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
        auto wire = request;
        const auto secret = wire.value("credential", "");
        wire.erase("credential");
        if (!secret.empty()) {
            const auto payload = wire.dump();
            wire["auth_payload"] = payload;
            wire["auth_proof"] = nomad::runtime::detail::make_proof(secret, "nomad-core:request:v1:" + payload);
        }
        send_raw(wire.dump());
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

const std::map<std::string, std::string> test_credentials{
    {"test-client", std::string(64, 'a')}, {"operator", std::string(64, 'b')},
    {"other-client", std::string(64, 'c')}, {"client-a", std::string(64, 'd')},
    {"client-b", std::string(64, 'e')},
};

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
