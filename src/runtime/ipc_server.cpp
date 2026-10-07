// SPDX-License-Identifier: Apache-2.0
#include "ipc_server.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <memory>
#include <string>
#include <utility>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <cerrno>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>
#endif

namespace nomad::runtime::detail {
namespace {

#ifdef _WIN32
using NativeSocket = SOCKET;
constexpr NativeSocket kInvalidSocket = INVALID_SOCKET;
#else
using NativeSocket = int;
constexpr NativeSocket kInvalidSocket = -1;
#endif

constexpr std::size_t kMaximumClients = 32;
using Deadline = std::chrono::steady_clock::time_point;

NativeSocket native_socket(std::intptr_t socket) {
    return static_cast<NativeSocket>(socket);
}

std::intptr_t stored_socket(NativeSocket socket) {
    return static_cast<std::intptr_t>(socket);
}

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

void shutdown_socket(NativeSocket socket) {
    if (socket == kInvalidSocket) {
        return;
    }
#ifdef _WIN32
    ::shutdown(socket, SD_BOTH);
#else
    ::shutdown(socket, SHUT_RDWR);
#endif
}

void set_socket_timeout(NativeSocket socket, int milliseconds) {
#ifdef _WIN32
    const DWORD timeout = static_cast<DWORD>(milliseconds);
    setsockopt(socket, SOL_SOCKET, SO_RCVTIMEO, reinterpret_cast<const char *>(&timeout), sizeof(timeout));
    setsockopt(socket, SOL_SOCKET, SO_SNDTIMEO, reinterpret_cast<const char *>(&timeout), sizeof(timeout));
#else
    const timeval timeout{milliseconds / 1000, (milliseconds % 1000) * 1000};
    setsockopt(socket, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    setsockopt(socket, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
#endif
}

bool send_all(NativeSocket socket, std::string_view text) {
    std::size_t sent = 0;
    while (sent < text.size()) {
#ifdef _WIN32
        const int count = ::send(socket, text.data() + sent, static_cast<int>(text.size() - sent), 0);
        if (count == SOCKET_ERROR || count == 0) {
            return false;
        }
#else
        const auto count = ::send(socket, text.data() + sent, text.size() - sent, MSG_NOSIGNAL);
        if (count <= 0) {
            return false;
        }
#endif
        sent += static_cast<std::size_t>(count);
    }
    return true;
}

enum class ReadResult { Line, Closed, TooLarge, Failed };

bool wait_until_readable(NativeSocket socket, Deadline deadline) {
    const auto now = std::chrono::steady_clock::now();
    if (now >= deadline) {
        return false;
    }
    const auto remaining = std::chrono::duration_cast<std::chrono::microseconds>(deadline - now);
    fd_set readable;
    FD_ZERO(&readable);
    FD_SET(socket, &readable);
    timeval wait{static_cast<long>(remaining.count() / 1000000),
                 static_cast<long>(remaining.count() % 1000000)};
#ifdef _WIN32
    const int selected = select(0, &readable, nullptr, nullptr, &wait);
#else
    const int selected = select(socket + 1, &readable, nullptr, nullptr, &wait);
#endif
    return selected > 0 && FD_ISSET(socket, &readable);
}

ReadResult read_line(NativeSocket socket, std::string &line, std::string &buffered) {
    constexpr auto kReadTimeout = std::chrono::seconds(3);
    std::array<char, 2048> buffer{};
    auto newline = buffered.find('\n');
    if (newline != std::string::npos) {
        if (newline > kMaximumMessageBytes) {
            return ReadResult::TooLarge;
        }
        line.assign(buffered, 0, newline);
        buffered.erase(0, newline + 1);
        if (!line.empty() && line.back() == '\r') {
            line.pop_back();
        }
        return ReadResult::Line;
    }
    line = std::move(buffered);
    buffered.clear();
    const auto deadline = std::chrono::steady_clock::now() + kReadTimeout;
    while (true) {
        if (!wait_until_readable(socket, deadline)) {
            return ReadResult::Failed;
        }
#ifdef _WIN32
        const int count = ::recv(socket, buffer.data(), static_cast<int>(buffer.size()), 0);
        if (count == SOCKET_ERROR) {
            return ReadResult::Failed;
        }
#else
        const auto count = ::recv(socket, buffer.data(), buffer.size(), 0);
        if (count < 0) {
            return ReadResult::Failed;
        }
#endif
        if (count == 0) {
            return line.empty() ? ReadResult::Closed : ReadResult::Failed;
        }
        const auto buffer_end = buffer.begin() + count;
        const auto line_break = std::find(buffer.begin(), buffer_end, '\n');
        const auto chunk_size = static_cast<std::size_t>(line_break - buffer.begin());
        if (line.size() + chunk_size > kMaximumMessageBytes) {
            return ReadResult::TooLarge;
        }
        line.append(buffer.data(), chunk_size);
        if (line_break != buffer_end) {
            const auto remainder = static_cast<std::size_t>(count) - chunk_size - 1;
            buffered.assign(buffer.data() + chunk_size + 1, remainder);
            if (line.size() > 0 && line.back() == '\r') {
                line.pop_back();
            }
            return ReadResult::Line;
        }
    }
}

NativeSocket open_listener(std::uint16_t port) {
    if (!initialize_sockets()) {
        return kInvalidSocket;
    }
    const NativeSocket socket = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (socket == kInvalidSocket) {
        return kInvalidSocket;
    }
#ifdef _WIN32
    const BOOL exclusive = TRUE;
    setsockopt(socket, SOL_SOCKET, SO_EXCLUSIVEADDRUSE, reinterpret_cast<const char *>(&exclusive), sizeof(exclusive));
#else
    const int reuse_address = 1;
    setsockopt(socket, SOL_SOCKET, SO_REUSEADDR, &reuse_address, sizeof(reuse_address));
#endif
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(port);
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    if (::bind(socket, reinterpret_cast<const sockaddr *>(&address), sizeof(address)) != 0 ||
        ::listen(socket, 16) != 0) {
        close_socket(socket);
        return kInvalidSocket;
    }
    return socket;
}

std::string oversized_response() {
    return R"({"protocol":"nomad-core","version":1,"id":"","ok":false,"error":)"
           R"({"code":"message_too_large","message":"message exceeds 65536 bytes"}})";
}

} // namespace

IpcServer::~IpcServer() {
    stop();
}

bool IpcServer::start(std::uint16_t port, RequestHandler handler, std::string &error) {
    if (running()) {
        error = "IPC server is already running";
        return false;
    }
    try {
        clients_.reserve(kMaximumClients + 1);
    } catch (...) {
        error = "could not allocate IPC client worker storage";
        return false;
    }
    const auto socket = open_listener(port);
    if (socket == kInvalidSocket) {
        error = "could not bind IPC TCP 127.0.0.1:" + std::to_string(port);
        return false;
    }
    listener_ = stored_socket(socket);
    handler_ = std::move(handler);
    running_ = true;
    try {
        accept_thread_ = std::thread(&IpcServer::accept_clients, this);
    } catch (...) {
        running_ = false;
        close_socket(socket);
        listener_ = -1;
        error = "could not start IPC accept worker";
        return false;
    }
    return true;
}

void IpcServer::stop() {
    running_ = false;
    if (accept_thread_.joinable()) {
        accept_thread_.join();
    }
    join_clients();
    handler_ = {};
}

bool IpcServer::running() const {
    return running_;
}

void IpcServer::accept_clients() {
    const auto listener = native_socket(listener_);
    while (running_) {
        reap_finished_clients();
        fd_set readable;
        FD_ZERO(&readable);
        FD_SET(listener, &readable);
        timeval wait{0, 100000};
#ifdef _WIN32
        const int selected = select(0, &readable, nullptr, nullptr, &wait);
#else
        const int selected = select(listener + 1, &readable, nullptr, nullptr, &wait);
#endif
        if (!running_) {
            break;
        }
        if (selected <= 0 || !FD_ISSET(listener, &readable)) {
            continue;
        }
        const NativeSocket client = ::accept(listener, nullptr, nullptr);
        if (client == kInvalidSocket) {
            continue;
        }
        set_socket_timeout(client, 3000);
        std::shared_ptr<std::atomic_bool> finished;
        try {
            finished = std::make_shared<std::atomic_bool>(false);
        } catch (...) {
            close_socket(client);
            continue;
        }
        std::lock_guard lock(clients_mutex_);
        const auto active = std::count_if(clients_.begin(), clients_.end(), [](const ClientWorker &worker) {
            return !worker.finished->load();
        });
        if (active >= kMaximumClients) {
            close_socket(client);
            continue;
        }
        std::thread worker;
        try {
            worker = std::thread([this, client, finished] {
                serve_client(stored_socket(client));
                close_socket(client);
                finished->store(true);
            });
            clients_.emplace_back();
            clients_.back().finished = std::move(finished);
            clients_.back().thread = std::move(worker);
        } catch (...) {
            if (worker.joinable()) {
                shutdown_socket(client);
                worker.join();
            } else {
                close_socket(client);
            }
        }
    }
    close_socket(listener);
    listener_ = -1;
    join_clients();
}

void IpcServer::serve_client(std::intptr_t stored_client) {
    const auto client = native_socket(stored_client);
    std::string buffered;
    while (running_) {
        std::string request;
        const auto result = read_line(client, request, buffered);
        if (result == ReadResult::Closed) {
            return;
        }
        if (result == ReadResult::TooLarge) {
            send_all(client, oversized_response() + "\n");
            return;
        }
        if (result != ReadResult::Line) {
            return;
        }
        std::string response;
        try {
            response = handler_(request);
        } catch (...) {
            response = R"({"protocol":"nomad-core","version":1,"id":"","ok":false,"error":)"
                       R"({"code":"internal_error","message":"request failed"}})";
        }
        if (response.size() > kMaximumMessageBytes || !send_all(client, response + "\n")) {
            return;
        }
    }
}

void IpcServer::reap_finished_clients() {
    std::lock_guard lock(clients_mutex_);
    auto worker = clients_.begin();
    while (worker != clients_.end()) {
        if (!worker->finished->load()) {
            ++worker;
            continue;
        }
        if (worker->thread.joinable()) {
            worker->thread.join();
        }
        worker = clients_.erase(worker);
    }
}

void IpcServer::join_clients() {
    std::lock_guard lock(clients_mutex_);
    for (auto &worker : clients_) {
        if (worker.thread.joinable()) {
            worker.thread.join();
        }
    }
    clients_.clear();
}

} // namespace nomad::runtime::detail
