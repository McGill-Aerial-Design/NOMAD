// SPDX-License-Identifier: Apache-2.0
#include "nomad/mavlink/udp_connection.hpp"

#include "nomad/mavlink/protocol.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <span>
#include <string_view>
#include <utility>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <netdb.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

namespace nomad::mavlink {
namespace {

// Matches the GCS identity of the command senders in udp_commands.cpp.
constexpr std::uint8_t kSourceSystem = 255;
constexpr std::uint8_t kSourceComponent = 190;
constexpr auto kHeartbeatTimeout = std::chrono::seconds(3);
constexpr std::size_t kReceiveBufferSize = 2048;

#ifdef _WIN32
using Socket = SOCKET;
using AddressLength = int;
constexpr Socket kInvalidSocket = INVALID_SOCKET;
#else
using Socket = int;
using AddressLength = socklen_t;
constexpr Socket kInvalidSocket = -1;
#endif

void close_socket(Socket socket) {
#ifdef _WIN32
    closesocket(socket);
#else
    close(socket);
#endif
}

bool split_endpoint(std::string_view endpoint, std::string &host, std::string &port, bool &listen) {
    listen = true;
    const auto scheme_end = endpoint.find(':');
    if (scheme_end != std::string_view::npos) {
        const auto scheme = endpoint.substr(0, scheme_end);
        if (scheme == "udpout") {
            listen = false;
        } else if (scheme != "udp" && scheme != "udpin") {
            return false;
        }
        endpoint.remove_prefix(scheme_end + 1);
    }

    const auto port_start = endpoint.rfind(':');
    if (port_start == std::string_view::npos) {
        return false;
    }
    host = std::string(endpoint.substr(0, port_start));
    port = std::string(endpoint.substr(port_start + 1));
    return !host.empty() && !port.empty();
}

bool wait_for_socket(Socket socket, std::chrono::milliseconds timeout) {
    timeval value{};
    value.tv_sec = static_cast<long>(timeout.count() / 1000);
    value.tv_usec = static_cast<long>((timeout.count() % 1000) * 1000);
    fd_set readable;
    FD_ZERO(&readable);
    FD_SET(socket, &readable);
#ifdef _WIN32
    return select(0, &readable, nullptr, nullptr, &value) > 0;
#else
    return select(socket + 1, &readable, nullptr, nullptr, &value) > 0;
#endif
}

bool has_telemetry(const telemetry::VehicleState &state) {
    return state.position_valid || state.battery_valid || state.gps_valid || state.attitude_valid;
}

// Resolve a UDP host:port pair into a send/bind address (IPv4 datagram).
bool resolve_udp_address(const std::string &host, const std::string &port, sockaddr_storage &address,
                         AddressLength &address_size) {
    addrinfo hints{};
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_DGRAM;
    hints.ai_protocol = IPPROTO_UDP;
    addrinfo *result = nullptr;
    if (getaddrinfo(host.c_str(), port.c_str(), &hints, &result) != 0 || result == nullptr) {
        return false;
    }
    address_size = static_cast<AddressLength>(result->ai_addrlen);
    std::memcpy(&address, result->ai_addr, result->ai_addrlen);
    freeaddrinfo(result);
    return true;
}

// Resolve the NOMAD_RELAY_ADDRESS override for pre-latch GCS-heartbeat
// announcements (a relay behind a separate gateway IP is not reachable through
// a wildcard listen endpoint; see docs/operations.md). Returns false when the
// variable is unset (use the endpoint); target_size 0 means malformed.
bool announcement_override(AddressLength &target_size, sockaddr_storage &target) {
    const char *configured = std::getenv("NOMAD_RELAY_ADDRESS");
    if (configured == nullptr || configured[0] == '\0') {
        return false;
    }
    std::string host;
    std::string port;
    bool listen = true;
    const bool malformed = !split_endpoint(configured, host, port, listen) ||
                           !resolve_udp_address(host, port, target, target_size);
    target_size = malformed ? 0 : target_size;
    return true;
}

} // namespace

struct UdpMavlinkConnection::Implementation {
    Socket socket{kInvalidSocket};
    sockaddr_storage endpoint{};
    AddressLength endpoint_size{0};
    sockaddr_storage peer{};
    AddressLength peer_size{0};
    bool connected{false};
    bool listen{true};
    bool has_peer{false};
    // NOMAD_RELAY_ADDRESS override for pre-latch GCS-heartbeat announcements.
    sockaddr_storage announcement_target{};
    AddressLength announcement_size{0};
    bool has_announcement_target{false};
#ifdef _WIN32
    bool winsock_started{false};
#endif
};

UdpMavlinkConnection::UdpMavlinkConnection(std::string endpoint)
    : endpoint_(std::move(endpoint)), implementation_(std::make_unique<Implementation>()) {}

UdpMavlinkConnection::~UdpMavlinkConnection() {
    disconnect();
}

bool UdpMavlinkConnection::parse_endpoint() {
    std::string host;
    std::string port;
    if (!split_endpoint(endpoint_, host, port, implementation_->listen) ||
        !resolve_udp_address(host, port, implementation_->endpoint, implementation_->endpoint_size)) {
        return false;
    }
    return true;
}

bool UdpMavlinkConnection::connect() {
    if (is_connected()) {
        return true;
    }

#ifdef _WIN32
    WSADATA data{};
    if (WSAStartup(MAKEWORD(2, 2), &data) != 0) {
        return false;
    }
    implementation_->winsock_started = true;
#endif

    if (!parse_endpoint()) {
        disconnect();
        return false;
    }

    implementation_->socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (implementation_->socket == kInvalidSocket) {
        disconnect();
        return false;
    }

    // A busy SITL link can stream thousands of telemetry datagrams per second.
    // A small OS receive buffer would drop the command acknowledgement under
    // that flood, so ask for a large buffer where the platform allows it.
    constexpr int kReceiveBufferBytes = 1 << 20;
    static_cast<void>(setsockopt(implementation_->socket, SOL_SOCKET, SO_RCVBUF,
                                 reinterpret_cast<const char *>(&kReceiveBufferBytes), sizeof(kReceiveBufferBytes)));

    const auto result = implementation_->listen
                            ? bind(implementation_->socket, reinterpret_cast<const sockaddr *>(&implementation_->endpoint),
                                   implementation_->endpoint_size)
                            : ::connect(implementation_->socket,
                                        reinterpret_cast<const sockaddr *>(&implementation_->endpoint),
                                        implementation_->endpoint_size);
    if (result != 0) {
        disconnect();
        return false;
    }
    // A malformed NOMAD_RELAY_ADDRESS fails closed: a silent wrong target
    // would strand the link behind a relay that never opens.
    if (announcement_override(implementation_->announcement_size, implementation_->announcement_target) &&
        implementation_->announcement_size == 0) {
        disconnect();
        return false;
    }
    implementation_->has_announcement_target = implementation_->announcement_size != 0;
    if (!implementation_->listen) {
        implementation_->peer = implementation_->endpoint;
        implementation_->peer_size = implementation_->endpoint_size;
        implementation_->has_peer = true;
    }
    implementation_->connected = true;
    return true;
}

void UdpMavlinkConnection::disconnect() {
    // Send the zero setpoint before clearing the latched target (send_velocity
    // refuses to send with target_system_ == 0).
    if (is_velocity_active()) {
        send_velocity({});
    }
    if (implementation_->socket != kInvalidSocket) {
        close_socket(implementation_->socket);
        implementation_->socket = kInvalidSocket;
    }
    implementation_->connected = false;
    implementation_->has_peer = false;
    {
        std::lock_guard lock(receive_mutex_);
        pending_messages_.clear();
        target_system_ = 0;
        target_component_ = 1;
        state_ = {};
        last_heartbeat_ = {};
        velocity_active_ = false;
    }
#ifdef _WIN32
    if (implementation_->winsock_started) {
        WSACleanup();
        implementation_->winsock_started = false;
    }
#endif
}

bool UdpMavlinkConnection::is_connected() const {
    return implementation_->connected;
}

void UdpMavlinkConnection::drain_socket() {
    // Read until the socket is momentarily empty so a busy link never strands
    // the command acknowledgement behind a full OS receive buffer.
    while (wait_for_socket(implementation_->socket, std::chrono::milliseconds(0))) {
        std::array<std::uint8_t, kReceiveBufferSize> buffer{};
        sockaddr_storage sender{};
        AddressLength sender_size = sizeof(sender);
        const auto received = recvfrom(implementation_->socket, reinterpret_cast<char *>(buffer.data()),
                                       static_cast<int>(buffer.size()), 0, reinterpret_cast<sockaddr *>(&sender),
                                       &sender_size);
        if (received <= 0) {
            break;
        }

        // Decode every frame in the datagram, not just the first: MAVProxy
        // coalesces bursts of telemetry into single UDP packets, and the
        // command acknowledgement is often the second or later frame.
        const auto datagram =
            std::span<const std::uint8_t>(buffer).first(static_cast<std::size_t>(received));
        std::size_t offset = 0;
        while (offset < datagram.size()) {
            std::size_t consumed = 0;
            auto message = decode_datagram(datagram.subspan(offset), consumed);
            if (!message.has_value()) {
                break;
            }
            offset += consumed;
            if (implementation_->listen && !implementation_->has_peer) {
                // Latch the peer from any decoded frame, not just heartbeats:
                // heartbeat-gated relays (mavlink-router, MAVProxy) reply to
                // the exact source address they received from, and the vehicle
                // may start streaming telemetry before its next heartbeat.
                const auto heartbeat = decode_heartbeat(*message);
                if (heartbeat.has_value() && accepts_heartbeat(*heartbeat, target_system_)) {
                    implementation_->peer = sender;
                    implementation_->peer_size = sender_size;
                    implementation_->has_peer = true;
                }
            }
            pending_messages_.push_back(std::move(*message));
        }
    }
}

std::optional<Message> UdpMavlinkConnection::receive_message(std::chrono::milliseconds timeout) {
    std::lock_guard lock(receive_mutex_);
    return receive_message_locked(timeout);
}

std::optional<Message> UdpMavlinkConnection::receive_message_locked(std::chrono::milliseconds timeout) {
    if (!is_connected()) {
        return std::nullopt;
    }
    // Drain new datagrams even when the queue is non-empty; otherwise heartbeats strand behind a backlog.
    if (wait_for_socket(implementation_->socket, std::chrono::milliseconds(0))) {
        drain_socket();
    }
    if (pending_messages_.empty()) {
        if (!wait_for_socket(implementation_->socket, timeout)) {
            return std::nullopt;
        }
        drain_socket();
    }
    if (pending_messages_.empty()) {
        return std::nullopt;
    }
    auto message = std::move(pending_messages_.front());
    pending_messages_.pop_front();
    const auto heartbeat = decode_heartbeat(message);
    if (heartbeat.has_value() && accepts_heartbeat(*heartbeat, target_system_)) {
        apply_heartbeat_locked(*heartbeat);
    }
    return message;
}

bool UdpMavlinkConnection::has_peer() const {
    return implementation_->has_peer;
}

std::optional<Heartbeat> UdpMavlinkConnection::wait_for_heartbeat(std::chrono::milliseconds timeout) {
    std::lock_guard lock(receive_mutex_);
    return wait_for_heartbeat_locked(timeout);
}

void UdpMavlinkConnection::send_gcs_heartbeat_locked() {
    // Many MAVLink relays and routers only start streaming a UDP leg once the
    // endpoint announces itself; a 1 Hz GCS heartbeat opens that gate without
    // touching vehicle state (it is a status frame, not a command).
    const auto now = std::chrono::steady_clock::now();
    if (now - last_gcs_heartbeat_ < std::chrono::seconds(1)) {
        return;
    }
    last_gcs_heartbeat_ = now;
    const auto frame = encode_gcs_heartbeat(sequence_++, kSourceSystem, kSourceComponent);
    if (frame.empty()) {
        return;
    }
    // Before the peer latches, announce to a routable target: the configured
    // NOMAD_RELAY_ADDRESS when set, otherwise the endpoint (a wildcard
    // 0.0.0.0 host falls back to loopback for same-host relays).
    if (!implementation_->has_peer) {
        sockaddr_storage target{};
        AddressLength target_size = 0;
        if (implementation_->has_announcement_target) {
            target = implementation_->announcement_target;
            target_size = implementation_->announcement_size;
        } else {
            target = implementation_->endpoint;
            target_size = implementation_->endpoint_size;
            auto *address = reinterpret_cast<sockaddr_in *>(&target);
            if (address->sin_addr.s_addr == htonl(INADDR_ANY)) {
                address->sin_addr.s_addr = htonl(INADDR_LOOPBACK);
            }
        }
        static_cast<void>(sendto(implementation_->socket,
                                 reinterpret_cast<const char *>(frame.data()),
                                 static_cast<int>(frame.size()), 0,
                                 reinterpret_cast<const sockaddr *>(&target),
                                 target_size));
        return;
    }
    static_cast<void>(send_frame(frame));
}

std::optional<Heartbeat> UdpMavlinkConnection::wait_for_heartbeat_locked(std::chrono::milliseconds timeout) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        send_gcs_heartbeat_locked();
        const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - std::chrono::steady_clock::now());
        const auto message = receive_message_locked((std::max)(std::chrono::milliseconds(1), remaining));
        if (!message.has_value()) {
            continue;
        }
        const auto heartbeat = decode_heartbeat(*message);
        if (!heartbeat.has_value() || !accepts_heartbeat(*heartbeat, target_system_)) {
            continue;
        }
        apply_heartbeat_locked(*heartbeat);
        return heartbeat;
    }
    return std::nullopt;
}

std::optional<telemetry::VehicleState> UdpMavlinkConnection::wait_for_state(std::chrono::milliseconds timeout) {
    std::lock_guard lock(receive_mutex_);
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        // Consume every queued frame so a slow sampler never strands heartbeats.
        if (wait_for_socket(implementation_->socket, std::chrono::milliseconds(0))) {
            drain_socket();
        }
        bool consumed_any = false;
        while (!pending_messages_.empty()) {
            auto message = std::move(pending_messages_.front());
            pending_messages_.pop_front();
            consumed_any = true;
            const auto heartbeat = decode_heartbeat(message);
            if (heartbeat.has_value() && !accepts_heartbeat(*heartbeat, target_system_)) {
                continue;
            }
            if (target_system_ == 0) {
                if (!heartbeat.has_value() || !accepts_heartbeat(*heartbeat, 0)) {
                    continue;
                }
                target_system_ = heartbeat->system_id;
                target_component_ = heartbeat->component_id;
            }
            if (message.system_id != target_system_) {
                continue;
            }
            update_vehicle_state(message, state_);
            if (message.message_id == 0) {
                state_.heartbeat_fresh = true;
                last_heartbeat_ = std::chrono::steady_clock::now();
            }
        }
        if (consumed_any) {
            const auto current_state = get_state_locked();
            if (current_state.connected && has_telemetry(current_state)) {
                return current_state;
            }
        }
        // Nothing usable yet: wait briefly for the next datagram and retry.
        // Announce ourselves as a GCS so heartbeat-gated relays start streaming.
        send_gcs_heartbeat_locked();
        wait_for_socket(implementation_->socket, std::chrono::milliseconds(20));
    }
    return std::nullopt;
}

bool UdpMavlinkConnection::is_velocity_active() const {
    std::lock_guard lock(receive_mutex_);
    return velocity_active_;
}

telemetry::VehicleState UdpMavlinkConnection::get_state() const {
    std::lock_guard lock(receive_mutex_);
    return get_state_locked();
}

void UdpMavlinkConnection::apply_heartbeat_locked(const Heartbeat &heartbeat) {
    if (target_system_ == 0) {
        target_system_ = heartbeat.system_id;
        target_component_ = heartbeat.component_id;
    }
    state_.connected = true;
    state_.heartbeat_fresh = true;
    state_.system_id = heartbeat.system_id;
    state_.component_id = heartbeat.component_id;
    state_.custom_mode = heartbeat.custom_mode;
    state_.armed = (heartbeat.base_mode & 0x80) != 0;
    last_heartbeat_ = std::chrono::steady_clock::now();
}

telemetry::VehicleState UdpMavlinkConnection::get_state_locked() const {
    auto state = state_;
    if (last_heartbeat_ == std::chrono::steady_clock::time_point{}) {
        state.heartbeat_fresh = false;
        state.connected = false;
        return state;
    }
    const auto elapsed = std::chrono::steady_clock::now() - last_heartbeat_;
    state.heartbeat_fresh = elapsed <= kHeartbeatTimeout;
    state.connected = state.heartbeat_fresh;
    return state;
}

std::size_t UdpMavlinkConnection::send_frame(const std::vector<std::uint8_t> &frame) {
    if (!implementation_->has_peer) {
        return 0;
    }
    // POSIX send/sendto take const void*; Windows takes const char*.
    const auto sent = implementation_->listen
                          ? sendto(implementation_->socket, reinterpret_cast<const char *>(frame.data()),
                                   static_cast<int>(frame.size()), 0,
                                   reinterpret_cast<const sockaddr *>(&implementation_->peer),
                                   implementation_->peer_size)
                          : send(implementation_->socket, reinterpret_cast<const char *>(frame.data()),
                                 static_cast<int>(frame.size()), 0);
    return sent < 0 ? 0 : static_cast<std::size_t>(sent);
}

} // namespace nomad::mavlink
