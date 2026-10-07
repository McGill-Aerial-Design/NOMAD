// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors

#include "nomad/mavlink/mavlink_observation.hpp"

#include <mavlink/ardupilotmega/mavlink.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <limits>
#include <mutex>
#include <string>
#include <thread>
#include <utility>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <cerrno>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>
#endif

namespace nomad::mavlink {
namespace {

using Clock = std::chrono::steady_clock;

#ifdef _WIN32
using NativeSocket = SOCKET;
constexpr NativeSocket kInvalidSocket = INVALID_SOCKET;
#else
using NativeSocket = int;
constexpr NativeSocket kInvalidSocket = -1;
#endif

constexpr auto kHeartbeatTimeout = std::chrono::seconds(3);
constexpr auto kReceiveTimeout = std::chrono::milliseconds(100);

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

bool set_receive_timeout(NativeSocket socket) {
#ifdef _WIN32
    const DWORD timeout_ms = static_cast<DWORD>(kReceiveTimeout.count());
    return setsockopt(socket, SOL_SOCKET, SO_RCVTIMEO, reinterpret_cast<const char *>(&timeout_ms),
                      sizeof(timeout_ms)) == 0;
#else
    const timeval timeout{0, static_cast<suseconds_t>(kReceiveTimeout.count() * 1000)};
    return setsockopt(socket, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) == 0;
#endif
}

bool is_receive_timeout() {
#ifdef _WIN32
    const auto error = WSAGetLastError();
    return error == WSAETIMEDOUT || error == WSAEWOULDBLOCK;
#else
    return errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR;
#endif
}

struct UdpBindAddress {
    std::string host;
    std::string port;
};

std::optional<UdpBindAddress> parse_bind_address(std::string_view endpoint) {
    const auto canonical = mavsdk_phase_a::canonicalize_udp_input_endpoint(endpoint);
    if (!canonical) {
        return {};
    }
    const std::string_view address(*canonical);
    auto rest = address.substr(std::string_view("udpin://").size());
    const auto separator = rest.rfind(':');
    if (separator == std::string_view::npos) {
        return {};
    }
    auto host = rest.substr(0, separator);
    if (host.starts_with('[') && host.ends_with(']')) {
        host = host.substr(1, host.size() - 2);
    }
    return UdpBindAddress{std::string(host), std::string(rest.substr(separator + 1))};
}

NativeSocket create_udp_input_socket(const UdpBindAddress &address) {
    if (!initialize_sockets()) {
        return kInvalidSocket;
    }
    addrinfo hints{};
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_DGRAM;
    hints.ai_protocol = IPPROTO_UDP;
    hints.ai_flags = AI_PASSIVE;
    const char *host = address.host == "0.0.0.0" || address.host == "::" ? nullptr : address.host.c_str();
    addrinfo *candidates = nullptr;
    if (getaddrinfo(host, address.port.c_str(), &hints, &candidates) != 0) {
        return kInvalidSocket;
    }

    auto selected = kInvalidSocket;
    for (auto *candidate = candidates; candidate != nullptr; candidate = candidate->ai_next) {
        const auto socket = ::socket(candidate->ai_family, candidate->ai_socktype, candidate->ai_protocol);
        if (socket == kInvalidSocket) {
            continue;
        }
        if (::bind(socket, candidate->ai_addr, static_cast<int>(candidate->ai_addrlen)) == 0 &&
            set_receive_timeout(socket)) {
            selected = socket;
            break;
        }
        close_socket(socket);
    }
    freeaddrinfo(candidates);
    return selected;
}

bool is_expected_autopilot_message(const mavlink_message_t &message, std::uint8_t expected_system_id) {
    return message.sysid == expected_system_id && message.compid == MAV_COMP_ID_AUTOPILOT1;
}

double get_voltage_v(std::uint16_t millivolts) {
    if (millivolts == std::numeric_limits<std::uint16_t>::max()) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return static_cast<double>(millivolts) / 1000.0;
}

double get_battery_percent(std::int8_t percentage) {
    if (percentage < 0) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return static_cast<double>(percentage);
}

bool is_newer_boot_time(std::uint32_t candidate, std::uint32_t previous) {
    return static_cast<std::int32_t>(candidate - previous) > 0;
}

} // namespace

struct MavlinkObservation::Implementation {
    explicit Implementation(MavlinkObservationOptions input) : options(std::move(input)) {}

    MavlinkObservationOptions options;
    NativeSocket socket{kInvalidSocket};
    std::thread receiver;
    std::atomic<bool> stopping{};
    mutable std::mutex mutex;
    std::condition_variable heartbeat_seen;
    mavsdk_phase_a::StatusValues values;
    mavlink_message_t message_buffer{};
    mavlink_status_t parser_status{};
    mavlink_status_t message_status{};
    std::uint32_t last_position_boot_ms{};
    std::uint64_t last_gps_time_usec{};
    std::uint8_t last_message_sequence{};
    std::size_t position_updates{};
    bool has_position_boot_time{};
    bool has_gps_time{};
    bool saw_other_autopilot{};
    bool socket_failed{};
    bool has_message_sequence{};
    Clock::time_point first_position{};
    Clock::time_point last_position{};
    Clock::time_point last_gps{};
    Clock::time_point last_battery{};
    Clock::time_point last_heartbeat{};
    MavlinkObservationError error{MavlinkObservationError::None};

    void reset_observation() {
        std::scoped_lock lock(mutex);
        values = {};
        message_buffer = {};
        parser_status = {};
        message_status = {};
        last_position_boot_ms = 0;
        last_gps_time_usec = 0;
        last_message_sequence = 0;
        position_updates = 0;
        has_position_boot_time = false;
        has_gps_time = false;
        saw_other_autopilot = false;
        socket_failed = false;
        has_message_sequence = false;
        first_position = {};
        last_position = {};
        last_gps = {};
        last_battery = {};
        last_heartbeat = {};
    }

    void observe_position(const mavlink_message_t &message) {
        mavlink_global_position_int_t position{};
        mavlink_msg_global_position_int_decode(&message, &position);
        const auto now = Clock::now();
        std::scoped_lock lock(mutex);
        if (has_position_boot_time && !is_newer_boot_time(position.time_boot_ms, last_position_boot_ms)) {
            return;
        }
        values.latitude_deg = static_cast<double>(position.lat) / 1e7;
        values.longitude_deg = static_cast<double>(position.lon) / 1e7;
        values.relative_altitude_m = static_cast<double>(position.relative_alt) / 1000.0;
        values.absolute_altitude_m = static_cast<double>(position.alt) / 1000.0;
        if (position_updates == 0) {
            first_position = now;
        }
        last_position_boot_ms = position.time_boot_ms;
        has_position_boot_time = true;
        last_position = now;
        ++position_updates;
    }

    void observe_gps(const mavlink_message_t &message) {
        mavlink_gps_raw_int_t gps{};
        mavlink_msg_gps_raw_int_decode(&message, &gps);
        const auto now = Clock::now();
        std::scoped_lock lock(mutex);
        if (has_gps_time && gps.time_usec <= last_gps_time_usec) {
            return;
        }
        values.gps_fix_type = static_cast<int>(gps.fix_type);
        values.satellites = static_cast<int>(gps.satellites_visible);
        last_gps_time_usec = gps.time_usec;
        has_gps_time = true;
        last_gps = now;
    }

    void observe_battery(const mavlink_message_t &message) {
        mavlink_sys_status_t battery{};
        mavlink_msg_sys_status_decode(&message, &battery);
        std::scoped_lock lock(mutex);
        values.battery_voltage_v = get_voltage_v(battery.voltage_battery);
        values.battery_remaining_percent = get_battery_percent(battery.battery_remaining);
        last_battery = Clock::now();
    }

    void observe_message(const mavlink_message_t &message) {
        if (message.sysid == options.expected_system_id && message.compid == MAV_COMP_ID_AUTOPILOT1 &&
            !accept_sequence(message)) {
            return;
        }
        if (message.msgid == MAVLINK_MSG_ID_HEARTBEAT && message.compid == MAV_COMP_ID_AUTOPILOT1) {
            mavlink_heartbeat_t heartbeat{};
            mavlink_msg_heartbeat_decode(&message, &heartbeat);
            std::scoped_lock lock(mutex);
            if (message.sysid != options.expected_system_id) {
                saw_other_autopilot = true;
                return;
            }
            if (heartbeat.autopilot == MAV_AUTOPILOT_INVALID) {
                return;
            }
            values.flight_mode = heartbeat.custom_mode != 0 ? static_cast<int>(heartbeat.custom_mode)
                                                            : static_cast<int>(heartbeat.base_mode);
            last_heartbeat = Clock::now();
            heartbeat_seen.notify_all();
            return;
        }
        if (!is_expected_autopilot_message(message, options.expected_system_id)) {
            return;
        }
        if (message.msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT) {
            observe_position(message);
        } else if (message.msgid == MAVLINK_MSG_ID_GPS_RAW_INT) {
            observe_gps(message);
        } else if (message.msgid == MAVLINK_MSG_ID_SYS_STATUS) {
            observe_battery(message);
        }
    }

    bool accept_sequence(const mavlink_message_t &message) {
        const auto now = Clock::now();
        std::scoped_lock lock(mutex);
        if (has_message_sequence && now - last_heartbeat <= kHeartbeatTimeout) {
            const auto sequence_delta = static_cast<std::uint8_t>(message.seq - last_message_sequence);
            if (sequence_delta == 0 || sequence_delta >= 128) {
                return false;
            }
        }
        last_message_sequence = message.seq;
        has_message_sequence = true;
        return true;
    }

    void parse_datagram(const std::uint8_t *data, std::size_t size) {
        for (std::size_t index = 0; index < size; ++index) {
            mavlink_message_t parsed{};
            const auto result = mavlink_frame_char_buffer(&message_buffer, &parser_status, data[index], &parsed,
                                                          &message_status);
            if (result == MAVLINK_FRAMING_OK) {
                observe_message(parsed);
            }
        }
    }

    void receive_loop() {
        std::array<std::uint8_t, 65536> buffer{};
        while (!stopping.load()) {
            const auto received = recvfrom(socket, reinterpret_cast<char *>(buffer.data()),
                                           static_cast<int>(buffer.size()), 0, nullptr, nullptr);
            if (received < 0) {
                if (stopping.load() || is_receive_timeout()) {
                    continue;
                }
                std::scoped_lock lock(mutex);
                socket_failed = true;
                heartbeat_seen.notify_all();
                return;
            }
            if (received > 0) {
                parse_datagram(buffer.data(), static_cast<std::size_t>(received));
            }
        }
    }

    bool wait_for_heartbeat() {
        const auto deadline = Clock::now() + options.discovery_timeout;
        std::unique_lock lock(mutex);
        heartbeat_seen.wait_until(lock, deadline, [this] {
            return last_heartbeat != Clock::time_point{} || socket_failed || stopping.load();
        });
        if (last_heartbeat != Clock::time_point{}) {
            return true;
        }
        if (socket_failed) {
            error = MavlinkObservationError::AddConnectionFailed;
        } else {
            error = saw_other_autopilot ? MavlinkObservationError::WrongPeer
                                        : MavlinkObservationError::DiscoveryTimeout;
        }
        return false;
    }

    void close() {
        stopping.store(true);
        heartbeat_seen.notify_all();
        if (receiver.joinable()) {
            receiver.join();
        }
        close_socket(socket);
        socket = kInvalidSocket;
        reset_observation();
    }
};

MavlinkObservation::MavlinkObservation(MavlinkObservationOptions options)
    : implementation_(std::make_unique<Implementation>(std::move(options))) {}

MavlinkObservation::~MavlinkObservation() {
    disconnect();
}

bool MavlinkObservation::connect() {
    auto &implementation = *implementation_;
    implementation.close();
    implementation.error = MavlinkObservationError::None;
    const auto address = parse_bind_address(implementation.options.endpoint);
    if (!address || implementation.options.expected_system_id == 0 ||
        implementation.options.discovery_timeout <= std::chrono::milliseconds::zero()) {
        implementation.error = MavlinkObservationError::InvalidConfiguration;
        return false;
    }
    implementation.socket = create_udp_input_socket(*address);
    if (implementation.socket == kInvalidSocket) {
        implementation.error = MavlinkObservationError::AddConnectionFailed;
        return false;
    }
    implementation.stopping.store(false);
    implementation.receiver = std::thread([&implementation] { implementation.receive_loop(); });
    if (implementation.wait_for_heartbeat()) {
        return true;
    }
    implementation.close();
    return false;
}

void MavlinkObservation::disconnect() {
    implementation_->close();
}

bool MavlinkObservation::is_connected() const {
    std::scoped_lock lock(implementation_->mutex);
    return !implementation_->socket_failed && implementation_->last_heartbeat != Clock::time_point{} &&
           Clock::now() - implementation_->last_heartbeat <= kHeartbeatTimeout;
}

std::uint8_t MavlinkObservation::system_id() const {
    return is_connected() ? implementation_->options.expected_system_id : 0;
}

MavlinkObservationError MavlinkObservation::last_error() const {
    return implementation_->error;
}

MavlinkObservationSnapshot MavlinkObservation::get_status() const {
    const auto now = Clock::now();
    std::scoped_lock lock(implementation_->mutex);
    const auto &observation = *implementation_;
    MavlinkObservationSnapshot snapshot{observation.values, observation.position_updates, 0, 0, {}, {}};
    if (observation.position_updates > 0) {
        const auto window = observation.last_position - observation.first_position;
        snapshot.observation_ms = std::chrono::duration_cast<std::chrono::milliseconds>(window).count();
        const auto age = now - observation.last_position;
        snapshot.age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(age).count();
    }
    if (observation.last_gps != Clock::time_point{}) {
        snapshot.gps_age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - observation.last_gps).count();
    }
    if (observation.last_battery != Clock::time_point{}) {
        snapshot.battery_age_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(now - observation.last_battery).count();
    }
    return snapshot;
}

std::optional<MavlinkObservationSnapshot>
MavlinkObservation::wait_for_status(std::chrono::milliseconds timeout) const {
    const auto deadline = Clock::now() + timeout;
    while (Clock::now() < deadline && is_connected()) {
        const auto snapshot = get_status();
        if (mavsdk_phase_a::has_valid_status(snapshot.values) &&
            mavsdk_phase_a::has_fresh_position_stream(snapshot.position_updates, snapshot.observation_ms,
                                                      snapshot.age_ms)) {
            return snapshot;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return {};
}

std::string_view mavlink_observation_error_message(MavlinkObservationError error) {
    switch (error) {
    case MavlinkObservationError::None:
        return "none";
    case MavlinkObservationError::InvalidConfiguration:
        return "invalid MAVLink UDP input endpoint or expected system ID";
    case MavlinkObservationError::AddConnectionFailed:
        return "could not bind the MAVLink telemetry observer socket";
    case MavlinkObservationError::DiscoveryTimeout:
        return "timed out waiting for expected autopilot heartbeat";
    case MavlinkObservationError::WrongPeer:
        return "received an autopilot heartbeat from a different system ID";
    }
    return "unknown MAVLink observer error";
}

} // namespace nomad::mavlink
