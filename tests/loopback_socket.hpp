// SPDX-License-Identifier: Apache-2.0
#pragma once

// Loopback UDP socket helpers shared by the wire-level transport tests.
//
// The connection under test binds a port this helper has just released; the
// reuse race window is negligible for a unit test. Include this header after
// the standard library headers, at the position the previous per-file
// ``#ifdef _WIN32 <winsock2.h>`` blocks used, so winsock2.h still precedes any
// accidental windows.h.

#include "support/test_harness.hpp"

#include <cstdint>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

namespace nomad::test {

#ifdef _WIN32
using Socket = SOCKET;
constexpr Socket kInvalidSocket = INVALID_SOCKET;
#else
using Socket = int;
constexpr Socket kInvalidSocket = -1;
#endif

inline void close_socket(Socket socket) {
#ifdef _WIN32
    closesocket(socket);
#else
    close(socket);
#endif
}

#ifdef _WIN32
// Owns the process-wide Winsock lifetime for a test binary that opens sockets
// before (or without) calling find_free_udp_port.
struct WinsockGuard {
    WinsockGuard() {
        WSADATA data{};
        WSAStartup(MAKEWORD(2, 2), &data);
    }
    ~WinsockGuard() {
        WSACleanup();
    }
};
#endif

// Bind to an ephemeral loopback port, read the assigned number, and release
// it. The connection under test then binds that port.
inline std::uint16_t find_free_udp_port() {
#ifdef _WIN32
    WSADATA data{};
    CHECK(WSAStartup(MAKEWORD(2, 2), &data) == 0);
#endif
    const Socket probe = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    CHECK(probe != kInvalidSocket);
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    address.sin_port = 0;
    CHECK(bind(probe, reinterpret_cast<const sockaddr *>(&address), sizeof(address)) == 0);
    sockaddr_in bound{};
#ifdef _WIN32
    int length = sizeof(bound);
#else
    socklen_t length = sizeof(bound);
#endif
    CHECK(getsockname(probe, reinterpret_cast<sockaddr *>(&bound), &length) == 0);
    const auto port = ntohs(bound.sin_port);
    close_socket(probe);
    return port;
}

} // namespace nomad::test
