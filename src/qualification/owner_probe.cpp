// SPDX-License-Identifier: Apache-2.0
#include "owner_probe.hpp"

#include <charconv>
#include <cstdlib>
#include <string_view>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#else
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

bool runtime_endpoint_is_open() {
    unsigned int port = 14611;
    if (const char *configured = std::getenv("NOMAD_RUNTIME_IPC_PORT")) {
        const std::string_view value(configured);
        const auto parsed = std::from_chars(value.data(), value.data() + value.size(), port);
        if (parsed.ec != std::errc{} || parsed.ptr != value.data() + value.size() || port == 0 || port > 65535) {
            return true;
        }
    }
#ifdef _WIN32
    WSADATA data{};
    if (WSAStartup(MAKEWORD(2, 2), &data) != 0) {
        return true;
    }
#endif
    const auto socket = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(static_cast<unsigned short>(port));
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    const bool occupied = ::bind(socket, reinterpret_cast<const sockaddr *>(&address), sizeof(address)) != 0;
#ifdef _WIN32
    closesocket(socket);
    WSACleanup();
#else
    ::close(socket);
#endif
    return occupied;
}
