// SPDX-License-Identifier: Apache-2.0
#include "nomad/mavlink/protocol.hpp"
#include "nomad/mavlink/udp_connection.hpp"

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <netdb.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

namespace {

#ifdef _WIN32
using Socket = SOCKET;
constexpr Socket kInvalidSocket = INVALID_SOCKET;
#else
using Socket = int;
constexpr Socket kInvalidSocket = -1;
#endif

void close_socket(Socket socket) {
#ifdef _WIN32
    closesocket(socket);
#else
    close(socket);
#endif
}

// A failing assert on Windows opens a dialog that blocks unattended CI runs,
// so the checks below throw instead of calling assert().
void check_impl(bool ok, const char *condition, int line) {
    if (!ok) {
        throw std::runtime_error(std::string("check failed at line ") + std::to_string(line) + ": " + condition);
    }
}

#define CHECK(condition) check_impl(static_cast<bool>(condition), #condition, __LINE__)

// Bind to an ephemeral loopback port, read the assigned number, and release
// it. The connection under test then binds that port. The reuse race window
// is negligible for a unit test.
std::uint16_t find_free_udp_port() {
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

// A plain loopback UDP socket that plays the role of the remote MAVLink link.
// Bind to an explicit port when the test needs the connection's pre-latch
// announcements to land here; otherwise use an ephemeral port.
Socket open_peer_on_port(std::uint16_t port) {
    const Socket peer = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    CHECK(peer != kInvalidSocket);
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    address.sin_port = htons(port);
    CHECK(bind(peer, reinterpret_cast<const sockaddr *>(&address), sizeof(address)) == 0);
    return peer;
}

Socket open_peer() {
    return open_peer_on_port(0);
}

void send_to_port(Socket peer, std::uint16_t port, const std::vector<std::uint8_t> &datagram) {
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    address.sin_port = htons(port);
    const auto sent = sendto(peer, reinterpret_cast<const char *>(datagram.data()),
                             static_cast<int>(datagram.size()), 0, reinterpret_cast<const sockaddr *>(&address),
                             sizeof(address));
    CHECK(sent == static_cast<int>(datagram.size()));
}

std::vector<std::uint8_t> heartbeat_frame(std::uint8_t sequence, std::uint8_t system_id) {
    std::vector<std::uint8_t> payload(9, 0);
    payload[4] = 2;  // MAV_TYPE_QUADROTOR
    payload[5] = 3;  // MAV_AUTOPILOT_ARDUPILOTMEGA
    payload[6] = 1;  // MAV_MODE_FLAG_SAFETY_ARMED is bit 7; keep it disabled
    payload[8] = 3;  // MAVLink protocol version
    return nomad::mavlink::encode_message(sequence, system_id, 1, 0, payload, true).value();
}

std::vector<std::uint8_t> sys_status_frame(std::uint8_t sequence, std::uint8_t system_id) {
    return nomad::mavlink::encode_message(sequence, system_id, 1, 1, std::vector<std::uint8_t>(31), true).value();
}

std::vector<std::uint8_t> command_ack_frame(std::uint8_t sequence, std::uint8_t system_id, std::uint16_t command,
                                            std::uint8_t result) {
    std::vector<std::uint8_t> payload(3, 0);
    payload[0] = static_cast<std::uint8_t>(command & 0xff);
    payload[1] = static_cast<std::uint8_t>(command >> 8);
    payload[2] = result;
    return nomad::mavlink::encode_message(sequence, system_id, 1, 77, payload, true).value();
}

// MAVProxy coalesces bursts of MAVLink frames into single UDP datagrams.
// Join the given frames into one datagram, in order.
std::vector<std::uint8_t> coalesce(std::initializer_list<std::vector<std::uint8_t>> frames) {
    std::vector<std::uint8_t> datagram;
    for (const auto &frame : frames) {
        datagram.insert(datagram.end(), frame.begin(), frame.end());
    }
    return datagram;
}

// Read every datagram currently pending on a socket without blocking.
std::vector<std::vector<std::uint8_t>> drain_pending(Socket peer) {
    std::vector<std::vector<std::uint8_t>> datagrams;
    for (;;) {
        u_long pending = 0;
#ifdef _WIN32
        CHECK(ioctlsocket(peer, FIONREAD, &pending) == 0);
#else
        CHECK(ioctl(peer, FIONREAD, &pending) == 0);
#endif
        if (pending == 0) {
            return datagrams;
        }
        std::vector<std::uint8_t> buffer(pending);
        const auto count = recvfrom(peer, reinterpret_cast<char *>(buffer.data()), static_cast<int>(buffer.size()), 0,
                                    nullptr, nullptr);
        CHECK(count == static_cast<int>(pending));
        datagrams.push_back(std::move(buffer));
    }
}

// Set NOMAD_RELAY_ADDRESS for one scope and always restore the previous
// environment, so a failing check cannot leak the override into other tests.
class RelayAddressOverride {
  public:
    explicit RelayAddressOverride(const char *value) {
        const char *existing = std::getenv("NOMAD_RELAY_ADDRESS");
        if (existing != nullptr) {
            previous_ = existing;
        }
#ifdef _WIN32
        CHECK(_putenv_s("NOMAD_RELAY_ADDRESS", value == nullptr ? "" : value) == 0);
#else
        CHECK(setenv("NOMAD_RELAY_ADDRESS", value == nullptr ? "" : value, 1) == 0);
#endif
    }
    ~RelayAddressOverride() {
#ifdef _WIN32
        static_cast<void>(_putenv_s("NOMAD_RELAY_ADDRESS", previous_.c_str()));
#else
        static_cast<void>(setenv("NOMAD_RELAY_ADDRESS", previous_.c_str(), 1));
#endif
    }
    RelayAddressOverride(const RelayAddressOverride &) = delete;
    RelayAddressOverride &operator=(const RelayAddressOverride &) = delete;

  private:
    std::string previous_;
};

void test_coalesced_heartbeat_is_found() {
    std::fputs("[udp] coalesced heartbeat\n", stderr);
    const auto port = find_free_udp_port();
    nomad::mavlink::UdpMavlinkConnection connection("udp:127.0.0.1:" + std::to_string(port));
    CHECK(connection.connect());
    const Socket peer = open_peer();

    // The heartbeat is the second frame of the datagram; a decoder that only
    // reads the first frame would never see it.
    const auto datagram = coalesce({sys_status_frame(1, 1), heartbeat_frame(2, 1)});
    send_to_port(peer, port, datagram);

    const auto heartbeat = connection.wait_for_heartbeat(std::chrono::seconds(2));

    CHECK(heartbeat.has_value());
    CHECK(heartbeat->system_id == 1);
    close_socket(peer);
    connection.disconnect();
}

// A caller that samples the link slowly (a ROS telemetry timer at a few
// hertz) must not let the pending queue fall behind the stream. Every
// wait_for_state call consumes all queued frames, so heartbeats are read
// promptly and an on-demand freshness check never reports a healthy link as
// stale.
void test_slow_sampler_keeps_heartbeat_fresh() {
    std::fputs("[udp] slow sampler\n", stderr);
    const auto port = find_free_udp_port();
    nomad::mavlink::UdpMavlinkConnection connection("udp:127.0.0.1:" + std::to_string(port));
    CHECK(connection.connect());
    const Socket peer = open_peer();

    send_to_port(peer, port, heartbeat_frame(1, 1));
    CHECK(connection.wait_for_heartbeat(std::chrono::seconds(2)).has_value());

    // Flood the link with telemetry (plus a heartbeat every ~100 ms) for six
    // seconds — far faster than the 5 Hz sampling below and long enough that
    // a sampler which never reaches the queued heartbeats goes stale (> 3 s).
    std::atomic<bool> done{false};
    std::thread flooder([&] {
        std::uint8_t sequence = 10;
        std::size_t tick = 0;
        const auto until = std::chrono::steady_clock::now() + std::chrono::seconds(6);
        while (std::chrono::steady_clock::now() < until) {
            send_to_port(peer, port,
                         coalesce({sys_status_frame(sequence++, 1), sys_status_frame(sequence++, 1),
                                   sys_status_frame(sequence++, 1), sys_status_frame(sequence++, 1)}));
            if (tick % 10 == 0) {  // one heartbeat every 100 ms
                send_to_port(peer, port, heartbeat_frame(sequence++, 1));
            }
            ++tick;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        done = true;
    });

    // Sample like the ROS node: one bounded wait every 200 ms.
    while (!done.load()) {
        static_cast<void>(connection.wait_for_state(std::chrono::milliseconds(50)));
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    // Consume the tail of the burst, then require the link to look fresh.
    static_cast<void>(connection.wait_for_state(std::chrono::milliseconds(50)));

    const auto state = connection.get_state();
    flooder.join();
    CHECK(state.heartbeat_fresh);
    CHECK(state.connected);
    close_socket(peer);
    connection.disconnect();
}

void test_coalesced_ack_is_received() {
    std::fputs("[udp] coalesced ack\n", stderr);
    const auto port = find_free_udp_port();
    nomad::mavlink::UdpMavlinkConnection connection("udp:127.0.0.1:" + std::to_string(port));
    CHECK(connection.connect());
    const Socket peer = open_peer();

    // Establish the link with a single-frame heartbeat so the connection
    // learns the peer address and target system.
    send_to_port(peer, port, heartbeat_frame(1, 1));
    CHECK(connection.wait_for_heartbeat(std::chrono::seconds(2)).has_value());

    // The peer answers the command with a coalesced datagram whose second
    // frame is the acknowledgement, exactly what MAVProxy produces under a
    // telemetry burst.
    std::thread responder([&] {
        std::array<std::uint8_t, 2048> buffer{};
        sockaddr_storage sender{};
#ifdef _WIN32
        int sender_size = sizeof(sender);
#else
        socklen_t sender_size = sizeof(sender);
#endif
        const auto received = recvfrom(peer, reinterpret_cast<char *>(buffer.data()),
                                       static_cast<int>(buffer.size()), 0,
                                       reinterpret_cast<sockaddr *>(&sender), &sender_size);
        CHECK(received > 0);
        const auto reply = coalesce({sys_status_frame(2, 1), command_ack_frame(3, 1, 400, 0)});
        const auto sent = sendto(peer, reinterpret_cast<const char *>(reply.data()), static_cast<int>(reply.size()), 0,
                                 reinterpret_cast<const sockaddr *>(&sender), sender_size);
        CHECK(sent == static_cast<int>(reply.size()));
    });

    const auto acknowledgement = connection.send_command(
        nomad::mavlink::Command{400, {1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F}}, std::chrono::seconds(2));
    responder.join();

    CHECK(acknowledgement.has_value());
    CHECK(acknowledgement->command == 400);
    CHECK(acknowledgement->result == 0);
    close_socket(peer);
    connection.disconnect();
}

// A heartbeat-gated relay (mavlink-router, Docker's UDP relay, MAVProxy)
// only streams its UDP leg after it sees a GCS heartbeat from our endpoint.
// Before any datagram arrives the connection has no latched peer, so it must
// announce itself: standard GCS heartbeats (sysid 255, compid 190,
// MAV_TYPE_GCS) at ~1 Hz, sent to the configured endpoint (or the
// NOMAD_RELAY_ADDRESS override). This is the behavior that unblocks
// heartbeat-gated links, so it is pinned here end to end.
void test_unlatched_connection_sends_gcs_heartbeats() {
    std::fputs("[udp] gcs heartbeat emission\n", stderr);
    // Reserve one loopback port, bind the peer to it, and point the
    // connection at that port with udpout:. Before the peer latches, the
    // connection sends every announcement to the configured endpoint —
    // exactly this peer socket. The peer stays silent so it never latches as
    // a vehicle and the connection must keep announcing.
    const auto peer_port = find_free_udp_port();
    nomad::mavlink::UdpMavlinkConnection connection("udpout:127.0.0.1:" + std::to_string(peer_port));
    CHECK(connection.connect());
    const Socket peer = open_peer_on_port(peer_port);

    // The peer stays silent so the connection never latches it and must keep
    // announcing to the configured endpoint. The heartbeats are only emitted
    // while the caller is waiting on the link, so keep making short
    // wait_for_heartbeat calls (as a real client would) and collect what
    // arrives on the peer socket between waits.
    std::vector<std::vector<std::uint8_t>> received;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    while (received.size() < 2 && std::chrono::steady_clock::now() < deadline) {
        static_cast<void>(connection.wait_for_heartbeat(std::chrono::milliseconds(300)));
        for (;;) {
            u_long pending = 0;
#ifdef _WIN32
            CHECK(ioctlsocket(peer, FIONREAD, &pending) == 0);
#else
            CHECK(ioctl(peer, FIONREAD, &pending) == 0);
#endif
            if (pending == 0) {
                break;
            }
            std::vector<std::uint8_t> buffer(pending);
            const auto count = recvfrom(peer, reinterpret_cast<char *>(buffer.data()), static_cast<int>(buffer.size()),
                                        0, nullptr, nullptr);
            CHECK(count == static_cast<int>(pending));
            received.emplace_back(std::move(buffer));
        }
    }

    CHECK(received.size() >= 2);  // 1 Hz cadence maintained
    const auto message = nomad::mavlink::decode_message(received.front());
    CHECK(message.has_value());
    CHECK(message->message_id == 0);      // HEARTBEAT
    CHECK(message->system_id == 255);     // GCS source system
    CHECK(message->component_id == 190);  // GCS source component
    CHECK(message->payload.size() >= 9);
    CHECK(message->payload[4] == 6);  // MAV_TYPE_GCS
    CHECK(message->payload[5] == 8);  // MAV_AUTOPILOT_INVALID

    close_socket(peer);
    connection.disconnect();
}

// A relay behind a separate gateway IP (Docker Desktop's UDP proxy, LTE
// routers) is not reachable through the configured listen endpoint. The
// NOMAD_RELAY_ADDRESS override must carry every pre-latch announcement to
// that relay, and stop once a vehicle latches as the peer.
void test_relay_address_override_targets_prelatch_announcements() {
    std::fputs("[udp] relay address override\n", stderr);
    const auto relay_port = find_free_udp_port();
    const Socket relay = open_peer_on_port(relay_port);
    const Socket vehicle = open_peer();

    {
        RelayAddressOverride override(("udpout:127.0.0.1:" + std::to_string(relay_port)).c_str());
        // A wildcard bind endpoint: without the override, pre-latch
        // announcements would go to loopback instead of the relay.
        const auto bind_port = find_free_udp_port();
        nomad::mavlink::UdpMavlinkConnection connection("udpin:0.0.0.0:" + std::to_string(bind_port));
        CHECK(connection.connect());

        static_cast<void>(connection.wait_for_heartbeat(std::chrono::milliseconds(300)));
        auto announced = drain_pending(relay);
        CHECK(!announced.empty());  // override used: announcement landed on the relay
        for (const auto &frame : announced) {
            const auto message = nomad::mavlink::decode_message(frame);
            CHECK(message.has_value());
            CHECK(message->message_id == 0);
            CHECK(message->system_id == 255);
        }

        // Once the vehicle latches as the peer, announcements stop and the
        // override must not divert command traffic from the latched peer.
        send_to_port(vehicle, bind_port, heartbeat_frame(1, 1));
        CHECK(connection.wait_for_heartbeat(std::chrono::seconds(2)).has_value());
        static_cast<void>(connection.wait_for_heartbeat(std::chrono::milliseconds(300)));
        for (const auto &frame : drain_pending(relay)) {
            const auto message = nomad::mavlink::decode_message(frame);
            CHECK(message.has_value() && message->message_id != 0);
        }

        connection.disconnect();
    }
    close_socket(vehicle);
    close_socket(relay);
}

// A malformed NOMAD_RELAY_ADDRESS would silently strand the link behind a
// relay that never opens, so connect() must fail closed on one.
void test_invalid_relay_address_fails_closed() {
    std::fputs("[udp] invalid relay address refused\n", stderr);
    const RelayAddressOverride override("udpout:not-a-remote-host:99999");
    const auto port = find_free_udp_port();
    nomad::mavlink::UdpMavlinkConnection connection("udpin:0.0.0.0:" + std::to_string(port));
    CHECK(!connection.connect());
}

} // namespace

int main() {
    try {
        test_coalesced_heartbeat_is_found();
        test_unlatched_connection_sends_gcs_heartbeats();
        test_relay_address_override_targets_prelatch_announcements();
        test_invalid_relay_address_fails_closed();
        test_coalesced_ack_is_received();
        test_slow_sampler_keeps_heartbeat_fresh();
    } catch (const std::exception &error) {
        std::fprintf(stderr, "FAILED: %s\n", error.what());
        return 1;
    }
    return 0;
}
