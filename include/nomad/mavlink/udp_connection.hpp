// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "nomad/mavlink/connection.hpp"
#include "nomad/mavlink/protocol.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace nomad::mavlink {

class UdpMavlinkConnection final : public MavlinkConnection {
  public:
    explicit UdpMavlinkConnection(std::string endpoint);
    ~UdpMavlinkConnection() override;

    bool connect() override;
    void disconnect() override;
    bool is_connected() const override;
    std::optional<Heartbeat> wait_for_heartbeat(std::chrono::milliseconds timeout) override;
    std::optional<telemetry::VehicleState> wait_for_state(std::chrono::milliseconds timeout) override;
    telemetry::VehicleState get_state() const override;
    std::optional<CommandAck> send_command(const Command &command, std::chrono::milliseconds timeout) override;
    bool send_velocity(const VelocitySetpoint &setpoint) override;
    bool is_velocity_active() const override;
    bool request_data_stream(std::uint8_t stream_id, std::uint16_t message_rate) override;
    bool send_fence_point(const FencePoint &point, std::uint8_t index, std::uint8_t total) override;
    bool request_fence_point(std::uint8_t index) override;
    std::optional<FencePoint> wait_for_fence_point(std::chrono::milliseconds timeout) override;
    bool upload_fence_plan(const std::vector<FencePlanItem> &items) override;
    std::optional<std::vector<FencePlanItem>> download_fence_plan(std::chrono::milliseconds timeout) override;
    std::optional<float> read_param(const std::string &param_id, std::chrono::milliseconds timeout) override;

  private:
    struct Implementation;

    // True when a peer datagram has been received. Kept out of fence.cpp,
    // which cannot name the incomplete pimpl type.
    bool has_peer() const;

    // Locked internals: public entry points serialize on receive_mutex_, so
    // functions that call other public functions internally must use these
    // (otherwise the mutex would be re-acquired and deadlock).
    std::optional<Message> receive_message_locked(std::chrono::milliseconds timeout);
    std::optional<Heartbeat> wait_for_heartbeat_locked(std::chrono::milliseconds timeout);
    // Announce this endpoint as a GCS so heartbeat-gated relays start
    // streaming; sent at most once per second while waiting.
    void send_gcs_heartbeat_locked();
    void apply_heartbeat_locked(const Heartbeat &heartbeat);
    telemetry::VehicleState get_state_locked() const;
    std::optional<Message> receive_message(std::chrono::milliseconds timeout);
    void drain_socket();
    std::size_t send_frame(const std::vector<std::uint8_t> &frame);
    bool parse_endpoint();

    std::string endpoint_;
    std::unique_ptr<Implementation> implementation_;
    std::uint8_t sequence_{0};
    std::uint8_t target_system_{0};
    std::uint8_t target_component_{1};
    telemetry::VehicleState state_{};
    std::chrono::steady_clock::time_point last_heartbeat_{};
    std::chrono::steady_clock::time_point last_gcs_heartbeat_{};
    bool velocity_active_{false};
    // Serializes all access to the receive state below: the pending queue,
    // the decoded state, the latched target, and the send sequence. The
    // vehicle may be consumed concurrently by a telemetry pump and a command
    // waiter (the ROS node's timers and service callbacks), and command
    // acknowledgements must not be stolen between those threads.
    mutable std::mutex receive_mutex_;
    // Datagrams can carry several MAVLink frames (MAVProxy coalesces bursts).
    // Frames decoded ahead of the one returned stay here until the caller asks.
    std::deque<Message> pending_messages_;
};

} // namespace nomad::mavlink
