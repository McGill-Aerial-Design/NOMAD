// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors

#pragma once

// Private declaration of the MAVSDK-backed MavlinkConnection. Kept in
// src/ so the public surface stays the make_mavsdk_connection() factory in
// nomad/mavlink/mavsdk_transport.hpp.

#include "nomad/mavlink/connection.hpp"

#include <mavsdk/mavsdk.hpp>
#include <plugins/action/action.hpp>
#include <plugins/geofence/geofence.hpp>
#include <plugins/mavlink_passthrough/mavlink_passthrough.hpp>
#include <plugins/offboard/offboard.hpp>
#include <plugins/param/param.hpp>
#include <plugins/telemetry/telemetry.hpp>

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <shared_mutex>
#include <string>
#include <vector>

namespace nomad::mavlink {

std::optional<std::uint8_t> mavsdk_command_result_code(mavsdk::MavlinkPassthrough::Result result);

using ObservationClock = std::chrono::steady_clock;

constexpr auto kHeartbeatTimeout = std::chrono::seconds(3);
constexpr std::uint8_t kAutopilotComponent = 1; // MAV_COMP_ID_AUTOPILOT1
constexpr std::uint8_t kArmModeFlag = 0x80;     // MAV_MODE_FLAG_SAFETY_ARMED

// Holds the observation lock for one telemetry update and always wakes the
// waiters on release, so no observer can forget to notify.
class ObservationUpdate {
  public:
    ObservationUpdate(std::mutex &mutex, std::condition_variable &changed) : lock_(mutex), changed_(changed) {}
    ~ObservationUpdate() { changed_.notify_all(); }

    ObservationUpdate(const ObservationUpdate &) = delete;
    ObservationUpdate &operator=(const ObservationUpdate &) = delete;

  private:
    std::lock_guard<std::mutex> lock_;
    std::condition_variable &changed_;
};

class MavsdkMavlinkConnection final : public MavlinkConnection {
  public:
    MavsdkMavlinkConnection(std::string endpoint, std::uint8_t expected_system_id,
                            std::chrono::milliseconds discovery_timeout);
    ~MavsdkMavlinkConnection() override;

    MavsdkMavlinkConnection(const MavsdkMavlinkConnection &) = delete;
    MavsdkMavlinkConnection &operator=(const MavsdkMavlinkConnection &) = delete;

    bool connect() override;
    void disconnect() override;
    bool is_connected() const override;
    ConnectFailure get_connect_failure() const override;
    std::optional<Heartbeat> wait_for_heartbeat(std::chrono::milliseconds timeout) override;
    std::optional<telemetry::VehicleState> wait_for_state(std::chrono::milliseconds timeout) override;
    telemetry::VehicleState get_state() const override;
    std::optional<CommandAck> send_command(const Command &command, std::chrono::milliseconds timeout) override;
    std::optional<CommandAck> send_command(const Command &command, std::uint64_t expected_session_id,
                                           std::chrono::milliseconds timeout) override;
    bool goto_location_relative(double latitude_deg, double longitude_deg, float relative_altitude_m,
                                std::chrono::milliseconds timeout) override;
    std::optional<CommandAck> send_fixed_wing_waypoint(
        const FixedWingWaypointCommand &waypoint, std::uint64_t expected_session_id,
        std::chrono::milliseconds timeout) override;
    bool send_velocity(const VelocitySetpoint &setpoint) override;
    bool is_velocity_active() const override;
    bool send_fence_point(const FencePoint &point, std::uint8_t index, std::uint8_t total) override;
    bool request_fence_point(std::uint8_t index) override;
    std::optional<FencePoint> wait_for_fence_point(std::chrono::milliseconds timeout) override;
    bool upload_fence_plan(const std::vector<FencePlanItem> &items) override;
    std::optional<std::vector<FencePlanItem>> download_fence_plan(std::chrono::milliseconds timeout) override;
    std::optional<float> read_param(const std::string &param_id, std::chrono::milliseconds timeout) override;
    std::optional<AutopilotVersion> read_autopilot_version(std::chrono::milliseconds timeout) override;

  private:
    bool is_connected_unlocked() const;
    telemetry::VehicleState state_locked() const;

    void subscribe();
    void unsubscribe();
    void close();
    bool select_system();
    void identify_quadplane_from_parameters(ObservationClock::time_point deadline);

    void observe_heartbeat(const mavlink_message_t &message);
    void observe_position(const mavsdk::Telemetry::Position &position);
    void observe_velocity(const mavsdk::Telemetry::VelocityNed &velocity);
    void observe_battery(const mavsdk::Telemetry::Battery &battery);
    void observe_gps(const mavsdk::Telemetry::GpsInfo &gps);
    void observe_attitude(const mavsdk::Telemetry::EulerAngle &attitude);
    void observe_vtol_state(mavsdk::Telemetry::VtolState state);
    void observe_landed_state(mavsdk::Telemetry::LandedState state);

    mavsdk::MavlinkPassthrough::Result send_long(const Command &command, std::chrono::milliseconds timeout,
                                                 const TransmissionAdmission &admission);
    mavsdk::Offboard::Result queue_velocity_setpoint(const VelocitySetpoint &setpoint);

    std::string endpoint_;
    std::uint8_t expected_system_id_{0};
    std::chrono::milliseconds discovery_timeout_{0};
    ConnectFailure connect_failure_{ConnectFailure::None};
    mavsdk::Mavsdk sdk_;
    mutable std::shared_mutex plugin_lifetime_mutex_;
    std::optional<mavsdk::Mavsdk::ConnectionHandle> handle_;
    std::shared_ptr<mavsdk::System> system_;
    std::unique_ptr<mavsdk::Action> action_;
    std::unique_ptr<mavsdk::Telemetry> telemetry_;
    std::unique_ptr<mavsdk::MavlinkPassthrough> passthrough_;
    std::unique_ptr<mavsdk::Geofence> geofence_;
    std::unique_ptr<mavsdk::Param> param_;
    std::unique_ptr<mavsdk::Offboard> offboard_;

    mutable std::mutex observation_mutex_;
    std::condition_variable observation_changed_;
    telemetry::VehicleState state_;
    std::uint64_t session_id_counter_{0};
    // ArduPlane QuadPlanes report MAV_TYPE_FIXED_WING. Q_ENABLE is the
    // authoritative discriminator; no value means identity is unresolved.
    std::optional<bool> quadplane_enabled_;
    // True while the last accepted setpoint was non-zero, so shutdown knows the
    // vehicle is still being steered and must be zeroed. Guarded by
    // observation_mutex_ with the rest of the observed state.
    bool velocity_active_{false};
    std::optional<Heartbeat> heartbeat_;
    ObservationClock::time_point last_heartbeat_{};
    std::uint8_t target_system_{0};
    std::uint8_t target_component_{kAutopilotComponent};

    // Subscription handles, kept so disconnect can stop callbacks before the
    // owning plugins and system are destroyed.
    std::optional<mavsdk::Telemetry::PositionHandle> position_handle_;
    std::optional<mavsdk::Telemetry::VelocityNedHandle> velocity_handle_;
    std::optional<mavsdk::Telemetry::BatteryHandle> battery_handle_;
    std::optional<mavsdk::Telemetry::GpsInfoHandle> gps_handle_;
    std::optional<mavsdk::Telemetry::AttitudeEulerHandle> attitude_handle_;
    std::optional<mavsdk::Telemetry::VtolStateHandle> vtol_state_handle_;
    std::optional<mavsdk::Telemetry::LandedStateHandle> landed_state_handle_;
    std::optional<mavsdk::MavlinkPassthrough::MessageHandle> heartbeat_handle_;
    std::optional<mavsdk::System::IsConnectedHandle> connection_handle_;
};

} // namespace nomad::mavlink
