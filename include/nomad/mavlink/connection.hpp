// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "nomad/telemetry/state.hpp"

#include <array>
#include <chrono>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace nomad::mavlink {

struct Command {
    std::uint16_t id{};
    std::array<float, 7> parameters{};
    // The vehicle may require current observation facts at every covered send.
    std::function<bool(const telemetry::VehicleState &)> state_admission{};
};

struct CommandAck {
    enum class Status {
        Acknowledged,
        AdmissionCancelled,
    };

    std::uint16_t command{};
    std::uint8_t result{};
    Status status{Status::Acknowledged};
};

struct VelocitySetpoint {
    float vx{};
    float vy{};
    float vz{};
    float yaw_rate{};
};

struct Heartbeat {
    std::uint8_t system_id{};
    std::uint8_t component_id{};
    std::uint32_t custom_mode{};
    std::uint8_t vehicle_type{};
    std::uint8_t autopilot_type{};
    std::uint8_t base_mode{};
};

struct FencePoint {
    float latitude_deg{};
    float longitude_deg{};
};

struct FenceStatus {
    std::uint8_t breach_status{};
    std::uint16_t breach_count{};
    std::uint8_t breach_type{};
    std::uint32_t breach_time{};
};

struct FencePlanItem {
    FencePoint point{};
    std::uint16_t sequence{};
    std::uint16_t command{};
    // For a polygon vertex item ArduPilot reads the boundary's total vertex
    // count from param1; every vertex of one polygon carries the same count.
    float param1{};
};

struct FixedWingWaypointCommand {
    double latitude_deg{};
    double longitude_deg{};
    float relative_altitude_m{};
    float loiter_radius_m{};
};

struct ParamValue {
    std::string param_id;
    float value{};
};

struct AutopilotVersion {
    std::uint8_t major{};
    std::uint8_t minor{};
    std::uint8_t patch{};
    std::string git_hash;
};

struct SendAuthorityToken {
    std::string runtime_incarnation;
    std::uint64_t vehicle_session{};
    std::uint64_t authority_generation{};
    std::string source;
    std::string request_id;
    std::uint64_t sequence{};
    std::int64_t expires_at_ms{};
};

using TransmissionAdmission = std::function<bool(const std::function<void()> &)>;
using TransmissionAdmissionFactory = std::function<TransmissionAdmission()>;

// Which half of connect() failed. Opening the endpoint and finding an expected
// autopilot are different operator problems, so the caller reports the client
// diagnostic that matches instead of collapsing both into one message.
enum class ConnectFailure {
    None,
    LinkUnavailable, // the endpoint could not be opened as configured
    NoAutopilot,     // the link opened but no expected autopilot answered
};

class MavlinkConnection {
  public:
    virtual ~MavlinkConnection() = default;

    void set_transmission_admission_factory(TransmissionAdmissionFactory factory) {
        transmission_admission_factory_ = std::move(factory);
    }

    void set_vehicle_session_changed_handler(std::function<void(std::uint64_t)> handler) {
        vehicle_session_changed_handler_ = std::move(handler);
    }

    virtual bool connect() = 0;
    virtual void disconnect() = 0;
    virtual bool is_connected() const = 0;
    // Reads back why the most recent connect() returned false; None once a
    // connect() has succeeded.
    virtual ConnectFailure get_connect_failure() const = 0;
    virtual std::optional<Heartbeat> wait_for_heartbeat(std::chrono::milliseconds timeout) = 0;
    virtual std::optional<telemetry::VehicleState> wait_for_state(std::chrono::milliseconds timeout) = 0;
    virtual telemetry::VehicleState get_state() const = 0;
    virtual std::optional<CommandAck> send_command(const Command &command, std::chrono::milliseconds timeout) = 0;
    virtual std::optional<CommandAck> send_command(const Command &command, std::uint64_t expected_session_id,
                                                   std::chrono::milliseconds timeout) {
        if (expected_session_id == 0 || get_state().session_id != expected_session_id) {
            return std::nullopt;
        }
        return send_command(command, timeout);
    }
    virtual bool goto_location_relative(double latitude_deg, double longitude_deg, float relative_altitude_m,
                                        std::chrono::milliseconds timeout) = 0;
    virtual std::optional<CommandAck> send_fixed_wing_waypoint(
        const FixedWingWaypointCommand &waypoint, std::uint64_t expected_session_id,
        std::chrono::milliseconds timeout) = 0;
    virtual bool send_velocity(const VelocitySetpoint &setpoint) = 0;
    virtual bool is_velocity_active() const = 0;
    virtual bool send_fence_point(const FencePoint &point, std::uint8_t index, std::uint8_t total) = 0;
    virtual bool request_fence_point(std::uint8_t index) = 0;
    virtual std::optional<FencePoint> wait_for_fence_point(std::chrono::milliseconds timeout) = 0;
    virtual bool upload_fence_plan(const std::vector<FencePlanItem> &items) = 0;
    virtual std::optional<std::vector<FencePlanItem>> download_fence_plan(std::chrono::milliseconds timeout) = 0;
    // Reads a named parameter back from the autopilot. The returned value is
    // authoritative autopilot state, not an acknowledgement.
    virtual std::optional<float> read_param(const std::string &param_id, std::chrono::milliseconds timeout) = 0;
    virtual std::optional<AutopilotVersion> read_autopilot_version(std::chrono::milliseconds timeout) = 0;

  protected:
    bool admit_send() const {
        const auto admission = capture_transmission_admission();
        return !admission || admission([] {});
    }

    TransmissionAdmission capture_transmission_admission() const {
        if (!transmission_admission_factory_) {
            return {};
        }
        return transmission_admission_factory_();
    }

    void notify_vehicle_session_changed(std::uint64_t session) const {
        if (vehicle_session_changed_handler_) {
            vehicle_session_changed_handler_(session);
        }
    }

  private:
    TransmissionAdmissionFactory transmission_admission_factory_;
    std::function<void(std::uint64_t)> vehicle_session_changed_handler_;
};

} // namespace nomad::mavlink
