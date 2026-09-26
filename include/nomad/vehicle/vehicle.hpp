// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "nomad/mavlink/connection.hpp"
#include "nomad/safety/payload.hpp"
#include "nomad/vehicle/config.hpp"
#include "nomad/vehicle/operation.hpp"

#include <array>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

namespace nomad::vehicle {

// Altitude is above home (relative to takeoff point), matching the GCS
// guided-target conventions; takeoff uses the same relative frame.
struct Location {
    double latitude_deg{};
    double longitude_deg{};
    float altitude_m{};
};

struct RouteWaypoint {
    double latitude_deg{};
    double longitude_deg{};
    // Relative to home, matching the GUIDED reposition frame.
    float relative_altitude_m{};
};

struct RecoveryPoint {
    double latitude_deg{};
    double longitude_deg{};
    float relative_altitude_m{};
};

struct LandingPoint {
    double latitude_deg{};
    double longitude_deg{};
};

struct CommandResult {
    bool success{false};
    std::string message;
};

class Vehicle {
  public:
    explicit Vehicle(mavlink::MavlinkConnection &connection, VehicleConfig config = {});
    ~Vehicle();

    Vehicle(const Vehicle &) = delete;
    Vehicle &operator=(const Vehicle &) = delete;

    std::optional<telemetry::VehicleState> wait_for_state(std::chrono::milliseconds timeout);

    // Polls for up to @p timeout and returns the freshest state that satisfies
    // @p is_complete. When none does, returns the freshest state seen at all so
    // a caller can still report partial telemetry; nullopt means no telemetry
    // arrived inside the window.
    std::optional<telemetry::VehicleState>
    wait_for_telemetry(std::chrono::milliseconds timeout,
                       const std::function<bool(const telemetry::VehicleState &)> &is_complete);

    CommandResult arm();
    CommandResult disarm();
    CommandResult set_mode(std::uint32_t custom_mode);
    CommandResult set_guided_mode();
    CommandResult takeoff(float altitude_m);
    CommandResult vtol_takeoff(float altitude_m);
    CommandResult transition_to_fixed_wing();
    CommandResult transition_to_vtol(const RecoveryPoint &point);
    CommandResult quadplane_vtol_land(const LandingPoint &point);
    CommandResult fixed_wing_route(const std::vector<RouteWaypoint> &route);
    CommandResult fixed_wing_recovery(const RecoveryPoint &point);
    CommandResult update_vio(bool healthy, float confidence);
    CommandResult set_velocity(const safety::VelocityCommand &command);
    CommandResult set_servo(int channel, int pwm_microseconds);
    CommandResult set_relay(int relay_number, bool on);
    CommandResult motor_test(int motor_instance, int pwm_microseconds, float timeout_seconds);
    CommandResult configure_gimbal(int mount_mode);
    CommandResult send_user_command(const std::array<float, 7> &parameters);
    CommandResult arm_payload();
    CommandResult release_payload(int relay_number, float duration_seconds);
    CommandResult stop_velocity();
    bool velocity_control_active() const;
    safety::WatchdogReason last_velocity_stop_reason() const;
    CommandResult goto_location(const Location &location);
    CommandResult land();
    CommandResult wait_until_disarmed(std::chrono::milliseconds timeout);
    CommandResult return_to_launch();
    CommandResult upload_fence(const std::vector<safety::GlobalPoint> &boundary);
    CommandResult verify_fence_uploaded(const std::vector<safety::GlobalPoint> &expected_boundary);

  private:
    // A waiter's verdict: nullopt keeps polling, a result stops the wait.
    using StateVerdict = std::function<std::optional<CommandResult>(const telemetry::VehicleState &)>;

    CommandResult send_command(const mavlink::Command &command, const char *name);
    CommandResult send_command(const mavlink::Command &command, const char *name,
                               std::uint64_t expected_session_id);
    CommandResult prepare_vtol_takeoff(const telemetry::VehicleState &initial_state);
    CommandResult arm_for_vtol_takeoff(std::uint64_t expected_session_id);
    CommandResult set_guided_mode_for_vtol_takeoff(std::uint64_t expected_session_id,
                                                   telemetry::AircraftClass aircraft_class);
    CommandResult wait_for_state_until(std::chrono::milliseconds timeout, std::string timeout_message,
                                       const StateVerdict &verdict);
    CommandResult wait_for_armed_state(bool expected, const char *name);
    CommandResult wait_for_mode(std::uint32_t expected, const char *name);
    CommandResult wait_for_mode(const std::function<bool(std::uint32_t)> &matches, const char *name);
    CommandResult send_mode_and_verify(std::uint32_t custom_mode, const char *name);
    CommandResult wait_for_altitude(float minimum_altitude_m, const char *name);
    CommandResult wait_for_vtol_takeoff(float target_altitude_m, const telemetry::VehicleState &expected_state,
                                        std::chrono::steady_clock::time_point ack_boundary);
    std::optional<std::string> vtol_takeoff_identity_error(const telemetry::VehicleState &state,
                                                           const telemetry::VehicleState &expected_state) const;
    std::optional<std::string> vtol_takeoff_state_error(const telemetry::VehicleState &state,
                                                        const telemetry::VehicleState &expected_state) const;
    CommandResult wait_for_fixed_wing_transition(const telemetry::VehicleState &expected_state,
                                                 std::chrono::steady_clock::time_point ack_boundary);
    CommandResult wait_for_transition_ready(const RecoveryPoint &point, std::uint64_t expected_session_id,
                                            std::uint8_t expected_system_id, std::uint8_t expected_component_id,
                                            std::chrono::steady_clock::time_point deadline);
    CommandResult verify_transition_profile(std::uint64_t expected_session_id, std::uint8_t expected_system_id,
                                            std::uint8_t expected_component_id,
                                            std::chrono::steady_clock::time_point deadline);
    CommandResult verify_quadplane_profile(const LandingPoint &point, std::uint64_t expected_session_id,
                                            std::uint8_t expected_system_id, std::uint8_t expected_component_id,
                                            std::chrono::steady_clock::time_point deadline);
    CommandResult verify_quadplane_landing_admission(const LandingPoint &point,
                                                      std::chrono::steady_clock::time_point deadline,
                                                      std::uint64_t &session_id, std::uint8_t &system_id,
                                                      std::uint8_t &component_id, float &entry_altitude_m);
    CommandResult send_quadplane_qland(std::uint64_t expected_session_id,
                                       std::chrono::steady_clock::time_point deadline);
    CommandResult wait_for_quadplane_landing_ready(const LandingPoint &point, std::uint64_t expected_session_id,
                                                    std::uint8_t expected_system_id,
                                                    std::uint8_t expected_component_id,
                                                    std::chrono::steady_clock::time_point deadline);
    CommandResult verify_quadplane_touchdown(const LandingPoint &point, std::uint64_t expected_session_id,
                                              std::uint8_t expected_system_id,
                                              std::uint8_t expected_component_id,
                                              std::chrono::steady_clock::time_point ack_boundary,
                                              std::chrono::steady_clock::time_point deadline,
                                              float entry_altitude_m);
    CommandResult wait_for_multicopter_state(const RecoveryPoint &point, std::uint64_t expected_session_id,
                                              std::uint8_t expected_system_id, std::uint8_t expected_component_id,
                                              std::chrono::steady_clock::time_point ack_boundary,
                                              std::chrono::steady_clock::time_point deadline);
    CommandResult verify_final_multicopter_state(const RecoveryPoint &point, std::uint64_t expected_session_id,
                                                 std::uint8_t expected_system_id, std::uint8_t expected_component_id,
                                                 std::chrono::steady_clock::time_point ack_boundary) const;
    CommandResult wait_for_fixed_wing_waypoint(const RouteWaypoint &waypoint, std::uint64_t expected_session_id,
                                               std::chrono::steady_clock::time_point acknowledgement_boundary,
                                               double acknowledgement_distance_m,
                                               std::chrono::steady_clock::time_point deadline);
    CommandResult execute_fixed_wing_route(const std::vector<RouteWaypoint> &route,
                                           std::uint64_t expected_session_id,
                                           std::chrono::steady_clock::time_point deadline);
    std::optional<std::string> fixed_wing_route_state_error(const telemetry::VehicleState &state,
                                                            std::uint64_t expected_session_id,
                                                            bool require_auto_mode) const;
    std::optional<std::string> vtol_transition_state_error(const telemetry::VehicleState &state,
                                                            const telemetry::VehicleState &expected_state,
                                                            bool require_precondition) const;
    CommandResult wait_for_location(const Location &location);
    CommandResult require_operation(VehicleOperation operation) const;
    // A fresh heartbeat does not imply a fresh position: callers fail closed
    // when this is true rather than trusting a stale fix.
    bool position_is_stale(const telemetry::VehicleState &state) const;
    safety::FlightConditions get_flight_conditions(const telemetry::VehicleState &state,
                                                   std::chrono::steady_clock::time_point now) const;
    safety::WatchdogInput get_watchdog_input(const telemetry::VehicleState &state,
                                             std::chrono::steady_clock::time_point now) const;
    void start_watchdog_locked();
    void run_watchdog();
    bool send_zero_velocity_locked(safety::WatchdogReason reason);

    mavlink::MavlinkConnection &connection_;
    safety::WatchdogPolicy watchdog_policy_;
    safety::GlobalFencePolicy fence_policy_;
    safety::VelocityLimits velocity_limits_;
    std::chrono::milliseconds position_freshness_timeout_;
    // Bounds the authoritative VTOL climb wait; generic Copter takeoff uses its
    // independent fixed deadline.
    std::chrono::milliseconds vtol_takeoff_state_timeout_;
    // Bounds the authoritative fixed-wing transition wait.
    std::chrono::milliseconds transition_state_timeout_;
    // Bounds the full two-point fixed-wing route, including each authoritative
    // position wait after a target request is acknowledged.
    std::chrono::milliseconds fixed_wing_route_timeout_;
    std::chrono::milliseconds fixed_wing_recovery_timeout_;
    std::chrono::milliseconds transition_ready_dwell_;
    std::chrono::milliseconds quadplane_landing_timeout_;
    safety::ReleaseInterlock payload_interlock_;
    mutable std::mutex payload_mutex_;
    mutable std::mutex velocity_mutex_;
    std::condition_variable velocity_condition_;
    std::thread watchdog_thread_;
    bool shutting_down_{false};
    bool velocity_control_active_{false};
    safety::WatchdogReason last_velocity_stop_reason_{safety::WatchdogReason::none};
    std::chrono::steady_clock::time_point last_command_time_{};
    std::chrono::steady_clock::time_point last_vio_update_{};
    bool vio_healthy_{false};
    float vio_confidence_{0.0F};
};

} // namespace nomad::vehicle
