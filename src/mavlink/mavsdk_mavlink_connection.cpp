// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
//
// MAVSDK-backed MavlinkConnection. MAVSDK owns framing,
// transport and its internal workers; NOMAD keeps command authorization and
// authoritative outcome verification. Guided goto uses MAVSDK Action; remaining
// commands without a suitable high-level API use COMMAND_LONG passthrough.

#include "mavsdk_mavlink_connection.hpp"

#include "mavsdk_system.hpp"
#include "nomad/mavlink/mavsdk_transport.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace nomad::mavlink {
namespace {

constexpr auto kTelemetryWaitIncrement = std::chrono::milliseconds(20);
constexpr auto kQuadplaneParameterTimeout = std::chrono::milliseconds(2000);

bool has_telemetry(const telemetry::VehicleState &state) {
    return state.position_valid || state.battery_valid || state.gps_valid || state.attitude_valid ||
           state.vtol_state_valid || state.landed_state_valid;
}

bool has_configuration(const std::string &endpoint, std::uint8_t expected_system_id,
                       std::chrono::milliseconds discovery_timeout) {
    return mavsdk_phase_a::canonicalize_udp_endpoint(endpoint).has_value() && expected_system_id != 0 &&
           discovery_timeout > std::chrono::milliseconds::zero();
}

std::optional<std::chrono::milliseconds> remaining_timeout(ObservationClock::time_point deadline) {
    const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - ObservationClock::now());
    if (remaining <= std::chrono::milliseconds::zero()) {
        return std::nullopt;
    }
    return remaining;
}

} // namespace

MavsdkMavlinkConnection::MavsdkMavlinkConnection(std::string endpoint, std::uint8_t expected_system_id,
                                                 std::chrono::milliseconds discovery_timeout)
    : endpoint_(std::move(endpoint)),
      expected_system_id_(expected_system_id),
      discovery_timeout_(discovery_timeout),
      sdk_(mavsdk::Mavsdk::Configuration{mavsdk::ComponentType::GroundStation}) {}

MavsdkMavlinkConnection::~MavsdkMavlinkConnection() {
    disconnect();
}

bool MavsdkMavlinkConnection::connect() {
    if (is_connected()) {
        connect_failure_ = ConnectFailure::None;
        return true;
    }
    close();
    connect_failure_ = ConnectFailure::LinkUnavailable;
    if (!has_configuration(endpoint_, expected_system_id_, discovery_timeout_)) {
        return false;
    }
    const auto endpoint = mavsdk_phase_a::canonicalize_udp_endpoint(endpoint_);
    auto [result, handle] = sdk_.add_any_connection_with_handle(*endpoint);
    if (result != mavsdk::ConnectionResult::Success) {
        return false;
    }
    // The endpoint is open, so from here a failure means no autopilot answered.
    connect_failure_ = ConnectFailure::NoAutopilot;
    handle_ = handle;
    const auto deadline = ObservationClock::now() + discovery_timeout_;
    while (ObservationClock::now() < deadline) {
        if (select_system()) {
            identify_quadplane_from_parameters(deadline);
            connect_failure_ = ConnectFailure::None;
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    close();
    return false;
}

ConnectFailure MavsdkMavlinkConnection::get_connect_failure() const {
    return connect_failure_;
}

void MavsdkMavlinkConnection::disconnect() {
    // Zero the vehicle while the target is still latched, then tear the link
    // down: a last setpoint left on the wire would keep steering a vehicle NOMAD
    // has stopped controlling.
    if (is_velocity_active()) {
        send_velocity({});
    }
    close();
}

bool MavsdkMavlinkConnection::select_system() {
    mavsdk_phase_a::SystemSelection selection{};
    const auto candidate = mavsdk_system::select_expected_autopilot(sdk_, expected_system_id_, selection);
    if (candidate == nullptr) {
        return false;
    }
    system_ = candidate;
    target_system_ = expected_system_id_;
    target_component_ = kAutopilotComponent;
    {
        ObservationUpdate update(observation_mutex_, observation_changed_);
        ++session_id_counter_;
        if (session_id_counter_ == 0) {
            ++session_id_counter_;
        }
        notify_vehicle_session_changed(session_id_counter_);
        state_.session_id = session_id_counter_;
    }
    subscribe();
    return true;
}

void MavsdkMavlinkConnection::identify_quadplane_from_parameters(ObservationClock::time_point deadline) {
    const auto heartbeat_timeout = remaining_timeout(deadline);
    if (!heartbeat_timeout || !wait_for_heartbeat(*heartbeat_timeout)) {
        return;
    }
    {
        std::lock_guard lock(observation_mutex_);
        if (state_.identity.autopilot_type != telemetry::kArduPilotAutopilot ||
            state_.identity.vehicle_type != telemetry::kFixedWing) {
            return;
        }
    }

    const auto parameter_budget = remaining_timeout(deadline);
    if (!parameter_budget) {
        return;
    }
    const auto value = read_param("Q_ENABLE", (std::min)(*parameter_budget, kQuadplaneParameterTimeout));
    ObservationUpdate update(observation_mutex_, observation_changed_);
    if (value && (*value == 1.0F || *value == 2.0F)) {
        quadplane_enabled_ = true;
        state_.identity.aircraft_class = telemetry::AircraftClass::QuadPlane;
    } else if (value && *value == 0.0F) {
        quadplane_enabled_ = false;
        state_.identity.aircraft_class = telemetry::AircraftClass::Plane;
    } else {
        quadplane_enabled_.reset();
        state_.identity.aircraft_class = telemetry::AircraftClass::Unknown;
    }
}

void MavsdkMavlinkConnection::subscribe() {
    action_ = std::make_unique<mavsdk::Action>(system_);
    telemetry_ = std::make_unique<mavsdk::Telemetry>(system_);
    passthrough_ = std::make_unique<mavsdk::MavlinkPassthrough>(system_);
    geofence_ = std::make_unique<mavsdk::Geofence>(system_);
    param_ = std::make_unique<mavsdk::Param>(system_);
    offboard_ = std::make_unique<mavsdk::Offboard>(system_);
    position_handle_ = telemetry_->subscribe_position([this](const auto &value) { observe_position(value); });
    velocity_handle_ = telemetry_->subscribe_velocity_ned([this](const auto &value) { observe_velocity(value); });
    battery_handle_ = telemetry_->subscribe_battery([this](const auto &value) { observe_battery(value); });
    gps_handle_ = telemetry_->subscribe_gps_info([this](const auto &value) { observe_gps(value); });
    attitude_handle_ = telemetry_->subscribe_attitude_euler([this](const auto &value) { observe_attitude(value); });
    vtol_state_handle_ = telemetry_->subscribe_vtol_state([this](const auto value) { observe_vtol_state(value); });
    landed_state_handle_ =
        telemetry_->subscribe_landed_state([this](const auto value) { observe_landed_state(value); });
    heartbeat_handle_ = passthrough_->subscribe_message(MAVLINK_MSG_ID_HEARTBEAT,
                                                        [this](const auto &message) { observe_heartbeat(message); });
    connection_handle_ = system_->subscribe_is_connected([this](bool connected) {
        if (connected) {
            return;
        }
        ObservationUpdate update(observation_mutex_, observation_changed_);
        ++session_id_counter_;
        if (session_id_counter_ == 0) {
            ++session_id_counter_;
        }
        notify_vehicle_session_changed(session_id_counter_);
        state_.session_id = session_id_counter_;
        state_.connected = false;
        state_.heartbeat_fresh = false;
    });
}

void MavsdkMavlinkConnection::unsubscribe() {
    if (telemetry_) {
        if (position_handle_) {
            telemetry_->unsubscribe_position(*position_handle_);
        }
        if (velocity_handle_) {
            telemetry_->unsubscribe_velocity_ned(*velocity_handle_);
        }
        if (battery_handle_) {
            telemetry_->unsubscribe_battery(*battery_handle_);
        }
        if (gps_handle_) {
            telemetry_->unsubscribe_gps_info(*gps_handle_);
        }
        if (attitude_handle_) {
            telemetry_->unsubscribe_attitude_euler(*attitude_handle_);
        }
        if (vtol_state_handle_) {
            telemetry_->unsubscribe_vtol_state(*vtol_state_handle_);
        }
        if (landed_state_handle_) {
            telemetry_->unsubscribe_landed_state(*landed_state_handle_);
        }
    }
    if (passthrough_ && heartbeat_handle_) {
        passthrough_->unsubscribe_message(MAVLINK_MSG_ID_HEARTBEAT, *heartbeat_handle_);
    }
    if (system_ && connection_handle_) {
        system_->unsubscribe_is_connected(*connection_handle_);
    }
    position_handle_.reset();
    velocity_handle_.reset();
    battery_handle_.reset();
    gps_handle_.reset();
    attitude_handle_.reset();
    vtol_state_handle_.reset();
    landed_state_handle_.reset();
    heartbeat_handle_.reset();
    connection_handle_.reset();
}

void MavsdkMavlinkConnection::close() {
    std::unique_lock lifetime_lock(plugin_lifetime_mutex_);
    {
        ObservationUpdate update(observation_mutex_, observation_changed_);
        ++session_id_counter_;
        if (session_id_counter_ == 0) {
            ++session_id_counter_;
        }
        notify_vehicle_session_changed(session_id_counter_);
        state_.session_id = session_id_counter_;
        state_.connected = false;
        state_.heartbeat_fresh = false;
    }
    unsubscribe();
    action_.reset();
    telemetry_.reset();
    passthrough_.reset();
    geofence_.reset();
    param_.reset();
    offboard_.reset();
    system_.reset();
    if (handle_) {
        sdk_.remove_connection(*handle_);
        handle_.reset();
    }
    std::lock_guard lock(observation_mutex_);
    state_ = {};
    quadplane_enabled_.reset();
    heartbeat_.reset();
    last_heartbeat_ = {};
    velocity_active_ = false;
}

void MavsdkMavlinkConnection::observe_heartbeat(const mavlink_message_t &message) {
    mavlink_heartbeat_t decoded{};
    mavlink_msg_heartbeat_decode(&message, &decoded);
    ObservationUpdate update(observation_mutex_, observation_changed_);
    state_.connected = true;
    state_.heartbeat_fresh = true;
    state_.system_id = message.sysid;
    state_.component_id = message.compid;
    state_.custom_mode = decoded.custom_mode;
    state_.armed = (decoded.base_mode & kArmModeFlag) != 0;
    state_.identity = telemetry::identify_vehicle(decoded.autopilot, decoded.type);
    if (state_.identity.aircraft_class == telemetry::AircraftClass::Plane) {
        if (!quadplane_enabled_.has_value()) {
            state_.identity.aircraft_class = telemetry::AircraftClass::Unknown;
        } else if (*quadplane_enabled_) {
            state_.identity.aircraft_class = telemetry::AircraftClass::QuadPlane;
        }
    }
    heartbeat_ = Heartbeat{message.sysid, message.compid, decoded.custom_mode, decoded.type, decoded.autopilot,
                           decoded.base_mode};
    last_heartbeat_ = ObservationClock::now();
}

void MavsdkMavlinkConnection::observe_position(const mavsdk::Telemetry::Position &position) {
    ObservationUpdate update(observation_mutex_, observation_changed_);
    state_.position.latitude_deg = position.latitude_deg;
    state_.position.longitude_deg = position.longitude_deg;
    state_.position.altitude_m = position.absolute_altitude_m;
    state_.position.relative_altitude_m = position.relative_altitude_m;
    state_.position_valid = true;
    state_.position_updated_at = ObservationClock::now();
}

void MavsdkMavlinkConnection::observe_velocity(const mavsdk::Telemetry::VelocityNed &velocity) {
    ObservationUpdate update(observation_mutex_, observation_changed_);
    state_.velocity.north_mps = velocity.north_m_s;
    state_.velocity.east_mps = velocity.east_m_s;
    state_.velocity.down_mps = velocity.down_m_s;
    state_.velocity.groundspeed_mps = std::sqrt(velocity.north_m_s * velocity.north_m_s +
                                                velocity.east_m_s * velocity.east_m_s);
    // NED down is positive downwards, so climb rate is its negation.
    state_.velocity.climb_rate_mps = -velocity.down_m_s;
    state_.velocity_updated_at = ObservationClock::now();
}

void MavsdkMavlinkConnection::observe_battery(const mavsdk::Telemetry::Battery &battery) {
    ObservationUpdate update(observation_mutex_, observation_changed_);
    state_.battery.voltage_v = battery.voltage_v;
    state_.battery.remaining_percent = battery.remaining_percent;
    state_.battery_valid = true;
    state_.battery_updated_at = ObservationClock::now();
}

void MavsdkMavlinkConnection::observe_gps(const mavsdk::Telemetry::GpsInfo &gps) {
    ObservationUpdate update(observation_mutex_, observation_changed_);
    state_.gps.fix_type = static_cast<std::uint8_t>(gps.fix_type);
    state_.gps.satellites = static_cast<std::uint8_t>(gps.num_satellites);
    state_.gps_valid = gps.fix_type != mavsdk::Telemetry::FixType::NoGps;
    if (state_.gps_valid) {
        state_.gps_updated_at = ObservationClock::now();
    }
}

void MavsdkMavlinkConnection::observe_attitude(const mavsdk::Telemetry::EulerAngle &attitude) {
    ObservationUpdate update(observation_mutex_, observation_changed_);
    state_.attitude.roll_deg = attitude.roll_deg;
    state_.attitude.pitch_deg = attitude.pitch_deg;
    state_.attitude.yaw_deg = attitude.yaw_deg;
    state_.attitude_valid = true;
    state_.attitude_updated_at = ObservationClock::now();
}

telemetry::VehicleState MavsdkMavlinkConnection::state_locked() const {
    auto state = state_;
    if (last_heartbeat_ == ObservationClock::time_point{}) {
        state.heartbeat_fresh = false;
        state.connected = false;
        return state;
    }
    const auto elapsed = ObservationClock::now() - last_heartbeat_;
    state.heartbeat_fresh = elapsed <= kHeartbeatTimeout;
    state.connected = state.heartbeat_fresh;
    return state;
}

telemetry::VehicleState MavsdkMavlinkConnection::get_state() const {
    std::lock_guard lock(observation_mutex_);
    return state_locked();
}

std::optional<Heartbeat> MavsdkMavlinkConnection::wait_for_heartbeat(std::chrono::milliseconds timeout) {
    std::unique_lock lock(observation_mutex_);
    const auto deadline = ObservationClock::now() + timeout;
    while (ObservationClock::now() < deadline) {
        if (heartbeat_.has_value() && state_locked().connected) {
            return heartbeat_;
        }
        observation_changed_.wait_until(lock, (std::min)(deadline, ObservationClock::now() + kTelemetryWaitIncrement));
    }
    return std::nullopt;
}

std::optional<telemetry::VehicleState> MavsdkMavlinkConnection::wait_for_state(std::chrono::milliseconds timeout) {
    std::unique_lock lock(observation_mutex_);
    const auto deadline = ObservationClock::now() + timeout;
    while (ObservationClock::now() < deadline) {
        const auto state = state_locked();
        if (state.connected && has_telemetry(state)) {
            return state;
        }
        observation_changed_.wait_until(lock, (std::min)(deadline, ObservationClock::now() + kTelemetryWaitIncrement));
    }
    return std::nullopt;
}

mavsdk::MavlinkPassthrough::Result MavsdkMavlinkConnection::send_long(
    const Command &command, std::chrono::milliseconds timeout, const TransmissionAdmission &admission) {
    mavsdk::MavlinkPassthrough::CommandLong wire{};
    wire.target_sysid = target_system_;
    wire.target_compid = target_component_;
    wire.command = command.id;
    wire.param1 = command.parameters[0];
    wire.param2 = command.parameters[1];
    wire.param3 = command.parameters[2];
    wire.param4 = command.parameters[3];
    wire.param5 = command.parameters[4];
    wire.param6 = command.parameters[5];
    wire.param7 = command.parameters[6];
    if (admission && !admission([] {})) {
        return mavsdk::MavlinkPassthrough::Result::CommandAdmissionCancelled;
    }
    mavsdk::OperationOptions options{timeout};
    options.transmission_admission = admission;
    return passthrough_->send_command_long(wire, options);
}

std::optional<float> MavsdkMavlinkConnection::read_param(const std::string &param_id,
                                                         std::chrono::milliseconds timeout) {
    std::shared_lock lifetime_lock(plugin_lifetime_mutex_);
    if (!is_connected_unlocked() || !param_ || param_id.empty() || timeout <= std::chrono::milliseconds::zero()) {
        return std::nullopt;
    }

    // ArduPilot reports integer-valued parameters such as FENCE_ENABLE with
    // their MAVLink integer type. Try the integer API first, then accept a
    // REAL32 parameter. MAVSDK owns the retry schedule; NOMAD only carries the
    // unused portion of the caller's total budget into the second type probe.
    const auto deadline = ObservationClock::now() + timeout;
    auto remaining = remaining_timeout(deadline);
    if (!remaining) {
        return std::nullopt;
    }
    const auto [int_result, int_value] =
        param_->get_param_int(param_id, mavsdk::OperationOptions{*remaining});
    if (int_result == mavsdk::Param::Result::Success) {
        return static_cast<float>(int_value);
    }

    remaining = remaining_timeout(deadline);
    if (!remaining) {
        return std::nullopt;
    }
    const auto [result, value] =
        param_->get_param_float(param_id, mavsdk::OperationOptions{*remaining});
    if (result == mavsdk::Param::Result::Success) {
        return value;
    }
    return std::nullopt;
}

std::optional<CommandAck> MavsdkMavlinkConnection::send_command(const Command &command,
                                                                std::chrono::milliseconds timeout) {
    const auto admission = capture_transmission_admission();
    std::shared_lock lifetime_lock(plugin_lifetime_mutex_);
    if (!is_connected_unlocked() || !passthrough_ || timeout <= std::chrono::milliseconds::zero()) {
        return std::nullopt;
    }
    const auto result = send_long(command, timeout, admission);
    if (result == mavsdk::MavlinkPassthrough::Result::CommandAdmissionCancelled) {
        return CommandAck{command.id, 0, CommandAck::Status::AdmissionCancelled};
    }
    const auto code = mavsdk_command_result_code(result);
    if (!code.has_value()) {
        return std::nullopt;
    }
    return CommandAck{command.id, *code};
}

std::unique_ptr<MavlinkConnection> make_mavsdk_connection(const std::string &endpoint,
                                                          std::uint8_t expected_system_id,
                                                          std::chrono::milliseconds discovery_timeout) {
    return std::make_unique<MavsdkMavlinkConnection>(endpoint, expected_system_id, discovery_timeout);
}

} // namespace nomad::mavlink
