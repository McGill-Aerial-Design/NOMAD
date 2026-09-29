// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors

#pragma once

#include "nomad/mavlink/mavsdk_validation.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <string_view>

namespace nomad::mavlink {

enum class MavlinkObservationError {
    None,
    InvalidConfiguration,
    AddConnectionFailed,
    DiscoveryTimeout,
    WrongPeer,
};

struct MavlinkObservationOptions {
    std::string endpoint;
    std::uint8_t expected_system_id{};
    std::chrono::milliseconds discovery_timeout{5000};
};

struct MavlinkObservationSnapshot {
    mavsdk_phase_a::StatusValues values;
    std::size_t position_updates{};
    std::int64_t observation_ms{};
    std::int64_t age_ms{};
    std::optional<std::int64_t> gps_age_ms;
    std::optional<std::int64_t> battery_age_ms;
};

// Receives MAVLink telemetry over UDP and has no API for transmitting frames.
class MavlinkObservation {
  public:
    explicit MavlinkObservation(MavlinkObservationOptions options);
    ~MavlinkObservation();

    MavlinkObservation(const MavlinkObservation &) = delete;
    MavlinkObservation &operator=(const MavlinkObservation &) = delete;

    bool connect();
    void disconnect();
    bool is_connected() const;
    std::uint8_t system_id() const;
    MavlinkObservationError last_error() const;
    MavlinkObservationSnapshot get_status() const;
    std::optional<MavlinkObservationSnapshot> wait_for_status(std::chrono::milliseconds timeout) const;

  private:
    struct Implementation;
    std::unique_ptr<Implementation> implementation_;
};

std::string_view mavlink_observation_error_message(MavlinkObservationError error);

} // namespace nomad::mavlink
