// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors

#include "nomad/mavlink/mavsdk_validation.hpp"

#include <charconv>
#include <cmath>
#include <cctype>

namespace nomad::mavsdk_phase_a {
namespace {

constexpr std::int64_t kMinimumObservationMs = 1000;
constexpr std::int64_t kMaximumTelemetryAgeMs = 1500;
constexpr std::size_t kMinimumPositionUpdates = 3;

std::optional<std::uint16_t> parse_port(std::string_view value) {
    if (value.empty()) {
        return {};
    }
    std::uint16_t port{};
    const auto [end, error] = std::from_chars(value.data(), value.data() + value.size(), port);
    if (error != std::errc{} || end != value.data() + value.size() || port == 0) {
        return {};
    }
    return port;
}

bool split_endpoint(std::string_view endpoint, std::string_view &scheme, std::string_view &rest) {
    constexpr std::string_view schemes[] = {"udpin", "udpout"};
    for (const auto candidate : schemes) {
        if (!endpoint.starts_with(candidate)) {
            continue;
        }
        const auto suffix = endpoint.substr(candidate.size());
        if (suffix.starts_with("://")) {
            scheme = candidate;
            rest = endpoint.substr(candidate.size() + 3);
            return true;
        }
        if (suffix.starts_with(":")) {
            scheme = candidate;
            rest = endpoint.substr(candidate.size() + 1);
            return true;
        }
    }
    return false;
}

bool split_host_port(std::string_view rest, std::string_view &host, std::string_view &port) {
    const auto separator = rest.rfind(':');
    if (separator == std::string_view::npos) {
        return false;
    }
    host = rest.substr(0, separator);
    port = rest.substr(separator + 1);
    if (host.empty() || port.empty()) {
        return false;
    }
    for (const auto character : host) {
        if (std::isspace(static_cast<unsigned char>(character)) != 0) {
            return false;
        }
    }
    if (host.front() == '[') {
        return host.back() == ']' && host.size() >= 3;
    }
    return host.find(':') == std::string_view::npos;
}

} // namespace

std::optional<std::string> canonicalize_udp_endpoint(std::string_view endpoint) {
    std::string_view scheme;
    std::string_view rest;
    if (!split_endpoint(endpoint, scheme, rest)) {
        return {};
    }
    std::string_view host;
    std::string_view port_text;
    if (!split_host_port(rest, host, port_text) || !parse_port(port_text)) {
        return {};
    }
    if (scheme == "udpout" && host == "0.0.0.0") {
        return {};
    }
    return std::string(scheme) + "://" + std::string(host) + ":" + std::string(port_text);
}

std::optional<std::string> canonicalize_udp_input_endpoint(std::string_view endpoint) {
    auto canonical = canonicalize_udp_endpoint(endpoint);
    if (!canonical || !canonical->starts_with("udpin://")) {
        return {};
    }
    return canonical;
}

std::optional<std::uint8_t> parse_system_id(std::string_view value) {
    if (value.empty()) {
        return {};
    }
    unsigned int system_id{};
    const auto [end, error] = std::from_chars(value.data(), value.data() + value.size(), system_id);
    if (error != std::errc{} || end != value.data() + value.size() || system_id == 0 || system_id > 255) {
        return {};
    }
    return static_cast<std::uint8_t>(system_id);
}

SystemSelection classify_system_ids(const std::vector<std::uint32_t> &system_ids, std::uint8_t expected_id) {
    if (system_ids.empty()) {
        return SystemSelection::NoAutopilot;
    }
    if (system_ids.size() > 1) {
        return SystemSelection::Ambiguous;
    }
    return system_ids.front() == expected_id ? SystemSelection::Selected : SystemSelection::WrongPeer;
}

bool has_valid_status(const StatusValues &values) {
    constexpr double kMinRelativeAltitudeM = -1000.0;
    constexpr double kMaxRelativeAltitudeM = 100000.0;
    return has_valid_position(values) && has_valid_gps(values) && has_valid_battery(values) &&
           std::isfinite(values.relative_altitude_m) && values.relative_altitude_m >= kMinRelativeAltitudeM &&
           values.relative_altitude_m <= kMaxRelativeAltitudeM && values.flight_mode > 0;
}

bool has_valid_position(const StatusValues &values) {
    constexpr double kMinAltitudeM = -1000.0;
    constexpr double kMaxAltitudeM = 100000.0;
    return std::isfinite(values.latitude_deg) && values.latitude_deg >= -90.0 && values.latitude_deg <= 90.0 &&
           std::isfinite(values.longitude_deg) && values.longitude_deg >= -180.0 && values.longitude_deg <= 180.0 &&
           std::isfinite(values.absolute_altitude_m) && values.absolute_altitude_m >= kMinAltitudeM &&
           values.absolute_altitude_m <= kMaxAltitudeM;
}

bool has_valid_gps(const StatusValues &values) {
    return values.gps_fix_type >= 3 && values.satellites > 0 && values.satellites <= 255;
}

bool has_valid_battery(const StatusValues &values) {
    constexpr double kMaxBatteryVoltageV = 1000.0;
    return std::isfinite(values.battery_voltage_v) && values.battery_voltage_v > 0.0 &&
           values.battery_voltage_v <= kMaxBatteryVoltageV && std::isfinite(values.battery_remaining_percent) &&
           values.battery_remaining_percent >= 0.0 && values.battery_remaining_percent <= 100.0;
}

bool has_fresh_position_stream(std::size_t update_count, std::int64_t observation_ms, std::int64_t age_ms) {
    return update_count >= kMinimumPositionUpdates && observation_ms >= kMinimumObservationMs && age_ms >= 0 &&
           age_ms <= kMaximumTelemetryAgeMs;
}

bool has_fresh_telemetry(std::int64_t age_ms, std::int64_t maximum_age_ms) {
    return maximum_age_ms >= 0 && age_ms >= 0 && age_ms <= maximum_age_ms;
}

} // namespace nomad::mavsdk_phase_a
