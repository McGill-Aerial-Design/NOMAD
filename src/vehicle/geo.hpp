// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <algorithm>
#include <cmath>
#include <numbers>

namespace nomad::vehicle::detail {

inline double radians(double degrees) {
    return degrees * std::numbers::pi / 180.0;
}

inline double distance_m(double latitude_a, double longitude_a, double latitude_b, double longitude_b) {
    constexpr double kEarthRadiusMeters = 6371000.0;
    const double latitude_delta = radians(latitude_b - latitude_a);
    const double longitude_delta = radians(longitude_b - longitude_a);
    const double a = std::sin(latitude_delta / 2.0) * std::sin(latitude_delta / 2.0) +
                     std::cos(radians(latitude_a)) * std::cos(radians(latitude_b)) *
                         std::sin(longitude_delta / 2.0) * std::sin(longitude_delta / 2.0);
    return 2.0 * kEarthRadiusMeters * std::asin(std::sqrt(std::clamp(a, 0.0, 1.0)));
}

} // namespace nomad::vehicle::detail
