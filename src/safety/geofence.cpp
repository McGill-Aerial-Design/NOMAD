// SPDX-License-Identifier: Apache-2.0
#include "nomad/safety/geofence.hpp"

#include <algorithm>
#include <cmath>

namespace nomad::safety {
namespace {

constexpr double kMetersPerDegreeLatitude = 110540.0;
constexpr double kMetersPerDegreeLongitude = 111320.0;

bool is_finite(const Point2d& point) {
    return std::isfinite(point.x) && std::isfinite(point.y);
}

double distance_to_segment(const Point2d& point, const Point2d& start, const Point2d& end) {
    const double dx = end.x - start.x;
    const double dy = end.y - start.y;
    const double length_squared = dx * dx + dy * dy;
    if (length_squared == 0.0) {
        return std::hypot(point.x - start.x, point.y - start.y);
    }
    const double projection = std::clamp(
        ((point.x - start.x) * dx + (point.y - start.y) * dy) / length_squared,
        0.0,
        1.0);
    const Point2d closest{start.x + projection * dx, start.y + projection * dy};
    return std::hypot(point.x - closest.x, point.y - closest.y);
}

bool is_finite(const GlobalPoint& point) {
    return std::isfinite(point.latitude_deg) && std::isfinite(point.longitude_deg) &&
           point.latitude_deg >= -90.0 && point.latitude_deg <= 90.0 &&
           point.longitude_deg >= -180.0 && point.longitude_deg <= 180.0;
}

Point2d to_local_point(const GlobalPoint& reference, const GlobalPoint& point) {
    const double longitude_scale = kMetersPerDegreeLongitude *
                                   std::cos(reference.latitude_deg * 0.017453292519943295);
    return {
        (point.latitude_deg - reference.latitude_deg) * kMetersPerDegreeLatitude,
        (point.longitude_deg - reference.longitude_deg) * longitude_scale,
    };
}

}  // namespace

bool point_in_polygon(const Point2d& point, const std::vector<Point2d>& polygon) {
    if (polygon.size() < 3) {
        return false;
    }
    bool inside = false;
    std::size_t previous = polygon.size() - 1;
    for (std::size_t current = 0; current < polygon.size(); ++current) {
        const auto& first = polygon[current];
        const auto& second = polygon[previous];
        if ((first.y > point.y) != (second.y > point.y)) {
            const double crossing = (second.x - first.x) * (point.y - first.y) /
                                        (second.y - first.y) + first.x;
            if (point.x < crossing) {
                inside = !inside;
            }
        }
        previous = current;
    }
    return inside;
}

double distance_to_boundary(const Point2d& point, const std::vector<Point2d>& polygon) {
    if (polygon.size() < 2) {
        return std::numeric_limits<double>::infinity();
    }
    double closest = std::numeric_limits<double>::infinity();
    std::size_t previous = polygon.size() - 1;
    for (std::size_t current = 0; current < polygon.size(); ++current) {
        closest = std::min(closest, distance_to_segment(point, polygon[previous], polygon[current]));
        previous = current;
    }
    return closest;
}

bool is_contained(const Point2d& point, const std::vector<Point2d>& polygon, double margin_m) {
    if (!point_in_polygon(point, polygon)) {
        return false;
    }
    if (margin_m <= 0.0) {
        return true;
    }
    return distance_to_boundary(point, polygon) >= margin_m;
}

FenceDecision evaluate_position(const FencePolicy& policy, const Point2d& point) {
    if (!is_finite(point)) {
        return {false, "nonfinite", "position target contains a non-finite value"};
    }
    if (!policy.boundary.has_value()) {
        return {true, "none", "position target accepted without a NOMAD fence"};
    }
    if (!std::isfinite(policy.margin_m) || policy.margin_m < 0.0) {
        return {false, "fence", "geofence margin is invalid"};
    }
    if (!is_contained(point, *policy.boundary, policy.margin_m)) {
        return {false, "fence", "position target is outside the geofence"};
    }
    return {true, "none", "position target accepted"};
}

FenceDecision evaluate_global_position(const GlobalFencePolicy& policy, const GlobalPoint& point) {
    if (!is_finite(point)) {
        return {false, "nonfinite", "global position target is invalid"};
    }
    if (!policy.boundary.has_value()) {
        return {true, "none", "global position target accepted without a NOMAD fence"};
    }
    if (policy.boundary->empty()) {
        return {false, "fence", "global geofence boundary is invalid"};
    }
    for (const auto& vertex : *policy.boundary) {
        if (!is_finite(vertex)) {
            return {false, "fence", "global geofence boundary contains an invalid vertex"};
        }
    }
    const auto reference = policy.boundary->front();
    std::vector<Point2d> local_boundary;
    local_boundary.reserve(policy.boundary->size());
    for (const auto& vertex : *policy.boundary) {
        local_boundary.push_back(to_local_point(reference, vertex));
    }
    return evaluate_position(
        FencePolicy{local_boundary, policy.margin_m},
        to_local_point(reference, point));
}

}  // namespace nomad::safety
