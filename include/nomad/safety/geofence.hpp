// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cstddef>
#include <limits>
#include <optional>
#include <string>
#include <vector>

namespace nomad::safety {

struct Point2d {
    double x{};
    double y{};
};

struct FencePolicy {
    std::optional<std::vector<Point2d>> boundary;
    double margin_m{};
};

struct FenceDecision {
    bool allowed{false};
    std::string reason;
    std::string message;
};

struct GlobalPoint {
    double latitude_deg{};
    double longitude_deg{};
};

struct GlobalFencePolicy {
    std::optional<std::vector<GlobalPoint>> boundary;
    double margin_m{};
};

bool point_in_polygon(const Point2d& point, const std::vector<Point2d>& polygon);
double distance_to_boundary(const Point2d& point, const std::vector<Point2d>& polygon);
bool is_contained(const Point2d& point, const std::vector<Point2d>& polygon, double margin_m);
FenceDecision evaluate_position(const FencePolicy& policy, const Point2d& point);
FenceDecision evaluate_global_position(const GlobalFencePolicy& policy, const GlobalPoint& point);

}  // namespace nomad::safety
