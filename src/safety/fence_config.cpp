// SPDX-License-Identifier: Apache-2.0
// Fence configuration loading: parse the NOMAD-side keep-in fence from the
// environment (NOMAD_FENCE_POLYGON / NOMAD_FENCE_MARGIN_M) into the
// GlobalFencePolicy the Vehicle applies to every position target. A malformed
// configured fence must fail closed: the policy then rejects every target.
//
// Polygon format: semicolon-separated "lat,lon" vertices (WGS84 degrees,
// >= 3). An unset NOMAD_FENCE_POLYGON returns a policy without a boundary —
// ArduPilot's own FC fence remains the enforcement in that case.
#include "nomad/safety/fence_config.hpp"

#include <cmath>
#include <cstdlib>
#include <string>
#include <vector>

namespace nomad::safety {
namespace {

constexpr double kMinimumMarginMeters = 0.0;

bool parse_finite_double(const std::string &text, double &value) {
    char *end = nullptr;
    const double parsed = std::strtod(text.c_str(), &end);
    if (end == text.c_str() || *end != '\0' || !std::isfinite(parsed)) {
        return false;
    }
    value = parsed;
    return true;
}

bool parse_vertex(const std::string &pair, GlobalPoint &vertex) {
    const auto comma = pair.find(',');
    if (comma == std::string::npos) {
        return false;
    }
    GlobalPoint parsed{};
    if (!parse_finite_double(pair.substr(0, comma), parsed.latitude_deg) ||
        !parse_finite_double(pair.substr(comma + 1), parsed.longitude_deg)) {
        return false;
    }
    vertex = parsed;
    return true;
}

}  // namespace

GlobalFencePolicy load_fence_policy(const char *polygon_env, const char *margin_env) {
    const char *polygon_text = polygon_env == nullptr ? "" : polygon_env;
    const char *margin_text = margin_env == nullptr ? "2.0" : margin_env;

    GlobalFencePolicy policy{std::nullopt, 2.0};
    if (!parse_finite_double(margin_text, policy.margin_m) || policy.margin_m < kMinimumMarginMeters) {
        // A broken margin must reject every target, not fly unfenced.
        return {std::vector<GlobalPoint>{}, 0.0};
    }

    const std::string raw(polygon_text);
    if (raw.empty()) {
        return policy;
    }

    std::vector<GlobalPoint> boundary;
    std::size_t start = 0;
    while (start <= raw.size()) {
        const auto separator = raw.find(';', start);
        const auto pair = raw.substr(start, separator == std::string::npos ? std::string::npos : separator - start);
        GlobalPoint vertex{};
        if (!parse_vertex(pair, vertex)) {
            return {std::vector<GlobalPoint>{}, policy.margin_m};
        }
        boundary.push_back(vertex);
        if (separator == std::string::npos) {
            break;
        }
        start = separator + 1;
    }
    if (boundary.size() < 3) {
        return {std::vector<GlobalPoint>{}, policy.margin_m};
    }
    policy.boundary = boundary;
    return policy;
}

}  // namespace nomad::safety
