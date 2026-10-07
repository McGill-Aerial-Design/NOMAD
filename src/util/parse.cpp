// SPDX-License-Identifier: Apache-2.0
#include "nomad/util/parse.hpp"

#include <cmath>
#include <cstdlib>
#include <string>

namespace nomad::util {
namespace {

// strtod/strtof need a NUL-terminated buffer, so the caller's view is copied.
// These are configuration and command-line values, not hot-path data.
bool consumed_whole_input(const std::string &buffer, const char *end) {
    return end != buffer.c_str() && *end == '\0';
}

} // namespace

std::optional<double> parse_double(std::string_view text) {
    const std::string buffer(text);
    char *end = nullptr;
    const double parsed = std::strtod(buffer.c_str(), &end);
    if (!consumed_whole_input(buffer, end) || !std::isfinite(parsed)) {
        return std::nullopt;
    }
    return parsed;
}

std::optional<float> parse_float(std::string_view text) {
    const std::string buffer(text);
    char *end = nullptr;
    const float parsed = std::strtof(buffer.c_str(), &end);
    if (!consumed_whole_input(buffer, end) || !std::isfinite(parsed)) {
        return std::nullopt;
    }
    return parsed;
}

std::optional<float> parse_non_negative_float(std::string_view text) {
    const auto parsed = parse_float(text);
    if (!parsed.has_value() || *parsed < 0.0F) {
        return std::nullopt;
    }
    return parsed;
}

} // namespace nomad::util
