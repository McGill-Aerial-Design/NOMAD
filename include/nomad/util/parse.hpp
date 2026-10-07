// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <optional>
#include <string_view>

namespace nomad::util {

// Parses a complete, finite decimal number. The whole input must be consumed,
// so trailing garbage, an empty string and non-finite values are all rejected.
// Policy (non-negative, bounded, defaulted) stays with the caller: these
// helpers only answer "is this a number?".
std::optional<double> parse_double(std::string_view text);
std::optional<float> parse_float(std::string_view text);

// As parse_float, but negative values are also rejected.
std::optional<float> parse_non_negative_float(std::string_view text);

} // namespace nomad::util
