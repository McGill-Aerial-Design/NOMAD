// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <chrono>

namespace nomad::vehicle::detail {

inline bool timestamp_is_fresh(std::chrono::steady_clock::time_point timestamp,
                               std::chrono::milliseconds timeout,
                               std::chrono::steady_clock::time_point now) {
    return timestamp != std::chrono::steady_clock::time_point{} && timestamp <= now && now - timestamp <= timeout;
}

} // namespace nomad::vehicle::detail
