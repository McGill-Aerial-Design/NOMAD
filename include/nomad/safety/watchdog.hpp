// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <chrono>
#include <cstdint>
#include <string>

namespace nomad::safety {

struct WatchdogPolicy {
    std::chrono::milliseconds command_timeout{500};
    std::chrono::milliseconds vio_timeout{1000};
    std::chrono::milliseconds poll_interval{50};
    float min_vio_confidence{0.3F};
};

enum class WatchdogReason {
    none,
    command_timeout,
    heartbeat_stale,
    disarmed,
    wrong_mode,
    vio_stale,
    invalid_policy,
};

struct WatchdogInput {
    bool active{false};
    bool connected{false};
    bool heartbeat_fresh{false};
    bool armed{false};
    std::uint32_t custom_mode{};
    bool command_fresh{false};
    bool vio_healthy{false};
    bool vio_fresh{false};
    float vio_confidence{};
};

struct WatchdogDecision {
    bool stop{false};
    WatchdogReason reason{WatchdogReason::none};
    std::string message;
};

bool is_valid_watchdog_policy(const WatchdogPolicy& policy);
WatchdogDecision evaluate_watchdog(const WatchdogPolicy& policy, const WatchdogInput& input);
const char* watchdog_reason_name(WatchdogReason reason);

}  // namespace nomad::safety
