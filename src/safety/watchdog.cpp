// SPDX-License-Identifier: Apache-2.0
#include "nomad/safety/watchdog.hpp"

#include "nomad/safety/velocity.hpp"

#include <cmath>

namespace nomad::safety {
namespace {

WatchdogDecision stop_for(WatchdogReason reason, const char* message) {
    return {true, reason, message};
}

bool has_valid_policy(const WatchdogPolicy& policy) {
    return policy.command_timeout.count() > 0 && policy.vio_timeout.count() > 0 &&
           policy.poll_interval.count() > 0 && std::isfinite(policy.min_vio_confidence) &&
           policy.min_vio_confidence >= 0.0F && policy.min_vio_confidence <= 1.0F;
}

bool has_usable_vio(const WatchdogPolicy& policy, const WatchdogInput& input) {
    return input.vio_healthy && input.vio_fresh && std::isfinite(input.vio_confidence) &&
           input.vio_confidence >= policy.min_vio_confidence;
}

}  // namespace

bool is_valid_watchdog_policy(const WatchdogPolicy& policy) {
    return has_valid_policy(policy);
}

WatchdogDecision evaluate_watchdog(const WatchdogPolicy& policy, const WatchdogInput& input) {
    if (!is_valid_watchdog_policy(policy)) {
        return {input.active, WatchdogReason::invalid_policy, "velocity watchdog policy is invalid"};
    }
    if (!input.active) {
        return {false, WatchdogReason::none, "velocity control is inactive"};
    }
    if (!input.command_fresh) {
        return stop_for(WatchdogReason::command_timeout, "velocity command timeout");
    }
    if (!input.connected || !input.heartbeat_fresh) {
        return stop_for(WatchdogReason::heartbeat_stale, "flight-controller heartbeat is stale");
    }
    if (!input.armed) {
        return stop_for(WatchdogReason::disarmed, "vehicle is disarmed");
    }
    if (input.custom_mode != kGuidedMode) {
        return stop_for(WatchdogReason::wrong_mode, "vehicle left GUIDED mode");
    }
    if (!has_usable_vio(policy, input)) {
        return stop_for(WatchdogReason::vio_stale, "VIO is unhealthy, stale, or low confidence");
    }
    return {false, WatchdogReason::none, "velocity control is healthy"};
}

const char* watchdog_reason_name(WatchdogReason reason) {
    switch (reason) {
    case WatchdogReason::none:
        return "none";
    case WatchdogReason::command_timeout:
        return "command_timeout";
    case WatchdogReason::heartbeat_stale:
        return "heartbeat_stale";
    case WatchdogReason::disarmed:
        return "disarmed";
    case WatchdogReason::wrong_mode:
        return "wrong_mode";
    case WatchdogReason::vio_stale:
        return "vio_stale";
    case WatchdogReason::invalid_policy:
        return "invalid_policy";
    }
    return "unknown";
}

}  // namespace nomad::safety
