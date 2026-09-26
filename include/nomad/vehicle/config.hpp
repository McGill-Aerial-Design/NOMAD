// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "nomad/safety/geofence.hpp"
#include "nomad/safety/velocity.hpp"
#include "nomad/safety/watchdog.hpp"

#include <chrono>

namespace nomad::vehicle {

struct VehicleTimeouts {
    std::chrono::milliseconds position_freshness{2000};
    std::chrono::milliseconds vtol_takeoff_state{30000};
    std::chrono::milliseconds transition_state{90000};
    std::chrono::milliseconds fixed_wing_route{180000};
    std::chrono::milliseconds fixed_wing_recovery{180000};
    std::chrono::milliseconds transition_ready_dwell{2000};
    std::chrono::milliseconds quadplane_landing{90000};
};

struct VehicleConfig {
    safety::WatchdogPolicy watchdog{};
    safety::GlobalFencePolicy fence{};
    safety::VelocityLimits velocity{};
    VehicleTimeouts timeouts{};
};

} // namespace nomad::vehicle
