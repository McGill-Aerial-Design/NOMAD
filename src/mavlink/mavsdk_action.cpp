// SPDX-License-Identifier: Apache-2.0
#include "mavsdk_mavlink_connection.hpp"

#include <chrono>
#include <cmath>
#include <shared_mutex>

namespace nomad::mavlink {

bool MavsdkMavlinkConnection::goto_location_relative(double latitude_deg, double longitude_deg,
                                                      float relative_altitude_m, std::chrono::milliseconds timeout) {
    const auto admission = capture_transmission_admission();
    if (admission && !admission([] {})) {
        return false;
    }
    std::shared_lock lifetime_lock(plugin_lifetime_mutex_);
    if (!is_connected_unlocked() || !resources_->action || timeout <= std::chrono::milliseconds::zero()) {
        return false;
    }
    mavsdk::OperationOptions options{timeout};
    options.transmission_admission = admission;
    return resources_->action->goto_location_relative(latitude_deg, longitude_deg, relative_altitude_m, NAN, options) ==
           mavsdk::Action::Result::Success;
}

} // namespace nomad::mavlink
