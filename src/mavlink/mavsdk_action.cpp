// SPDX-License-Identifier: Apache-2.0
#include "mavsdk_mavlink_connection.hpp"

#include <chrono>
#include <cmath>
#include <shared_mutex>

namespace nomad::mavlink {

bool MavsdkMavlinkConnection::goto_location_relative(double latitude_deg, double longitude_deg,
                                                      float relative_altitude_m, std::chrono::milliseconds timeout) {
    if (!admit_send()) {
        return false;
    }
    std::shared_lock lifetime_lock(plugin_lifetime_mutex_);
    if (!is_connected_unlocked() || !action_ || timeout <= std::chrono::milliseconds::zero()) {
        return false;
    }
    return action_->goto_location_relative(latitude_deg, longitude_deg, relative_altitude_m, NAN,
                                           mavsdk::OperationOptions{timeout}) == mavsdk::Action::Result::Success;
}

} // namespace nomad::mavlink
