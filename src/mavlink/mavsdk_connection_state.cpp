// SPDX-License-Identifier: Apache-2.0
// Connection-state access kept separate from command and observation paths.

#include "mavsdk_mavlink_connection.hpp"

namespace nomad::mavlink {

bool MavsdkMavlinkConnection::is_connected() const {
    std::shared_lock lifetime_lock(plugin_lifetime_mutex_);
    return is_connected_unlocked();
}

bool MavsdkMavlinkConnection::is_connected_unlocked() const {
    return resources_ != nullptr && resources_->system->is_connected();
}

} // namespace nomad::mavlink
