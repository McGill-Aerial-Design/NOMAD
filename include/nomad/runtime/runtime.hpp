// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "nomad/mavlink/connection.hpp"
#include "nomad/runtime/actuator.hpp"
#include "nomad/safety/geofence.hpp"
#include "nomad/safety/velocity.hpp"

#include <chrono>
#include <cstdint>
#include <memory>
#include <map>
#include <functional>
#include <string>
#include <vector>

namespace nomad::runtime {

struct RuntimeConfig {
    std::uint16_t ipc_port{14611};
    std::chrono::milliseconds discovery_timeout{std::chrono::seconds(6)};
    std::chrono::milliseconds reconnect_delay{std::chrono::seconds(1)};
    std::string version{"0.1.0"};
    safety::GlobalFencePolicy fence_policy{};
    safety::VelocityLimits velocity_limits{};
    bool actuation_enabled{false};
    std::map<std::string, std::string> client_credentials;
    std::string audit_directory;
    std::string actuator_config_file;
    std::vector<ActuatorDefinition> actuators;
    // A test may reject directory synchronization, never bypass its native barrier.
    std::function<bool()> actuator_directory_sync_guard;
    // Optional test fault injector: may reject an append, never bypass its native durability barrier.
    std::function<bool(const std::string &)> audit_write_guard;
};

// Owns one MAVLink connection and one Vehicle for the life of the process.
// Clients communicate through the versioned loopback IPC endpoint.
class Runtime {
  public:
    Runtime(std::unique_ptr<mavlink::MavlinkConnection> connection, RuntimeConfig config = {});
    ~Runtime();

    Runtime(const Runtime &) = delete;
    Runtime &operator=(const Runtime &) = delete;

    bool start(std::string &error);
    // Close final-send admission immediately; stop() then drains owned workers.
    void request_stop();
    bool stop();
    bool ready() const;

  private:
    struct Implementation;
    std::unique_ptr<Implementation> implementation_;
};

} // namespace nomad::runtime
