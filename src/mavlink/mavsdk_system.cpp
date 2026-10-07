// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors

#include "mavsdk_system.hpp"

namespace nomad::mavlink::mavsdk_system {

std::vector<std::shared_ptr<mavsdk::System>> connected_autopilots(const mavsdk::Mavsdk &sdk) {
    std::vector<std::shared_ptr<mavsdk::System>> systems;
    for (const auto &system : sdk.systems()) {
        if (system->is_connected() && system->has_autopilot()) {
            systems.push_back(system);
        }
    }
    return systems;
}

std::shared_ptr<mavsdk::System> select_expected_autopilot(const mavsdk::Mavsdk &sdk, std::uint8_t expected_system_id,
                                                          mavsdk_phase_a::SystemSelection &selection) {
    const auto systems = connected_autopilots(sdk);
    std::vector<std::uint32_t> ids;
    ids.reserve(systems.size());
    for (const auto &candidate : systems) {
        ids.push_back(candidate->get_system_id());
    }
    selection = mavsdk_phase_a::classify_system_ids(ids, expected_system_id);
    if (selection != mavsdk_phase_a::SystemSelection::Selected) {
        return nullptr;
    }
    return systems.front();
}

} // namespace nomad::mavlink::mavsdk_system
