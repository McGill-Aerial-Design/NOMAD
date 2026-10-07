// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "nomad/runtime/actuator.hpp"

#include <functional>
#include <string>
#include <vector>

namespace nomad::runtime::detail {

bool load_actuator_configuration(const std::string &path, std::vector<ActuatorDefinition> &definitions,
                                 std::string &error);
bool save_actuator_configuration(const std::string &path, const std::vector<ActuatorDefinition> &definitions,
                                 std::string &error, bool &replacement_attempted,
                                 const std::function<bool()> &directory_sync_guard = {});

} // namespace nomad::runtime::detail
