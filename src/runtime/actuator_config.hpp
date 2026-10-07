// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "nomad/runtime/actuator.hpp"
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

namespace nomad::runtime::detail {

using Json = nlohmann::json;
bool actuator_is_relay(const ActuatorDefinition &definition);
std::string actuator_behavior_name(ActuatorBehavior behavior);
bool validate_actuator_definitions(const std::vector<ActuatorDefinition> &definitions, std::string &error);
bool parse_actuator_definitions(const Json &data, std::vector<ActuatorDefinition> &definitions, std::string &error);
Json serialize_actuator_definitions(const std::vector<ActuatorDefinition> &definitions);

} // namespace nomad::runtime::detail
