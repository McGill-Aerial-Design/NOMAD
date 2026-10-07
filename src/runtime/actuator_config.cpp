// SPDX-License-Identifier: Apache-2.0
#include "actuator_config.hpp"

#include <algorithm>
#include <array>
#include <set>
#include <limits>

namespace nomad::runtime::detail {
namespace {

bool readable(const std::string &value, std::size_t maximum) {
    return !value.empty() && value.size() <= maximum &&
           std::any_of(value.begin(), value.end(), [](unsigned char ch) { return ch > 32; }) &&
           std::none_of(value.begin(), value.end(), [](unsigned char ch) { return ch < 32 || ch == 127; });
}

bool valid_definition(const ActuatorDefinition &p) {
    const bool relay = actuator_is_relay(p);
    const auto behavior = static_cast<int>(p.behavior);
    const bool valid_id = readable(p.id, 64) && std::all_of(p.id.begin(), p.id.end(), [](unsigned char ch) {
        return (ch >= 'a' && ch <= 'z') || (ch >= 'A' && ch <= 'Z') ||
               (ch >= '0' && ch <= '9') || ch == '_' || ch == '-' || ch == '.';
    });
    if (!valid_id || !readable(p.name, 80) ||
        behavior < 0 || behavior > 4 || p.channel < (relay ? 0 : 1) || p.channel > (relay ? 15 : 16)) {
        return false;
    }
    if (p.pwm_min < 500 || p.pwm_max > 2500 || p.pwm_min >= p.pwm_max ||
        p.pwm_neutral < p.pwm_min || p.pwm_neutral > p.pwm_max ||
        p.safe_pwm < p.pwm_min || p.safe_pwm > p.pwm_max ||
        p.pulse_ms < 50 || p.pulse_ms > 1500 || p.hold_ms < 50 || p.hold_ms > 1500) {
        return false;
    }
    if (p.confirmation_count < 0 || p.confirmation_count > 3 || (p.hazardous && p.confirmation_count < 2) ||
        p.confirmation_window_ms < 500 || p.confirmation_window_ms > 5000 || (p.hazardous && !p.require_neutral)) {
        return false;
    }
    if ((p.behavior == ActuatorBehavior::ServoToggle && p.safe_pwm != (p.reversed ? p.pwm_max : p.pwm_min)) ||
        (p.behavior == ActuatorBehavior::ServoBidirectional && p.safe_pwm != p.pwm_neutral)) {
        return false;
    }
    return readable(p.primary_label, 80) && readable(p.secondary_label, 80) &&
           readable(p.negative_label, 80) && readable(p.neutral_label, 80) && readable(p.positive_label, 80);
}

Json serialize_definition(const ActuatorDefinition &p) {
    return {{"id", p.id}, {"name", p.name}, {"behavior", actuator_behavior_name(p.behavior)},
            {"channel", p.channel}, {"pwm_min", p.pwm_min}, {"pwm_max", p.pwm_max},
            {"pwm_neutral", p.pwm_neutral}, {"safe_pwm", p.safe_pwm}, {"reversed", p.reversed},
            {"pulse_ms", p.pulse_ms}, {"hold_ms", p.hold_ms}, {"primary_label", p.primary_label},
            {"secondary_label", p.secondary_label}, {"negative_label", p.negative_label},
            {"neutral_label", p.neutral_label}, {"positive_label", p.positive_label},
            {"hazardous", p.hazardous}, {"confirmation_count", p.confirmation_count},
            {"confirmation_window_ms", p.confirmation_window_ms}, {"require_neutral", p.require_neutral}};
}

bool validate_fields(const Json &item, const ActuatorDefinition &p) {
    const auto expected = serialize_definition(p);
    if (!item.is_object() || item.size() != expected.size()) {
        return false;
    }
    for (const auto &[key, value] : expected.items()) {
        if (!item.contains(key) || item[key].type() != value.type()) {
            if (!value.is_number_integer() || !item.contains(key) || !item[key].is_number_integer()) {
                return false;
            }
        }
        if (value.is_number_integer()) {
            if (item[key].is_number_unsigned()) {
                if (item[key].get<std::uint64_t>() > static_cast<std::uint64_t>(std::numeric_limits<int>::max())) {
                    return false;
                }
            } else {
                const auto number = item[key].get<std::int64_t>();
                if (number < std::numeric_limits<int>::min() || number > std::numeric_limits<int>::max()) {
                    return false;
                }
            }
        }
    }
    return true;
}

void read_output_fields(const Json &item, ActuatorDefinition &p) {
    p.channel = item.at("channel").get<int>();
    p.pwm_min = item.at("pwm_min").get<int>();
    p.pwm_max = item.at("pwm_max").get<int>();
    p.pwm_neutral = item.at("pwm_neutral").get<int>();
    p.safe_pwm = item.at("safe_pwm").get<int>();
    p.reversed = item.at("reversed").get<bool>();
    p.pulse_ms = item.at("pulse_ms").get<int>();
    p.hold_ms = item.at("hold_ms").get<int>();
}

void read_safety_labels(const Json &item, ActuatorDefinition &p) {
    p.primary_label = item.at("primary_label").get<std::string>();
    p.secondary_label = item.at("secondary_label").get<std::string>();
    p.negative_label = item.at("negative_label").get<std::string>();
    p.neutral_label = item.at("neutral_label").get<std::string>();
    p.positive_label = item.at("positive_label").get<std::string>();
    p.hazardous = item.at("hazardous").get<bool>();
    p.confirmation_count = item.at("confirmation_count").get<int>();
    p.confirmation_window_ms = item.at("confirmation_window_ms").get<int>();
    p.require_neutral = item.at("require_neutral").get<bool>();
}

bool read_fields(const Json &item, ActuatorDefinition &p) {
    if (!validate_fields(item, p)) {
        return false;
    }
    p.id = item.at("id").get<std::string>();
    p.name = item.at("name").get<std::string>();
    const auto name = item.at("behavior").get<std::string>();
    bool found = false;
    for (int index = 0; index <= 4; ++index) {
        const auto behavior = static_cast<ActuatorBehavior>(index);
        if (actuator_behavior_name(behavior) == name) {
            p.behavior = behavior;
            found = true;
        }
    }
    if (!found) {
        return false;
    }
    read_output_fields(item, p);
    read_safety_labels(item, p);
    return true;
}

} // namespace

bool actuator_is_relay(const ActuatorDefinition &p) {
    return p.behavior == ActuatorBehavior::RelayToggle || p.behavior == ActuatorBehavior::RelayPulse;
}

std::string actuator_behavior_name(ActuatorBehavior behavior) {
    const std::array names{"ServoToggle", "ServoPosition", "ServoBidirectional", "RelayToggle", "RelayPulse"};
    const auto index = static_cast<std::size_t>(behavior);
    return index < names.size() ? names[index] : "Invalid";
}

bool validate_actuator_definitions(const std::vector<ActuatorDefinition> &definitions, std::string &error) {
    std::set<std::string> ids;
    std::set<std::pair<bool, int>> outputs;
    if (definitions.size() > 8) {
        error = "at most eight configured actuators are supported";
        return false;
    }
    for (const auto &p : definitions) {
        if (!valid_definition(p) || !ids.insert(p.id).second ||
            !outputs.emplace(actuator_is_relay(p), p.channel).second) {
            error = "invalid actuator settings or duplicate ID/output; review bounds and explicit safe state";
            return false;
        }
    }
    return true;
}

bool parse_actuator_definitions(const Json &data, std::vector<ActuatorDefinition> &definitions, std::string &error) {
    std::vector<ActuatorDefinition> candidate;
    if (!data.is_array() || data.size() > 8) {
        error = "actuator_configs must be an array with at most eight entries";
        return false;
    }
    try {
        for (const auto &item : data) {
            ActuatorDefinition p;
            if (!read_fields(item, p)) {
                error = "each actuator must contain exactly the reviewed typed configuration fields";
                return false;
            }
            candidate.push_back(std::move(p));
        }
    } catch (const Json::exception &) {
        error = "actuator configuration fields have invalid types or values";
        return false;
    }
    if (!validate_actuator_definitions(candidate, error)) {
        return false;
    }
    definitions = std::move(candidate);
    return true;
}

Json serialize_actuator_definitions(const std::vector<ActuatorDefinition> &definitions) {
    auto data = Json::array();
    for (const auto &p : definitions) {
        data.push_back(serialize_definition(p));
    }
    return data;
}

} // namespace nomad::runtime::detail
