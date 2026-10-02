// SPDX-License-Identifier: Apache-2.0
#include "service_config.hpp"
#include "protected_file.hpp"

#include <nlohmann/json.hpp>

#include <cstdlib>
#include <filesystem>
#include <set>
#include <string>

namespace {

const std::set<std::string> allowed{
    "NOMAD_MAVLINK_ENDPOINT", "NOMAD_RUNTIME_IPC_PORT", "NOMAD_CLIENT_CREDENTIALS_FILE",
    "NOMAD_AUDIT_DIRECTORY", "NOMAD_API_KEY", "NOMAD_FENCE_POLYGON", "NOMAD_FENCE_MARGIN_M",
    "NOMAD_VELOCITY_MAX_XY", "NOMAD_VELOCITY_MAX_Z", "NOMAD_VELOCITY_MAX_YAW_RATE"};

nlohmann::json parse_configuration(const std::string &text) {
    std::set<std::string> keys;
    bool duplicate = false;
    const auto values = nlohmann::json::parse(text, [&keys, &duplicate](int, auto event, auto &value) {
        if (event == nlohmann::json::parse_event_t::key && !keys.insert(value.template get<std::string>()).second) {
            duplicate = true;
        }
        return true;
    }, false);
    return duplicate ? nlohmann::json(nullptr) : values;
}

bool valid_configuration(const nlohmann::json &values) {
    if (!values.is_object()) {
        return false;
    }
    for (const auto &[key, value] : values.items()) {
        if (!allowed.contains(key) || !value.is_string() || value.get<std::string>().find('\0') != std::string::npos) {
            return false;
        }
    }
    for (const auto &key : {"NOMAD_MAVLINK_ENDPOINT", "NOMAD_RUNTIME_IPC_PORT",
                            "NOMAD_CLIENT_CREDENTIALS_FILE", "NOMAD_AUDIT_DIRECTORY"}) {
        if (!values.contains(key) || values[key].get<std::string>().empty()) {
            return false;
        }
    }
    for (const auto &key : {"NOMAD_CLIENT_CREDENTIALS_FILE", "NOMAD_AUDIT_DIRECTORY"}) {
        if (!std::filesystem::path(values[key].get<std::string>()).is_absolute()) {
            return false;
        }
    }
    return true;
}

} // namespace

bool load_service_environment(const std::string &path) {
    nomad::runtime::detail::ProtectedFile file;
    std::string text;
    if (!std::filesystem::path(path).is_absolute() ||
        !file.open(path, nomad::runtime::detail::FileMode::Read) || !file.read(text, 16384)) {
        return false;
    }
    const auto values = parse_configuration(text);
    if (!valid_configuration(values)) {
        return false;
    }
    for (const auto &key : allowed) {
        const auto value = values.value(key, std::string{});
#ifdef _WIN32
        if (_putenv_s(key.c_str(), value.c_str()) != 0) {
#else
        if (setenv(key.c_str(), value.c_str(), 1) != 0) {
#endif
            return false;
        }
    }
    return true;
}
