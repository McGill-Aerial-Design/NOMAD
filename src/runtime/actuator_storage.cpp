// SPDX-License-Identifier: Apache-2.0
#include "actuator_storage.hpp"

#include "actuator_config.hpp"
#include "protected_file.hpp"

#include <filesystem>
#include <set>
#include <vector>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#endif

namespace nomad::runtime::detail {
namespace {

Json parse_document(const std::string &text) {
    std::vector<std::set<std::string>> objects;
    bool duplicate = false;
    auto document = Json::parse(text, [&](int, auto event, auto &value) {
        if (event == Json::parse_event_t::object_start) {
            objects.emplace_back();
        } else if (event == Json::parse_event_t::object_end) {
            objects.pop_back();
        } else if (event == Json::parse_event_t::key &&
                   !objects.back().insert(value.template get<std::string>()).second) {
            duplicate = true;
        }
        return true;
    }, false);
    return duplicate ? Json(nullptr) : document;
}

bool replace_file(const std::filesystem::path &source, const std::filesystem::path &destination) {
#ifdef _WIN32
    return MoveFileExW(source.c_str(), destination.c_str(),
                       MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH) != 0;
#else
    std::error_code error;
    std::filesystem::rename(source, destination, error);
    return !error;
#endif
}

bool validate_destination(const std::filesystem::path &path) {
    if (!path.is_absolute() || !std::filesystem::is_directory(path.parent_path())) {
        return false;
    }
    std::error_code error;
    const auto status = std::filesystem::symlink_status(path, error);
    if (status.type() == std::filesystem::file_type::not_found) {
        return true;
    }
    if (error || !std::filesystem::is_regular_file(status)) {
        return false;
    }
    ProtectedFile existing;
    return existing.open(path.string(), FileMode::Read);
}

} // namespace

bool load_actuator_configuration(const std::string &path, std::vector<ActuatorDefinition> &definitions,
                                 std::string &error) {
    if (path.empty()) {
        definitions.clear();
        return true;
    }
    ProtectedFile file;
    std::string text;
    if (!std::filesystem::path(path).is_absolute() || !file.open(path, FileMode::Read) || !file.read(text, 32768)) {
        error = "actuator configuration requires an absolute protected readable file";
        return false;
    }
    const auto document = parse_document(text);
    if (!document.is_object() || document.size() != 1 || !document.contains("actuator_configs")) {
        error = "actuator configuration must contain only actuator_configs with no duplicate keys";
        return false;
    }
    return parse_actuator_definitions(document["actuator_configs"], definitions, error);
}

bool save_actuator_configuration(const std::string &path, const std::vector<ActuatorDefinition> &definitions,
                                 std::string &error, bool &replacement_attempted,
                                 const std::function<bool()> &directory_sync_guard) {
    replacement_attempted = false;
    const std::filesystem::path destination(path);
    if (!validate_destination(destination)) {
        error = "actuator configuration destination must be an absolute protected file in an existing directory";
        return false;
    }
    const auto temporary = destination.string() + ".tmp";
    ProtectedFile staged;
    const Json document{{"actuator_configs", serialize_actuator_definitions(definitions)}};
    if (!staged.open(temporary, FileMode::Create)) {
        error = "actuator configuration staging failed; the existing file was preserved";
        return false;
    }
    const bool written = staged.append(document.dump(2) + "\n");
    staged.close();
    replacement_attempted = written;
    if (!written || !replace_file(temporary, destination)) {
        std::error_code ignored;
        std::filesystem::remove(temporary, ignored);
        error = "actuator configuration replacement failed; the existing file was preserved";
        return false;
    }
    if (!sync_directory(destination.parent_path().string()) || (directory_sync_guard && !directory_sync_guard())) {
        error = "actuator configuration was replaced but directory synchronization failed; restart and review it";
        return false;
    }
    return true;
}

} // namespace nomad::runtime::detail
