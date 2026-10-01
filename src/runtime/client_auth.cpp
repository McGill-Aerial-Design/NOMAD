// SPDX-License-Identifier: Apache-2.0
#include "client_auth.hpp"
#include "protected_file.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cstdint>
#include <set>

namespace nomad::runtime::detail {
namespace {

bool valid_identity(const std::string &identity) {
    return !identity.empty() && identity.size() <= 64 &&
           std::all_of(identity.begin(), identity.end(), [](unsigned char c) {
               return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
                      (c >= '0' && c <= '9') || c == '-' || c == '_' || c == ':' || c == '.';
           });
}

bool valid_token(std::string_view token) {
    return token.size() == 64 && std::all_of(token.begin(), token.end(), [](unsigned char c) {
        return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f');
    });
}

} // namespace

bool valid_credentials(const std::map<std::string, std::string> &credentials) {
    if (credentials.empty() || credentials.size() > 32) {
        return false;
    }
    std::set<std::string> tokens;
    for (const auto &[identity, token] : credentials) {
        if (!valid_identity(identity) || !valid_token(token) || !tokens.insert(token).second) {
            return false;
        }
    }
    return true;
}

bool load_credentials(const std::string &path, std::map<std::string, std::string> &credentials) {
    credentials.clear();
    ProtectedFile file;
    std::string content;
    if (!file.open(path, FileMode::Read) || !file.read(content, 16384)) {
        return false;
    }
    std::set<std::string> keys;
    bool duplicate = false;
    const auto body = nlohmann::json::parse(content, [&keys, &duplicate](int, auto event, auto &value) {
        if (event == nlohmann::json::parse_event_t::key && !keys.insert(value.template get<std::string>()).second) {
            duplicate = true;
        }
        return true;
    }, false);
    if (duplicate || !body.is_object()) {
        return false;
    }
    std::map<std::string, std::string> loaded;
    for (const auto &[identity, token] : body.items()) {
        if (!token.is_string()) {
            return false;
        }
        loaded[identity] = token.get<std::string>();
    }
    if (!valid_credentials(loaded)) {
        return false;
    }
    credentials = std::move(loaded);
    return true;
}

std::string redact_credentials(std::string text, const std::map<std::string, std::string> &credentials) {
    for (const auto &[identity, token] : credentials) {
        if (token.empty()) {
            continue;
        }
        std::size_t offset = 0;
        while ((offset = text.find(token, offset)) != std::string::npos) {
            text.replace(offset, token.size(), "[redacted]");
            offset += 10;
        }
    }
    return text;
}

} // namespace nomad::runtime::detail
