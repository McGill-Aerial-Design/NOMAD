// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <map>
#include <string>
#include <string_view>

namespace nomad::runtime::detail {

bool valid_credentials(const std::map<std::string, std::string> &credentials);
bool load_credentials(const std::string &path, std::map<std::string, std::string> &credentials);
std::string redact_credentials(std::string text, const std::map<std::string, std::string> &credentials);

} // namespace nomad::runtime::detail
