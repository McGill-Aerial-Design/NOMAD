// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <string>
#include <string_view>

namespace nomad::runtime::detail {

std::string make_proof(std::string_view secret, std::string_view payload);
bool equal_proof(std::string_view left, std::string_view right);
std::string make_nonce();

} // namespace nomad::runtime::detail
