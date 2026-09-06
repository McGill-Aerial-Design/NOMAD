// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>

namespace nomad::safety {

struct VioSample {
    bool healthy{false};
    float confidence{0.0F};
    std::chrono::steady_clock::time_point timestamp{};
    std::string source;
};

struct VioValidationResult {
    bool valid{false};
    std::string message;
};

class VioSourceValidator {
  public:
    explicit VioSourceValidator(std::string expected_source);

    VioValidationResult validate(const VioSample &sample) const;

  private:
    std::string expected_source_;
};

} // namespace nomad::safety
