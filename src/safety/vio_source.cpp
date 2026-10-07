// SPDX-License-Identifier: Apache-2.0
#include "nomad/safety/vio_source.hpp"

#include <cmath>
#include <utility>

namespace nomad::safety {

VioSourceValidator::VioSourceValidator(std::string expected_source) : expected_source_(std::move(expected_source)) {}

VioValidationResult VioSourceValidator::validate(const VioSample &sample) const {
    if (expected_source_.empty()) {
        return {false, "expected VIO source is not configured"};
    }
    if (sample.source != expected_source_) {
        return {false, "VIO source mismatch: expected '" + expected_source_ + "', got '" + sample.source + "'"};
    }
    if (!sample.healthy) {
        return {false, "VIO sample is unhealthy"};
    }
    if (!std::isfinite(sample.confidence) || sample.confidence < 0.0F || sample.confidence > 1.0F) {
        return {false, "VIO confidence must be finite and between zero and one"};
    }
    if (sample.timestamp == std::chrono::steady_clock::time_point{}) {
        return {false, "VIO timestamp must be set"};
    }
    return {true, "VIO sample is valid"};
}

} // namespace nomad::safety
