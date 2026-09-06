// SPDX-License-Identifier: Apache-2.0
#include "nomad/safety/vio_source.hpp"

#ifdef NDEBUG
#undef NDEBUG
#endif

#include <cassert>
#include <chrono>
#include <string>
#include <stdexcept>
#include <cstdio>

namespace {

// A failing assert on Windows opens a dialog that blocks unattended CI runs,
// so main() runs the tests inside a try/catch and reports failures on stderr.
void check_impl(bool ok, const char *condition, int line) {
    if (!ok) {
        throw std::runtime_error(std::string("check failed at line ") + std::to_string(line) + ": " + condition);
    }
}

#define CHECK(condition) check_impl(static_cast<bool>(condition), #condition, __LINE__)


void test_vio_source_validator_accepts_valid_sample() {
    const nomad::safety::VioSourceValidator validator("zed2i");
    const nomad::safety::VioSample sample{
        true,
        0.95F,
        std::chrono::steady_clock::now(),
        "zed2i",
    };

    const auto result = validator.validate(sample);

    CHECK(result.valid);
}

void test_vio_source_validator_rejects_wrong_source() {
    const nomad::safety::VioSourceValidator validator("zed2i");
    const nomad::safety::VioSample sample{
        true,
        0.95F,
        std::chrono::steady_clock::now(),
        "realsense",
    };

    const auto result = validator.validate(sample);

    CHECK(!result.valid);
    CHECK(result.message.find("source mismatch") != std::string::npos);
}

void test_vio_source_validator_rejects_invalid_confidence() {
    const nomad::safety::VioSourceValidator validator("zed2i");
    const nomad::safety::VioSample sample{
        true,
        1.5F,
        std::chrono::steady_clock::now(),
        "zed2i",
    };

    const auto result = validator.validate(sample);

    CHECK(!result.valid);
    CHECK(result.message.find("confidence") != std::string::npos);
}

void test_vio_source_validator_rejects_missing_timestamp() {
    const nomad::safety::VioSourceValidator validator("zed2i");
    const nomad::safety::VioSample sample{
        true,
        0.95F,
        std::chrono::steady_clock::time_point{},
        "zed2i",
    };

    const auto result = validator.validate(sample);

    CHECK(!result.valid);
    CHECK(result.message.find("timestamp") != std::string::npos);
}

} // namespace

int main() {
    try {
    test_vio_source_validator_accepts_valid_sample();
    test_vio_source_validator_rejects_wrong_source();
    test_vio_source_validator_rejects_invalid_confidence();
    test_vio_source_validator_rejects_missing_timestamp();
    } catch (const std::exception &error) {
        std::fprintf(stderr, "FAILED: %s\n", error.what());
        return 1;
    }
    return 0;
}
