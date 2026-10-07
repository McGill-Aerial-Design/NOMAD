// SPDX-License-Identifier: Apache-2.0
#include "nomad/safety/vio_source.hpp"
#include "support/test_harness.hpp"

#ifdef NDEBUG
#undef NDEBUG
#endif

#include <cassert>
#include <chrono>
#include <string>
#include <stdexcept>
#include <cstdio>

namespace {

void test_vio_source_validator_accepts_valid_sample() {
    const nomad::safety::VioSourceValidator validator("test_vio");
    const nomad::safety::VioSample sample{
        true,
        0.95F,
        std::chrono::steady_clock::now(),
        "test_vio",
    };

    const auto result = validator.validate(sample);

    CHECK(result.valid);
}

void test_vio_source_validator_rejects_wrong_source() {
    const nomad::safety::VioSourceValidator validator("test_vio");
    const nomad::safety::VioSample sample{
        true,
        0.95F,
        std::chrono::steady_clock::now(),
        "other_vio",
    };

    const auto result = validator.validate(sample);

    CHECK(!result.valid);
    CHECK(result.message.find("source mismatch") != std::string::npos);
}

void test_vio_source_validator_rejects_unhealthy_sample() {
    const nomad::safety::VioSourceValidator validator("test_vio");
    const nomad::safety::VioSample sample{
        false,
        0.95F,
        std::chrono::steady_clock::now(),
        "test_vio",
    };

    const auto result = validator.validate(sample);

    CHECK(!result.valid);
    CHECK(result.message.find("unhealthy") != std::string::npos);
}

void test_vio_source_validator_rejects_invalid_confidence() {
    const nomad::safety::VioSourceValidator validator("test_vio");
    const nomad::safety::VioSample sample{
        true,
        1.5F,
        std::chrono::steady_clock::now(),
        "test_vio",
    };

    const auto result = validator.validate(sample);

    CHECK(!result.valid);
    CHECK(result.message.find("confidence") != std::string::npos);
}

void test_vio_source_validator_rejects_unconfigured_source() {
    const nomad::safety::VioSourceValidator validator("");
    const nomad::safety::VioSample sample{
        true,
        0.95F,
        std::chrono::steady_clock::now(),
        "test_vio",
    };

    const auto result = validator.validate(sample);

    CHECK(!result.valid);
    CHECK(result.message.find("not configured") != std::string::npos);
}

void test_vio_source_validator_rejects_missing_timestamp() {
    const nomad::safety::VioSourceValidator validator("test_vio");
    const nomad::safety::VioSample sample{
        true,
        0.95F,
        std::chrono::steady_clock::time_point{},
        "test_vio",
    };

    const auto result = validator.validate(sample);

    CHECK(!result.valid);
    CHECK(result.message.find("timestamp") != std::string::npos);
}

} // namespace

int main() {
    return nomad::test::run_tests([] {
        test_vio_source_validator_accepts_valid_sample();
        test_vio_source_validator_rejects_wrong_source();
        test_vio_source_validator_rejects_unhealthy_sample();
        test_vio_source_validator_rejects_invalid_confidence();
        test_vio_source_validator_rejects_unconfigured_source();
        test_vio_source_validator_rejects_missing_timestamp();
    });
}
