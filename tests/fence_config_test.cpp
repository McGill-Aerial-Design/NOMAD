// SPDX-License-Identifier: Apache-2.0
// Fence configuration parser tests (SR-FEN-02): the NOMAD-side keep-in fence
// must parse cleanly when configured, allow everything when unset, and fail
// closed (reject every target) on any malformed configured value.
#include "nomad/safety/fence_config.hpp"

#ifdef NDEBUG
#undef NDEBUG
#endif

#include <cassert>
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


bool rejects_everything(const nomad::safety::GlobalFencePolicy &policy) {
    const nomad::safety::GlobalPoint inside{45.0, -73.0};
    return !nomad::safety::evaluate_global_position(policy, inside).allowed;
}

void test_unset_polygon_allows_targets() {
    const auto policy = nomad::safety::load_fence_policy(nullptr, nullptr);

    CHECK(!policy.boundary.has_value());
    CHECK(nomad::safety::evaluate_global_position(policy, {1.0, 2.0}).allowed);
}

void test_valid_polygon_is_parsed_and_enforced() {
    const auto policy = nomad::safety::load_fence_policy("45.001,-73.001;45.001,-72.999;45.0,-72.999;45.0,-73.0",
                                                         "5.0");

    CHECK(policy.boundary.has_value());
    CHECK(policy.boundary->size() == 4);
    CHECK(policy.margin_m == 5.0);
    // The box spans 45.0..45.001 lat, so 45.0005 is inside; 45.002 is outside.
    CHECK(nomad::safety::evaluate_global_position(policy, {45.0005, -73.0}).allowed);
    CHECK(!nomad::safety::evaluate_global_position(policy, {45.002, -73.0}).allowed);
}

void test_default_margin_is_used_when_margin_unset() {
    const auto policy = nomad::safety::load_fence_policy("45.001,-73.001;45.001,-72.999;45.0,-73.0", nullptr);

    CHECK(policy.margin_m == 2.0);
}

void test_malformed_margin_fails_closed() {
    for (const char *bad : {"garbage", "-1.0", "inf", "nan"}) {
        const auto policy = nomad::safety::load_fence_policy("45.0,-73.0", bad);
        CHECK(rejects_everything(policy));
    }
}

void test_malformed_polygon_fails_closed() {
    for (const char *bad :
         {"garbage", "45.0,-73.0;45.0,-72.9", "45.0,-73.0;nan,-72.9;45.1,-72.8", "45.0,-73.0;;45.1,-72.8"}) {
        const auto policy = nomad::safety::load_fence_policy(bad, "2.0");
        CHECK(rejects_everything(policy));
    }
}

void test_polygon_with_too_few_vertices_fails_closed() {
    const auto policy = nomad::safety::load_fence_policy("45.0,-73.0;45.1,-73.0", "2.0");

    CHECK(rejects_everything(policy));
}

void test_empty_polygon_is_treated_as_unconfigured() {
    const auto policy = nomad::safety::load_fence_policy("", "2.0");

    CHECK(!policy.boundary.has_value());
}

} // namespace

int main() {
    try {
    test_unset_polygon_allows_targets();
    test_valid_polygon_is_parsed_and_enforced();
    test_default_margin_is_used_when_margin_unset();
    test_malformed_margin_fails_closed();
    test_malformed_polygon_fails_closed();
    test_polygon_with_too_few_vertices_fails_closed();
    test_empty_polygon_is_treated_as_unconfigured();
    } catch (const std::exception &error) {
        std::fprintf(stderr, "FAILED: %s\n", error.what());
        return 1;
    }
    return 0;
}
