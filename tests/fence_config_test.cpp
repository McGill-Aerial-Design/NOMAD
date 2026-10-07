// SPDX-License-Identifier: Apache-2.0
// Fence configuration parser tests (SR-FEN-02): the NOMAD-side keep-in fence
// must parse cleanly when configured, allow everything when unset, and fail
// closed (reject every target) on any malformed configured value.
#include "nomad/safety/fence_config.hpp"
#include "support/test_harness.hpp"

#ifdef NDEBUG
#undef NDEBUG
#endif

#include <limits>
#include <string>
#include <stdexcept>
#include <cstdio>

namespace {

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

void test_local_polygon_with_nonfinite_vertex_fails_closed() {
    const std::vector<nomad::safety::Point2d> polygon{
        {-1.0, -1.0},
        {1.0, -1.0},
        {std::numeric_limits<double>::quiet_NaN(), 1.0},
        {-1.0, 1.0},
    };

    const auto decision = nomad::safety::evaluate_position({polygon, 0.0}, {0.0, 0.0});

    CHECK(!decision.allowed);
    CHECK(decision.reason == "fence");
}

// Local containment geometry: point-in-polygon, inset margin and boundary
// distance are independent of the parser and live here with the other
// polygon-shape tests.
void test_geofence_contains_only_safe_targets() {
    const std::vector<nomad::safety::Point2d> square{
        {-5.0, -5.0},
        {5.0, -5.0},
        {5.0, 5.0},
        {-5.0, 5.0},
    };
    CHECK(nomad::safety::point_in_polygon({0.0, 0.0}, square));
    CHECK(!nomad::safety::point_in_polygon({10.0, 0.0}, square));
    CHECK(nomad::safety::is_contained({0.0, 0.0}, square, 2.0));
    CHECK(!nomad::safety::is_contained({4.0, 0.0}, square, 2.0));
    CHECK(nomad::safety::distance_to_boundary({0.0, 0.0}, square) == 5.0);
}

void test_geofence_rejects_invalid_configuration_and_targets() {
    const auto malformed =
        nomad::safety::evaluate_position({std::vector<nomad::safety::Point2d>{{0.0, 0.0}}, 0.0}, {0.0, 0.0});
    CHECK(!malformed.allowed);
    CHECK(malformed.reason == "fence");

    const auto nonfinite =
        nomad::safety::evaluate_position({std::nullopt, 0.0}, {std::numeric_limits<double>::quiet_NaN(), 0.0});
    CHECK(!nonfinite.allowed);
    CHECK(nonfinite.reason == "nonfinite");

    const auto unconfigured = nomad::safety::evaluate_position({std::nullopt, 0.0}, {100.0, -100.0});
    CHECK(unconfigured.allowed);
}

void test_global_geofence_projects_meters() {
    const nomad::safety::GlobalFencePolicy policy{
        std::vector<nomad::safety::GlobalPoint>{
            {45.0, -73.0},
            {45.0, -72.9999},
            {45.0001, -72.9999},
            {45.0001, -73.0},
        },
        1.0,
    };
    CHECK(nomad::safety::evaluate_global_position(policy, {45.00005, -72.99995}).allowed);
    CHECK(!nomad::safety::evaluate_global_position(policy, {45.002, -72.99995}).allowed);
}

void test_empty_polygon_is_treated_as_unconfigured() {
    const auto policy = nomad::safety::load_fence_policy("", "2.0");

    CHECK(!policy.boundary.has_value());
}

} // namespace

int main() {
    return nomad::test::run_tests([] {
        test_unset_polygon_allows_targets();
        test_valid_polygon_is_parsed_and_enforced();
        test_default_margin_is_used_when_margin_unset();
        test_malformed_margin_fails_closed();
        test_malformed_polygon_fails_closed();
        test_polygon_with_too_few_vertices_fails_closed();
        test_local_polygon_with_nonfinite_vertex_fails_closed();
        test_geofence_contains_only_safe_targets();
        test_geofence_rejects_invalid_configuration_and_targets();
        test_global_geofence_projects_meters();
        test_empty_polygon_is_treated_as_unconfigured();
    });
}
