// SPDX-License-Identifier: Apache-2.0
#include "geo.hpp"
#include "state_time.hpp"
#include "test_harness.hpp"

#include <array>
#include <chrono>
#include <cmath>
#include <limits>

namespace {

void test_geographic_baseline_values() {
    CHECK(nomad::vehicle::detail::distance_m(45.0, -73.0, 45.0, -73.0) == 0.0);
    CHECK(std::abs(nomad::vehicle::detail::distance_m(0.0, 0.0, 0.0, 0.001) - 111.19492664455875) <
          1.0e-9);
    CHECK(nomad::vehicle::detail::distance_m(90.0, 0.0, 90.0, 123.0) < 1.0e-8);
    CHECK(std::abs(nomad::vehicle::detail::distance_m(0.0, 179.9, 0.0, -179.9) - 22238.985328911924) <
          1.0e-8);
    CHECK(std::abs(nomad::vehicle::detail::distance_m(0.0, 0.0, 0.0, 179.999) - 20014975.601049278) <
          1.0e-3);

    const double near_antipodal_distance = nomad::vehicle::detail::distance_m(
        -59.26651785288814, 97.89764702473127, 59.26651785353999, -82.10235296968762);
    CHECK(std::isfinite(near_antipodal_distance));
    CHECK(std::abs(near_antipodal_distance - 20015086.79602057) < 0.5);

    const auto not_a_number = std::numeric_limits<double>::quiet_NaN();
    const auto infinity = std::numeric_limits<double>::infinity();
    CHECK(std::isnan(nomad::vehicle::detail::distance_m(not_a_number, 0.0, 1.0, 1.0)));
    CHECK(std::isnan(nomad::vehicle::detail::distance_m(0.0, infinity, 0.0, 0.0)));
}

struct DistanceBoundary {
    double radius_m;
    double below_longitude_deg;
    double at_longitude_deg;
    double above_longitude_deg;
};

void test_geographic_threshold_neighbors() {
    constexpr std::array boundaries{
        DistanceBoundary{5.0, 0.000044957087079877336, 0.000044966080295936529, 0.000044975073511995721},
        DistanceBoundary{45.0, 0.00040468572944736959, 0.00040469472266342876, 0.00040470371587948792},
        DistanceBoundary{55.0, 0.00049461789003924265, 0.00049462688325530187, 0.00049463587647136098},
        DistanceBoundary{100.0, 0.00089931261270267135, 0.00089932160591873057, 0.00089933059913478979},
    };

    for (const auto &boundary : boundaries) {
        const auto below = nomad::vehicle::detail::distance_m(0.0, 0.0, 0.0, boundary.below_longitude_deg);
        const auto at = nomad::vehicle::detail::distance_m(0.0, 0.0, 0.0, boundary.at_longitude_deg);
        const auto above = nomad::vehicle::detail::distance_m(0.0, 0.0, 0.0, boundary.above_longitude_deg);

        CHECK(below < boundary.radius_m);
        CHECK(std::abs(below - (boundary.radius_m - 0.001)) < 1.0e-9);
        CHECK(std::abs(at - boundary.radius_m) < 1.0e-9);
        CHECK(above > boundary.radius_m);
        CHECK(std::abs(above - (boundary.radius_m + 0.001)) < 1.0e-9);
    }
}

void test_timestamp_freshness_boundaries() {
    using Clock = std::chrono::steady_clock;
    const Clock::time_point now{std::chrono::seconds(100)};
    constexpr auto timeout = std::chrono::milliseconds(250);
    const auto one_tick = Clock::duration{1};

    CHECK(!nomad::vehicle::detail::timestamp_is_fresh(Clock::time_point{}, timeout, now));
    CHECK(!nomad::vehicle::detail::timestamp_is_fresh(now + one_tick, timeout, now));
    CHECK(nomad::vehicle::detail::timestamp_is_fresh(now - timeout, timeout, now));
    CHECK(!nomad::vehicle::detail::timestamp_is_fresh(now - timeout - one_tick, timeout, now));
}

} // namespace

int main() {
    return nomad::test::run_tests([] {
        test_geographic_baseline_values();
        test_geographic_threshold_neighbors();
        test_timestamp_freshness_boundaries();
    });
}
