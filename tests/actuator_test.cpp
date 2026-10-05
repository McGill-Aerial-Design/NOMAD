// SPDX-License-Identifier: Apache-2.0
#include "../src/runtime/actuator_state.hpp"
#include "../src/runtime/actuator_sequence.hpp"
#include "support/test_harness.hpp"

#include <limits>
#include <stdexcept>

using namespace nomad::runtime;
using namespace nomad::runtime::detail;
using namespace std::chrono_literals;

namespace {

ActuatorDefinition definition(ActuatorBehavior behavior = ActuatorBehavior::RelayPulse) {
    ActuatorDefinition p;
    p.id = "test-output";
    p.name = "Operator name";
    p.behavior = behavior;
    p.channel = 2;
    p.pulse_ms = 50;
    p.confirmation_count = 2;
    return p;
}

ActuatorInput input(std::string operation, std::string source = "ui", int slot = -1) {
    return {"test-output", std::move(operation), std::move(source), slot, {}, "owner:session:generation"};
}

void recover(ActuatorState &state, const std::string &authority = "owner:session:generation") {
    auto request = input("safe");
    request.authority = authority;
    const auto safe = state.begin(request, std::chrono::steady_clock::now());
    CHECK(safe.allowed && safe.execute && safe.safe);
    int commands = 0;
    const auto sequence = run_actuator_sequence(safe, [&](bool safe_edge, int) {
        CHECK(safe_edge);
        ++commands;
        return ActuatorStageResult{{true, "explicit successful safe"}, "success"};
    }, [&](int) { CHECK(false); });
    CHECK(commands == 1);
    state.finish(safe, "not-attempted", sequence.initial.outcome, sequence.outcome, false);
    CHECK(state.state("test-output")["recovery_required"] == false);
}

ActuatorDecision activate(ActuatorState &state) {
    const auto first = state.begin(input("activate"), std::chrono::steady_clock::now());
    CHECK(first.allowed && !first.execute);
    const auto final = state.begin(input("activate"), std::chrono::steady_clock::now());
    CHECK(final.allowed && final.execute && !final.safe);
    return final;
}

void test_staged_recovery_matrix() {
    for (const auto *off : {"success", "rejected", "failed-before-send", "failed", "interrupted", "unknown"}) {
        ActuatorState state({definition()});
        CHECK(!state.begin(input("activate"), std::chrono::steady_clock::now()).allowed);
        recover(state);
        const auto plan = activate(state);
        std::vector<bool> commands;
        int waits = 0;
        const auto sequence = run_actuator_sequence(plan, [&](bool safe, int) {
            commands.push_back(safe);
            const auto outcome = safe ? std::string(off) : std::string("success");
            return ActuatorStageResult{{outcome == "success", "test stage"}, outcome};
        }, [&](int duration) {
            CHECK(duration == 50);
            ++waits;
        });
        CHECK(commands == std::vector<bool>({false, true}));
        CHECK(waits == 1);
        state.finish(plan, sequence.initial.outcome, sequence.safe.outcome,
              sequence.outcome, sequence.activation_success);
        const auto snapshot = state.state("test-output");
        CHECK(snapshot["pulse_on_succeeded"] == true);
        CHECK(snapshot["safe_outcome"] == off);
        CHECK(snapshot["recovery_required"] == (std::string(off) != "success"));
        if (std::string(off) == "success") {
            continue;
        }
        CHECK(!state.begin(input("activate"), std::chrono::steady_clock::now()).allowed);
        CHECK(commands.size() == 2);
        auto safe = state.begin(input("safe"), std::chrono::steady_clock::now());
        CHECK(safe.execute && safe.safe);
        const auto recovery = run_actuator_sequence(safe, [&](bool safe_edge, int) {
            CHECK(safe_edge);
            commands.push_back(safe_edge);
            return ActuatorStageResult{{false, "explicit failed safe"}, off};
        }, [&](int) { CHECK(false); });
        CHECK(commands.size() == 3);
        state.finish(safe, "not-attempted", recovery.initial.outcome, recovery.outcome, false);
        CHECK(state.state("test-output")["recovery_required"] == true);
        recover(state);
    }
}

void test_initial_failure_and_final_audit_failure() {
    for (const auto *on : {"rejected", "failed-before-send", "failed", "interrupted", "unknown"}) {
        ActuatorState state({definition()});
        recover(state);
        const auto plan = activate(state);
        int commands = 0;
        const auto sequence = run_actuator_sequence(plan, [&](bool safe, int) {
            CHECK(!safe);
            ++commands;
            return ActuatorStageResult{{false, "initial ON failed"}, on};
        }, [&](int) { CHECK(false); });
        CHECK(commands == 1);
        state.finish(plan, sequence.initial.outcome, sequence.safe.outcome,
              sequence.outcome, sequence.activation_success);
        const auto snapshot = state.state("test-output");
        CHECK(snapshot["pulse_on_succeeded"] == false);
        CHECK(snapshot["safe_outcome"] == "not-attempted");
        CHECK(snapshot["recovery_required"] == (std::string(on) == "interrupted" || std::string(on) == "unknown"));
    }
    ActuatorState state({definition()});
    recover(state);
    auto plan = activate(state);
    state.finish(plan, "success", "success", "success", true);
    state.require_recovery("test-output");
    CHECK(state.state("test-output")["recovery_required"] == true);
    CHECK(!state.begin(input("activate"), std::chrono::steady_clock::now()).allowed);
}

void test_staged_exceptions_leave_explicit_recovery_available() {
    for (const bool throw_during_wait : {false, true}) {
        ActuatorState state({definition()});
        recover(state);
        const auto plan = activate(state);
        int commands = 0;
        const auto sequence = run_actuator_sequence(plan, [&](bool safe, int) {
            ++commands;
            if (safe) {
                throw std::runtime_error("test OFF exception");
            }
            return ActuatorStageResult{{true, "ON software success"}, "success"};
        }, [&](int) {
            if (throw_during_wait) {
                throw std::runtime_error("test wait exception");
            }
        });
        CHECK(commands == (throw_during_wait ? 1 : 2));
        CHECK(sequence.activation_success && sequence.outcome == "unknown");
        state.finish(plan, sequence.initial.outcome, sequence.safe.outcome,
              sequence.outcome, sequence.activation_success);
        CHECK(state.state("test-output")["pending"] == false);
        CHECK(state.state("test-output")["recovery_required"] == true);
        CHECK(!state.begin(input("activate"), std::chrono::steady_clock::now()).allowed);
        recover(state);
    }
}

void test_software_success_remains_distinct_from_disposition() {
    ActuatorState state({definition()});
    recover(state);
    const auto plan = activate(state);
    const auto sequence = run_actuator_sequence(plan, [&](bool safe, int) {
        return ActuatorStageResult{{true, "software acknowledgement observed", true}, safe ? "interrupted" : "success"};
    }, [](int) {});
    CHECK(!sequence.success && sequence.observed_command_success);
    state.finish(plan, sequence.initial.outcome, sequence.safe.outcome, sequence.outcome,
                 sequence.activation_success, sequence.observed_command_success);
    CHECK(state.state("test-output")["software_command_success"] == true);
    CHECK(state.state("test-output")["recovery_required"] == true);
    state.require_recovery("test-output");
    CHECK(state.state("test-output")["software_command_success"] == true);
    CHECK(!state.begin(input("activate"), std::chrono::steady_clock::now()).allowed);
    recover(state);
}

void test_hid_and_authority_bound_confirmation() {
    ActuatorState state({definition()});
    recover(state);
    const auto now = std::chrono::steady_clock::now();
    CHECK(!state.begin(input("neutral"), now).allowed);
    CHECK(!state.begin(input("activate", "hid", 32), now).allowed);
    auto edge = input("activate", "hid", 0);
    CHECK(!state.begin(edge, now).execute);
    CHECK(state.state(edge.id)["confirmation_remaining"] == 2);
    CHECK(!state.begin(input("neutral", "hid", 0), now).execute);
    CHECK(!state.begin(edge, now).execute);
    CHECK(state.state(edge.id)["confirmation_remaining"] == 1);
    CHECK(!state.begin(edge, now).execute);
    CHECK(!state.begin(input("neutral", "hid", 1), now).execute);
    CHECK(!state.begin(input("activate", "hid", 1), now).execute);
    CHECK(state.state(edge.id)["confirmation_remaining"] == 1);
    edge.authority = "new-owner:new-session:new-generation";
    CHECK(!state.begin(edge, now).execute);
    CHECK(state.state(edge.id)["confirmation_remaining"] == 2);
    CHECK(state.state(edge.id)["recovery_required"] == true);
    recover(state, edge.authority);
    auto neutral = edge;
    neutral.operation = "neutral";
    state.begin(neutral, now);
    CHECK(!state.begin(edge, now).execute);
    CHECK(!state.begin(edge, now + 3001ms).execute);
    CHECK(state.state(edge.id)["confirmation_remaining"] == 2);
    ActuatorState expiring({definition()});
    recover(expiring);
    CHECK(!expiring.begin(input("activate"), now).execute);
    CHECK(expiring.discover("owner:session:generation", now + 3001ms)[0]["state"]["confirmation_remaining"] == 2);
}

void test_authority_change_requires_fresh_explicit_safe() {
    ActuatorState state({definition()});
    recover(state);
    auto edge = input("activate");
    edge.authority = "replacement-owner:replacement-session:replacement-generation";
    const auto discovery = state.discover(edge.authority);
    CHECK(discovery[0]["state"]["recovery_required"] == true);
    CHECK(!state.begin(edge, std::chrono::steady_clock::now()).allowed);
    recover(state, edge.authority);
    CHECK(state.begin(edge, std::chrono::steady_clock::now()).allowed);
    edge.authority = "third-owner:third-session:third-generation";
    CHECK(!state.begin(edge, std::chrono::steady_clock::now()).allowed);
    CHECK(state.state(edge.id)["recovery_required"] == true);
    recover(state, edge.authority);
}

void test_direction_release_preserves_confirmations_and_supersedes_old_input() {
    auto p = definition(ActuatorBehavior::ServoBidirectional);
    p.safe_pwm = p.pwm_neutral;
    ActuatorState state({p});
    recover(state);
    auto neutral = input("neutral", "hid", 0);
    neutral.sequence = 1;
    state.begin(neutral, std::chrono::steady_clock::now());
    auto press = input("positive", "hid", 0);
    press.sequence = 2;
    CHECK(!state.begin(press, std::chrono::steady_clock::now()).execute);
    auto release = input("release_input", "hid", 0);
    release.sequence = 3;
    CHECK(!state.release_input(release).execute);
    CHECK(state.state(p.id)["confirmation_remaining"] == 1);
    neutral.sequence = 4;
    state.begin(neutral, std::chrono::steady_clock::now());
    press.sequence = 5;
    release.sequence = 6;
    CHECK(!state.release_input(release).execute);
    CHECK(!state.begin(press, std::chrono::steady_clock::now()).allowed);
    CHECK(state.state(p.id)["pending"] == false);
    CHECK(state.state(p.id)["confirmation_remaining"] == 1);
    neutral.sequence = 7;
    state.begin(neutral, std::chrono::steady_clock::now());
    press.sequence = 8;
    CHECK(state.begin(press, std::chrono::steady_clock::now()).execute);
    release.sequence = 9;
    CHECK(state.release_input(release).execute);
}

void test_configuration_validation_and_revision() {
    const auto valid = serialize_actuator_definitions({definition()});
    std::vector<ActuatorDefinition> parsed;
    std::string error;
    CHECK(parse_actuator_definitions(valid, parsed, error));
    CHECK(serialize_actuator_definitions(parsed) == valid);
    for (const auto *key : {"channel", "pwm_min", "confirmation_count"}) {
        for (const Json &number : {Json(std::numeric_limits<std::uint64_t>::max()),
            Json(std::numeric_limits<std::int64_t>::max()), Json(std::uint64_t{4294967297})}) {
            auto invalid = valid;
            invalid[0][key] = number;
            CHECK(!parse_actuator_definitions(invalid, parsed, error));
        }
    }
    for (const auto *key : {"id", "name", "primary_label", "secondary_label"}) {
        auto invalid = valid;
        invalid[0][key] = "  ";
        CHECK(!parse_actuator_definitions(invalid, parsed, error));
    }
    auto invalid = valid;
    invalid[0]["pulse_ms"] = 1501;
    CHECK(!parse_actuator_definitions(invalid, parsed, error));
    invalid = valid;
    invalid[0]["channel"] = 4294967298ULL;
    CHECK(!parse_actuator_definitions(invalid, parsed, error));
    invalid = valid;
    invalid.push_back(invalid[0]);
    CHECK(!parse_actuator_definitions(invalid, parsed, error));
    ActuatorState state({definition()});
    const auto initial = state.state("test-output")["state_revision"].get<std::uint64_t>();
    recover(state);
    CHECK(state.can_configure());
    state.replace({definition()});
    CHECK(state.state("test-output")["state_revision"].get<std::uint64_t>() > initial);
    CHECK(state.state("test-output")["recovery_required"] == true);
    CHECK(!state.can_configure());
}

} // namespace

int main() {
    return nomad::test::run_tests([] {
        test_staged_recovery_matrix();
        test_initial_failure_and_final_audit_failure();
        test_staged_exceptions_leave_explicit_recovery_available();
        test_software_success_remains_distinct_from_disposition();
        test_hid_and_authority_bound_confirmation();
        test_authority_change_requires_fresh_explicit_safe();
        test_direction_release_preserves_confirmations_and_supersedes_old_input();
        test_configuration_validation_and_revision();
    });
}
