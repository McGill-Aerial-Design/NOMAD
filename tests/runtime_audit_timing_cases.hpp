// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "../src/runtime/audit_journal.hpp"
#include <cstdlib>

namespace nomad::runtime::detail {
struct AuditJournalTestAccess {
    static void enable(AuditJournal &journal, bool value = true) {
        journal.timing_enabled_ = value;
    }
    static bool enabled(const AuditJournal &journal) {
        return journal.timing_enabled_;
    }
    static nlohmann::json snapshot(const AuditJournal &journal) {
        return journal.timing_json();
    }
    static void close_file(AuditJournal &journal) {
        journal.file_.close();
    }
};
} // namespace nomad::runtime::detail

namespace {
using NativeJournal = nomad::runtime::detail::AuditJournal;
using NativeAccess = nomad::runtime::detail::AuditJournalTestAccess;

Json native_land_record(const char *event = "mutation_intent") {
    return {{"operation", "land"}, {"event", event}, {"result", "pending"}, {"credential", "PRIVATE_SENTINEL"}};
}

void native_timing_environment_fixture() {
    auto config = test_config();
    auto first = std::make_shared<NativeJournal>();
    CHECK(first->start(config.audit_directory, "synthetic"));
    CHECK(first->append(native_land_record()));
    CHECK(first->append(native_land_record("mutation_outcome")));
    auto last = first;
    first.reset();
    last->stop();
    last->stop();
    std::cerr << "native_probe_before_final_release\n";
    last.reset();
    std::cerr << "native_probe_after_final_release\n";
}

void test_native_timing_default_bounds_and_privacy() {
    auto config = test_config();
    NativeJournal journal;
    const auto *flag = std::getenv("NOMAD_QUALIFICATION_AUDIT_TIMING");
    CHECK(NativeAccess::enabled(journal) == (flag && flag[0] == '1' && flag[1] == '\0'));
    CHECK(journal.start(config.audit_directory, "synthetic"));
    NativeAccess::enable(journal, false);
    CHECK(journal.append(native_land_record()));
    CHECK(NativeAccess::snapshot(journal)["records"].empty());
    NativeAccess::enable(journal);
    CHECK(journal.append({{"event", "runtime_start"}}));
    for (int index = 0; index < 10; ++index) {
        CHECK(journal.append(native_land_record()));
    }
    const auto facts = NativeAccess::snapshot(journal);
    CHECK(facts["records"].size() == 8);
    CHECK(facts["overflow"] == true);
    CHECK(facts.dump().find("PRIVATE_SENTINEL") == std::string::npos);
    for (const auto &row : facts["records"]) {
        CHECK(row["lock_acquired_us"] <= row["before_native_us"]);
        CHECK(row["before_native_us"] <= row["after_native_us"]);
        CHECK(row["after_native_us"] <= row["append_finish_us"]);
        CHECK(row["native_success"] == true);
        CHECK(row["append_success"] == true);
    }
    journal.stop();
    journal.stop();
}

void test_native_timing_rejected_guard_and_native_failure() {
    for (int fault = 0; fault < 4; ++fault) {
        auto config = test_config();
        NativeJournal journal([fault](const std::string &) {
            if (fault == 3) {
                throw std::runtime_error("synthetic guard exception");
            }
            return fault != 0;
        });
        NativeAccess::enable(journal);
        CHECK(journal.start(config.audit_directory, "synthetic"));
        auto record = native_land_record();
        if (fault == 1) {
            NativeAccess::close_file(journal);
        } else if (fault == 2) {
            record["invalid_utf8"] = std::string(1, static_cast<char>(0xff));
        }
        CHECK(!journal.append(record));
        CHECK(!journal.healthy());
        bool sent = false;
        CHECK(!journal.admit_send([&] { sent = true; }));
        CHECK(!sent);
        const auto row = NativeAccess::snapshot(journal)["records"].at(0);
        CHECK(row["append_success"] == false);
        if (fault == 1) {
            CHECK(!row["before_native_us"].is_null());
            CHECK(!row["after_native_us"].is_null());
            CHECK(row["native_success"] == false);
        } else {
            CHECK(row["before_native_us"].is_null());
            CHECK(row["after_native_us"].is_null());
            CHECK(row["native_success"].is_null());
        }
        journal.stop();
    }
}

void test_native_timing_guard_delay_and_mutex_order() {
    auto config = test_config();
    std::promise<void> entered, release, second_started;
    const auto released = release.get_future().share();
    std::atomic_int guards{0};
    NativeJournal journal([&](const std::string &) {
        if (++guards == 1) {
            entered.set_value();
            return released.wait_for(std::chrono::seconds(5)) == std::future_status::ready;
        }
        return true;
    });
    NativeAccess::enable(journal);
    CHECK(journal.start(config.audit_directory, "synthetic"));
    auto first = std::async(std::launch::async, [&] { return journal.append(native_land_record()); });
    const bool reached = entered.get_future().wait_for(std::chrono::seconds(5)) == std::future_status::ready;
    auto second = std::async(std::launch::async, [&] {
        second_started.set_value();
        return journal.append(native_land_record("mutation_outcome"));
    });
    const bool started = second_started.get_future().wait_for(std::chrono::seconds(5)) == std::future_status::ready;
    const bool blocked = second.wait_for(std::chrono::milliseconds(40)) == std::future_status::timeout;
    release.set_value();
    const bool first_ok = first.get(), second_ok = second.get();
    CHECK(reached && started && blocked && first_ok && second_ok);
    const auto rows = NativeAccess::snapshot(journal)["records"];
    CHECK(rows.size() == 2);
    CHECK(rows[0]["ordinal"] < rows[1]["ordinal"]);
    CHECK(rows[0]["before_native_us"].get<std::int64_t>() >= 30000);
    for (const auto &row : rows) {
        CHECK(row["lock_acquired_us"] <= row["before_native_us"]);
        CHECK(row["before_native_us"] <= row["after_native_us"]);
        CHECK(row["after_native_us"] <= row["append_finish_us"]);
    }
    journal.stop();
}
} // namespace

void run_native_audit_timing_scenarios() {
    nomad::test::run_scenario("native_audit_default_bounds_privacy", test_native_timing_default_bounds_and_privacy);
    nomad::test::run_scenario("native_audit_guard_and_file_failure", test_native_timing_rejected_guard_and_native_failure);
    nomad::test::run_scenario("native_audit_mutex_and_guard_timing", test_native_timing_guard_delay_and_mutex_order);
}
