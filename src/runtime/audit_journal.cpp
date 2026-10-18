// SPDX-License-Identifier: Apache-2.0
#include "audit_journal.hpp"

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <sstream>

namespace nomad::runtime::detail {

AuditJournal::AuditJournal(std::function<bool(const std::string &)> guard) : guard_(std::move(guard)) {
    const auto *enabled = std::getenv("NOMAD_QUALIFICATION_AUDIT_TIMING");
    timing_enabled_ = enabled && enabled[0] == '1' && enabled[1] == '\0';
}

AuditJournal::~AuditJournal() noexcept {
    if (!timing_enabled_) {
        return;
    }
    // The final shared owner outlives the runtime workers and connection-owned captures.
    // No output is performed by append/stop while a caller may hold authority locks.
    try {
        std::cerr << timing_json().dump() << '\n';
        if (admission_timing_count_ != 0) {
            std::cerr << timing_json(true).dump() << '\n';
        }
    } catch (...) {
        // Missing diagnostic delivery must not change audit or shutdown outcomes.
    }
}

std::optional<AuditJournal::TimedEvent> AuditJournal::timed_event(const nlohmann::json &record) {
    const auto operation = record.find("operation");
    const auto event = record.find("event");
    if (operation == record.end() || !operation->is_string() || event == record.end() || !event->is_string()) {
        return std::nullopt;
    }
    const auto &name = event->get_ref<const std::string &>();
    const auto &command = operation->get_ref<const std::string &>();
    if (command == "admit_authority") {
        if (name == "authority_intent") {
            return TimedEvent::AdmissionIntent;
        }
        return name == "authority_admission" ? std::optional{TimedEvent::Admission} : std::nullopt;
    }
    if (command != "land") {
        return std::nullopt;
    }
    if (name == "mutation_intent") {
        return TimedEvent::Intent;
    }
    if (name == "mutation_outcome") {
        return TimedEvent::Outcome;
    }
    return name == "request_rejected" ? std::optional{TimedEvent::Rejected} : std::nullopt;
}

const char *AuditJournal::timed_event_name(TimedEvent event) {
    switch (event) {
    case TimedEvent::Intent: return "mutation_intent";
    case TimedEvent::Outcome: return "mutation_outcome";
    case TimedEvent::Rejected: return "request_rejected";
    case TimedEvent::AdmissionIntent: return "authority_intent";
    case TimedEvent::Admission: return "authority_admission";
    }
    return "unknown";
}

nlohmann::json AuditJournal::timing_json(bool admission) const {
    using Json = nlohmann::json;
    auto rows = Json::array();
    const auto &timings = admission ? admission_timings_ : timings_;
    const auto count = admission ? admission_timing_count_ : timing_count_;
    for (std::size_t index = 0; index < count; ++index) {
        const auto &item = timings[index];
        rows.push_back({{"journal_event", timed_event_name(item.event)}, {"ordinal", item.ordinal},
            {"lock_acquired_us", item.lock_acquired_us},
            {"before_native_us", item.before_native_us ? Json(*item.before_native_us) : Json(nullptr)},
            {"after_native_us", item.after_native_us ? Json(*item.after_native_us) : Json(nullptr)},
            {"append_finish_us", item.append_finish_us},
            {"native_success", item.native_success ? Json(*item.native_success) : Json(nullptr)},
            {"append_success", item.append_success}});
    }
    return {{"event", admission ? "runtime_admission_native_timing" : "runtime_land_native_timing"},
            {"capacity", timings.size()}, {"overflow", admission ? admission_timing_overflow_ : timing_overflow_},
            {"records", std::move(rows)}};
}

bool AuditJournal::validate_history(const std::string &directory) {
    for (const auto &entry : std::filesystem::directory_iterator(directory)) {
        if (entry.path().extension() != ".jsonl") {
            continue;
        }
        ProtectedFile previous;
        std::string content;
        // debt: 64 MiB per incarnation; revisit before reaching this ceiling; then stream validation/rotation.
        if (!previous.open(entry.path().string(), FileMode::Read) || !previous.read(content, 64 * 1024 * 1024)) {
            return false;
        }
        if (content.empty() || content.back() != '\n') {
            return false;
        }
        std::istringstream records(content);
        std::string line;
        while (std::getline(records, line)) {
            const auto record = nlohmann::json::parse(line, nullptr, false);
            if (!record.is_object() || record.value("schema", 0) != 1 ||
                !record.contains("event") || !record["event"].is_string() ||
                !record.contains("runtime_incarnation") || !record["runtime_incarnation"].is_string() ||
                !record.contains("ordinal") || !record["ordinal"].is_number_unsigned()) {
                return false;
            }
        }
    }
    return true;
}

bool AuditJournal::start(const std::string &directory, const std::string &incarnation) {
    std::lock_guard lock(mutex_);
    try {
        if (!prepare_audit_directory(directory) ||
            !lock_.open((std::filesystem::path(directory) / "runtime.lock").string(), FileMode::Lock) ||
            !validate_history(directory) ||
            !file_.open((std::filesystem::path(directory) / (incarnation + ".jsonl")).string(), FileMode::Create) ||
            !sync_directory(directory)) {
            fail();
            file_.close();
            lock_.close();
            return false;
        }
    } catch (...) {
        fail();
        file_.close();
        lock_.close();
        return false;
    }
    incarnation_ = incarnation;
    ordinal_ = 0;
    healthy_ = true;
    return true;
}

void AuditJournal::fail() {
    healthy_ = false;
    std::cerr << "audit_failure: durable audit unavailable; mutations inhibited until restart\n";
}

bool AuditJournal::append(nlohmann::json record) {
    const auto entered = timing_enabled_ ? TimingClock::now() : TimingClock::time_point{};
    std::lock_guard lock(mutex_);
    const auto acquired = timing_enabled_ ? TimingClock::now() : TimingClock::time_point{};
    if (!healthy_) {
        return false;
    }
    record["schema"] = 1;
    record["runtime_incarnation"] = incarnation_;
    record["ordinal"] = ++ordinal_;
    record["time_ms"] = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    std::optional<AppendTiming> timing;
    bool admission_timing = false;
    const auto elapsed_us = [&] {
        return std::chrono::duration_cast<std::chrono::microseconds>(TimingClock::now() - entered).count();
    };
    if (timing_enabled_) {
        if (const auto event = timed_event(record)) {
            timing = AppendTiming{};
            timing->event = *event;
            admission_timing = *event == TimedEvent::AdmissionIntent || *event == TimedEvent::Admission;
            timing->ordinal = ordinal_;
            timing->lock_acquired_us =
                std::chrono::duration_cast<std::chrono::microseconds>(acquired - entered).count();
        }
    }
    const auto finish_timing = [&](bool success) {
        if (!timing) {
            return;
        }
        timing->append_success = success;
        timing->append_finish_us = elapsed_us();
        auto &timings = admission_timing ? admission_timings_ : timings_;
        auto &count = admission_timing ? admission_timing_count_ : timing_count_;
        auto &overflow = admission_timing ? admission_timing_overflow_ : timing_overflow_;
        if (count < timings.size()) {
            timings[count++] = *timing;
        } else {
            overflow = true;
        }
    };
    try {
        const auto bytes = record.dump() + "\n";
        if (!guard_ || guard_(bytes)) {
            if (timing) {
                timing->before_native_us = elapsed_us();
            }
            const bool written = file_.append(bytes);
            if (timing) {
                timing->after_native_us = elapsed_us();
                timing->native_success = written;
            }
            if (written) {
                finish_timing(true);
                return true;
            }
        }
    } catch (...) {
        // A failed serialization/write cannot authorize a vehicle send.
    }
    fail();
    // The sink may have failed mid-record: do not append a fake durable failure marker to it.
    std::cerr << "{\"schema\":1,\"event\":\"audit_failure\",\"mutations_inhibited\":true}\n";
    finish_timing(false);
    return false;
}

bool AuditJournal::admit_send(const std::function<void()> &send) {
    std::lock_guard lock(mutex_);
    if (!healthy_) {
        return false;
    }
    send();
    return true;
}

bool AuditJournal::healthy() const {
    return healthy_;
}

void AuditJournal::stop() {
    std::lock_guard lock(mutex_);
    healthy_ = false;
    file_.close();
    lock_.close();
}

} // namespace nomad::runtime::detail
