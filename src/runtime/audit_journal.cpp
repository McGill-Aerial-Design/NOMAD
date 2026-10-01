// SPDX-License-Identifier: Apache-2.0
#include "audit_journal.hpp"

#include <chrono>
#include <filesystem>
#include <iostream>
#include <sstream>

namespace nomad::runtime::detail {

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
    std::lock_guard lock(mutex_);
    if (!healthy_) {
        return false;
    }
    record["schema"] = 1;
    record["runtime_incarnation"] = incarnation_;
    record["ordinal"] = ++ordinal_;
    record["time_ms"] = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    try {
        const auto bytes = record.dump() + "\n";
        if ((!guard_ || guard_(bytes)) && file_.append(bytes)) {
            return true;
        }
    } catch (...) {
        // A failed serialization/write cannot authorize a vehicle send.
    }
    fail();
    // The sink may have failed mid-record: do not append a fake durable failure marker to it.
    std::cerr << "{\"schema\":1,\"event\":\"audit_failure\",\"mutations_inhibited\":true}\n";
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
