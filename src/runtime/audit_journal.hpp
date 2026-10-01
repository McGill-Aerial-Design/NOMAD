// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "protected_file.hpp"

#include <nlohmann/json.hpp>

#include <atomic>
#include <functional>
#include <mutex>
#include <string>

namespace nomad::runtime::detail {

class AuditJournal {
  public:
    explicit AuditJournal(std::function<bool(const std::string &)> guard = {}) : guard_(std::move(guard)) {}
    bool start(const std::string &directory, const std::string &incarnation);
    bool append(nlohmann::json record);
    bool admit_send(const std::function<void()> &send);
    bool healthy() const;
    void stop();

  private:
    bool validate_history(const std::string &directory);
    void fail();
    mutable std::mutex mutex_;
    ProtectedFile lock_;
    ProtectedFile file_;
    std::atomic_bool healthy_{false};
    std::string incarnation_;
    std::uint64_t ordinal_{};
    std::function<bool(const std::string &)> guard_;
};

} // namespace nomad::runtime::detail
