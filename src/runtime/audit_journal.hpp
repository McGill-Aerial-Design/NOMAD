// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "protected_file.hpp"

#include <nlohmann/json.hpp>

#include <atomic>
#include <array>
#include <chrono>
#include <functional>
#include <mutex>
#include <optional>
#include <string>

namespace nomad::runtime::detail {

class AuditJournal {
  public:
    explicit AuditJournal(std::function<bool(const std::string &)> guard = {});
    ~AuditJournal() noexcept;
    bool start(const std::string &directory, const std::string &incarnation);
    bool append(nlohmann::json record);
    bool admit_send(const std::function<void()> &send);
    bool healthy() const;
    void stop();

  private:
    friend struct AuditJournalTestAccess;
    using TimingClock = std::chrono::steady_clock;
    enum class LandEvent { Intent, Outcome, Rejected };
    struct AppendTiming {
        LandEvent event{};
        std::uint64_t ordinal{};
        std::int64_t lock_acquired_us{};
        std::optional<std::int64_t> before_native_us;
        std::optional<std::int64_t> after_native_us;
        std::int64_t append_finish_us{};
        std::optional<bool> native_success;
        bool append_success{};
    };
    static std::optional<LandEvent> land_event(const nlohmann::json &record);
    nlohmann::json timing_json() const;
    bool validate_history(const std::string &directory);
    void fail();
    mutable std::mutex mutex_;
    ProtectedFile lock_;
    ProtectedFile file_;
    std::atomic_bool healthy_{false};
    std::string incarnation_;
    std::uint64_t ordinal_{};
    std::function<bool(const std::string &)> guard_;
    bool timing_enabled_{false};
    std::array<AppendTiming, 8> timings_{};
    std::size_t timing_count_{};
    bool timing_overflow_{false};
};

} // namespace nomad::runtime::detail
