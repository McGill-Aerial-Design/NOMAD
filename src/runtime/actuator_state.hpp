// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "actuator_config.hpp"

#include <chrono>
#include <condition_variable>
#include <map>
#include <mutex>
#include <optional>

namespace nomad::runtime::detail {

struct ActuatorInput {
    std::string id;
    std::string operation;
    std::string source;
    int slot{-1};
    std::optional<double> value;
    std::string authority;
    std::uint64_t sequence{};
};

struct ActuatorDecision {
    bool allowed{false};
    bool execute{false};
    bool safe{false};
    int target_pwm{};
    int duration_ms{};
    std::string operation;
    std::string message;
    ActuatorDefinition definition;
};

class ActuatorState {
  public:
    explicit ActuatorState(std::vector<ActuatorDefinition> definitions = {});
    void replace(std::vector<ActuatorDefinition> definitions);
    bool can_configure() const;
    bool contains_output(int channel, bool relay) const;
    std::vector<ActuatorDefinition> definitions() const;
    Json discover(const std::string &authority,
                  std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now(),
                  std::uint64_t *configuration_revision = nullptr);
    Json state(const std::string &id) const;
    ActuatorDecision begin(const ActuatorInput &input, std::chrono::steady_clock::time_point now);
    ActuatorDecision release_input(const ActuatorInput &input);
    void finish(const ActuatorDecision &decision, const std::string &activation_outcome,
                const std::string &safe_outcome, const std::string &outcome, bool activation_success,
                std::optional<bool> observed_command_success = {});
    void require_recovery(const std::string &id);
    void signal_safe(const std::string &id);
    void wait(const std::string &id, std::chrono::milliseconds duration);

  private:
    struct State {
        bool recovery{true};
        bool pending{false};
        bool active{false};
        bool neutral{false};
        bool stop_requested{false};
        bool software_success{false};
        bool pulse_on_succeeded{false};
        int confirmations{};
        int slot{-1};
        std::optional<double> value;
        std::optional<double> position;
        std::string authority;
        std::string input_source;
        std::string operation;
        std::string activation_outcome;
        std::string safe_outcome;
        std::chrono::steady_clock::time_point first_confirmation{};
        std::uint64_t revision{};
        std::uint64_t input_sequence{};
    };
    Json state_locked(const ActuatorDefinition &definition, const State &state) const;
    bool confirm_locked(const ActuatorInput &input, const ActuatorDefinition &p, State &state,
                        std::chrono::steady_clock::time_point now);
    void reset_confirmation(State &state);
    void set_input_context(const ActuatorInput &input, State &state);
    ActuatorDecision begin_locked(const ActuatorInput &input, const ActuatorDefinition &definition, State &state,
                                  std::chrono::steady_clock::time_point now);
    ActuatorDecision start_output_locked(const ActuatorInput &input, ActuatorDecision decision, State &state);
    mutable std::mutex mutex_;
    std::condition_variable wake_;
    std::vector<ActuatorDefinition> definitions_;
    std::map<std::string, State> states_;
    std::uint64_t revision_{};
    std::uint64_t configuration_revision_{};
};

} // namespace nomad::runtime::detail
