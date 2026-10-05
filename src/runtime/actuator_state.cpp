// SPDX-License-Identifier: Apache-2.0
#include "actuator_state.hpp"

#include <algorithm>
#include <cmath>

namespace nomad::runtime::detail {
namespace {

Json actions(const ActuatorDefinition &p) {
    auto result = Json::array();
    const auto add = [&](const std::string &operation, const std::string &label, const char *control = "button") {
        const auto release = operation == "positive" || operation == "negative" ? "release_input" : "";
        result.push_back({{"operation", operation}, {"label", label}, {"control", control},
                          {"release_operation", release}});
    };
    if (p.behavior == ActuatorBehavior::ServoPosition) {
        add("position", p.primary_label, "position");
    } else if (p.behavior == ActuatorBehavior::ServoBidirectional) {
        add("negative", p.negative_label);
        add("positive", p.positive_label);
    } else {
        add("activate", p.primary_label);
        if (p.behavior == ActuatorBehavior::ServoToggle || p.behavior == ActuatorBehavior::RelayToggle) {
            add("toggle", p.primary_label + " / " + p.secondary_label);
        }
    }
    add(p.behavior == ActuatorBehavior::ServoBidirectional ? "stop" : "safe",
        p.behavior == ActuatorBehavior::ServoBidirectional ? p.neutral_label : p.secondary_label);
    return result;
}

bool matches_operation(const ActuatorDefinition &p, const ActuatorInput &input) {
    if ((input.source != "ui" && input.source != "hid") ||
        (input.source == "hid" && (input.slot < 0 || input.slot > 31)) ||
        (input.source == "ui" && input.slot != -1)) {
        return false;
    }
    if (input.operation == "neutral") {
        return input.source == "hid" && !input.value;
    }
    if (input.operation == "release_input") {
        return input.source == "hid" && !input.value && p.behavior == ActuatorBehavior::ServoBidirectional;
    }
    if (input.operation == "safe" || input.operation == "stop") {
        return !input.value.has_value();
    }
    if (p.behavior == ActuatorBehavior::ServoPosition) {
        return input.operation == "position" && input.value && std::isfinite(*input.value) &&
               *input.value >= 0 && *input.value <= 1;
    }
    if (input.value) {
        return false;
    }
    if (p.behavior == ActuatorBehavior::ServoBidirectional) {
        return input.operation == "positive" || input.operation == "negative";
    }
    return input.operation == "activate" ||
           (input.operation == "toggle" && p.behavior != ActuatorBehavior::RelayPulse);
}

int target_pwm(const ActuatorDefinition &p, const ActuatorInput &input) {
    if (input.operation == "position") {
        return p.pwm_min + static_cast<int>(std::lround(*input.value * (p.pwm_max - p.pwm_min)));
    }
    if (input.operation == "negative") {
        return p.pwm_min;
    }
    return p.reversed && p.behavior == ActuatorBehavior::ServoToggle ? p.pwm_min : p.pwm_max;
}

} // namespace

ActuatorState::ActuatorState(std::vector<ActuatorDefinition> definitions) {
    replace(std::move(definitions));
}

void ActuatorState::replace(std::vector<ActuatorDefinition> definitions) {
    std::lock_guard lock(mutex_);
    definitions_ = std::move(definitions);
    states_.clear();
    for (const auto &p : definitions_) {
        states_[p.id].revision = ++revision_;
    }
}

bool ActuatorState::can_configure() const {
    std::lock_guard lock(mutex_);
    return std::none_of(states_.begin(), states_.end(), [](const auto &entry) {
        return entry.second.pending || entry.second.active || entry.second.recovery || entry.second.stop_requested;
    });
}

bool ActuatorState::contains_output(int channel, bool relay) const {
    std::lock_guard lock(mutex_);
    return std::any_of(definitions_.begin(), definitions_.end(), [&](const auto &p) {
        return p.channel == channel && actuator_is_relay(p) == relay;
    });
}

std::vector<ActuatorDefinition> ActuatorState::definitions() const {
    std::lock_guard lock(mutex_);
    return definitions_;
}

Json ActuatorState::state_locked(const ActuatorDefinition &p, const State &s) const {
    return {{"id", p.id}, {"state_revision", s.revision}, {"recovery_required", s.recovery || s.stop_requested},
            {"pending", s.pending}, {"activation_commanded", s.active},
            {"confirmation_remaining", p.confirmation_count - s.confirmations},
            {"commanded_position", s.position ? Json(*s.position) : Json(nullptr)},
            {"software_command_success", s.software_success}, {"pulse_on_succeeded", s.pulse_on_succeeded},
            {"activation_outcome", s.activation_outcome}, {"safe_outcome", s.safe_outcome}};
}

Json ActuatorState::state(const std::string &id) const {
    std::lock_guard lock(mutex_);
    for (const auto &p : definitions_) {
        if (p.id == id) {
            return state_locked(p, states_.at(id));
        }
    }
    return Json(nullptr);
}

Json ActuatorState::discover(const std::string &authority, std::chrono::steady_clock::time_point now) {
    std::lock_guard lock(mutex_);
    auto result = Json::array();
    const auto configs = serialize_actuator_definitions(definitions_);
    for (std::size_t index = 0; index < definitions_.size(); ++index) {
        const auto &p = definitions_[index];
        auto &s = states_.at(p.id);
        if (!s.authority.empty() && s.authority != authority) {
            reset_confirmation(s);
            s.recovery = true;
            s.authority.clear();
            s.input_sequence = 0;
            s.neutral = false;
        }
        if (s.confirmations > 0 && (now < s.first_confirmation || now - s.first_confirmation >
            std::chrono::milliseconds(p.confirmation_window_ms))) {
            reset_confirmation(s);
        }
        result.push_back({{"id", p.id}, {"name", p.name}, {"behavior", actuator_behavior_name(p.behavior)},
                          {"actions", actions(p)}, {"state", state_locked(p, s)}, {"config", configs[index]}});
    }
    return result;
}

void ActuatorState::reset_confirmation(State &s) {
    s.confirmations = 0;
    s.operation.clear();
    s.value.reset();
    s.revision = ++revision_;
}

bool ActuatorState::confirm_locked(const ActuatorInput &input, const ActuatorDefinition &p, State &s,
                                   std::chrono::steady_clock::time_point now) {
    if (s.operation != input.operation || s.value != input.value ||
        (s.confirmations > 0 && (now < s.first_confirmation ||
         now - s.first_confirmation > std::chrono::milliseconds(p.confirmation_window_ms)))) {
        reset_confirmation(s);
    }
    if (input.source == "hid" && p.require_neutral && !s.neutral) {
        return false;
    }
    if (s.confirmations == 0) {
        s.first_confirmation = now;
    }
    s.operation = input.operation;
    s.value = input.value;
    s.neutral = false;
    ++s.confirmations;
    s.revision = ++revision_;
    if (s.confirmations < p.confirmation_count) {
        return false;
    }
    reset_confirmation(s);
    return true;
}

ActuatorDecision ActuatorState::begin(const ActuatorInput &input, std::chrono::steady_clock::time_point now) {
    std::lock_guard lock(mutex_);
    const auto found = std::find_if(definitions_.begin(), definitions_.end(), [&](const auto &p) {
        return p.id == input.id;
    });
    ActuatorDecision decision;
    if (found == definitions_.end() || input.operation == "release_input" || !matches_operation(*found, input)) {
        decision.message = "configured actuator/action is unavailable or value is invalid";
        return decision;
    }
    return begin_locked(input, *found, states_.at(input.id), now);
}

void ActuatorState::set_input_context(const ActuatorInput &input, State &s) {
    if (s.authority != input.authority) {
        s.input_sequence = 0;
    }
    if (!s.authority.empty() && s.authority != input.authority) {
        s.recovery = true;
    }
    if (s.authority != input.authority || s.input_source != input.source || s.slot != input.slot) {
        reset_confirmation(s);
        s.authority = input.authority;
        s.input_source = input.source;
        s.slot = input.slot;
        s.neutral = false;
    }
}

ActuatorDecision ActuatorState::begin_locked(const ActuatorInput &input, const ActuatorDefinition &p, State &s,
                                            std::chrono::steady_clock::time_point now) {
    ActuatorDecision decision;
    decision.definition = p;
    set_input_context(input, s);
    if (input.sequence > 0 && input.sequence < s.input_sequence && input.operation != "safe" &&
        input.operation != "stop") {
        decision.message = "newer actuator input was already observed";
        return decision;
    }
    s.input_sequence = std::max(s.input_sequence, input.sequence);
    if (input.operation == "neutral" && input.source == "hid") {
        s.neutral = true;
        s.revision = ++revision_;
        decision.allowed = true;
        decision.message = "physical input returned to neutral; no vehicle command sent";
        return decision;
    }
    decision.operation = input.operation == "toggle" ? (s.active || s.recovery ? "safe" : "activate") : input.operation;
    decision.safe = decision.operation == "safe" || decision.operation == "stop";
    if (s.pending || (!decision.safe && (s.recovery || s.stop_requested))) {
        decision.message = "actuator is pending or requires an explicit successful safe command";
        return decision;
    }
    if (!decision.safe) {
        auto resolved = input;
        resolved.operation = decision.operation;
        if (p.confirmation_count > 0 && !confirm_locked(resolved, p, s, now)) {
            decision.allowed = true;
            decision.message = input.source == "hid" && p.require_neutral && !s.neutral &&
                s.confirmations == 0 ? "physical input neutral return required; no vehicle command sent" :
                                      "confirmation sequence incomplete; no vehicle command sent";
            return decision;
        }
    } else {
        reset_confirmation(s);
    }
    return start_output_locked(input, std::move(decision), s);
}

ActuatorDecision ActuatorState::release_input(const ActuatorInput &input) {
    std::lock_guard lock(mutex_);
    const auto found = std::find_if(definitions_.begin(), definitions_.end(), [&](const auto &p) {
        return p.id == input.id;
    });
    ActuatorDecision decision;
    if (found == definitions_.end() || input.operation != "release_input" || !matches_operation(*found, input)) {
        decision.message = "physical release requires a configured bidirectional HID action";
        return decision;
    }
    auto &s = states_.at(input.id);
    set_input_context(input, s);
    if (input.sequence > 0 && input.sequence < s.input_sequence) {
        decision.message = "newer actuator input was already observed";
        return decision;
    }
    s.input_sequence = std::max(s.input_sequence, input.sequence);
    s.neutral = false;
    s.revision = ++revision_;
    decision.allowed = true;
    decision.execute = s.pending || s.active || s.recovery || s.stop_requested;
    decision.safe = decision.execute;
    decision.message = "physical input released; confirmation sequence preserved; no vehicle command sent";
    return decision;
}

ActuatorDecision ActuatorState::start_output_locked(const ActuatorInput &input, ActuatorDecision decision, State &s) {
    const auto &p = decision.definition;
    decision.allowed = true;
    decision.execute = true;
    decision.target_pwm = decision.safe ? p.safe_pwm : target_pwm(p, input);
    if (!decision.safe && p.behavior == ActuatorBehavior::RelayPulse) {
        decision.duration_ms = p.pulse_ms;
    } else if (!decision.safe && p.behavior == ActuatorBehavior::ServoBidirectional) {
        decision.duration_ms = p.hold_ms;
    }
    s.pending = true;
    s.revision = ++revision_;
    return decision;
}

void ActuatorState::finish(const ActuatorDecision &d, const std::string &activation,
                           const std::string &safe, const std::string &outcome, bool started,
                           std::optional<bool> observed_command_success) {
    std::lock_guard lock(mutex_);
    auto &s = states_.at(d.definition.id);
    const bool uncertain = outcome == "unknown" || outcome == "interrupted";
    if (uncertain || (started && d.duration_ms > 0 && safe != "success")) {
        s.recovery = true;
    } else if ((d.safe || d.duration_ms > 0) && outcome == "success") {
        s.recovery = false;
    }
    if (d.safe && outcome == "success") {
        s.stop_requested = false;
    }
    s.pending = false;
    s.software_success = observed_command_success.value_or(outcome == "success");
    s.pulse_on_succeeded = started && d.definition.behavior == ActuatorBehavior::RelayPulse;
    s.activation_outcome = activation;
    s.safe_outcome = safe;
    if (started) {
        s.active = true;
    }
    if ((d.safe || d.duration_ms > 0) && outcome == "success") {
        s.active = false;
    }
    if (outcome == "success" && !actuator_is_relay(d.definition)) {
        const auto pwm = d.safe || d.duration_ms > 0 ? d.definition.safe_pwm : d.target_pwm;
        s.position = static_cast<double>(pwm - d.definition.pwm_min) / (d.definition.pwm_max - d.definition.pwm_min);
    }
    s.revision = ++revision_;
}

void ActuatorState::require_recovery(const std::string &id) {
    std::lock_guard lock(mutex_);
    auto &s = states_.at(id);
    s.recovery = true;
    reset_confirmation(s);
}

void ActuatorState::signal_safe(const std::string &id) {
    std::lock_guard lock(mutex_);
    const auto found = states_.find(id);
    if (found != states_.end()) {
        found->second.stop_requested = true;
        reset_confirmation(found->second);
        wake_.notify_all();
    }
}

void ActuatorState::wait(const std::string &id, std::chrono::milliseconds duration) {
    std::unique_lock lock(mutex_);
    wake_.wait_for(lock, duration, [&] { return states_.at(id).stop_requested; });
}

} // namespace nomad::runtime::detail
