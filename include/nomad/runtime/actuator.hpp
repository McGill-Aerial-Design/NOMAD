// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <string>

namespace nomad::runtime {

enum class ActuatorBehavior { ServoToggle, ServoPosition, ServoBidirectional, RelayToggle, RelayPulse };

struct ActuatorDefinition {
    std::string id;
    std::string name;
    ActuatorBehavior behavior{ActuatorBehavior::ServoToggle};
    int channel{9};
    int pwm_min{1000};
    int pwm_max{2000};
    int pwm_neutral{1500};
    int safe_pwm{1000};
    bool reversed{false};
    int pulse_ms{500};
    int hold_ms{1000};
    std::string primary_label{"Position B"};
    std::string secondary_label{"Position A"};
    std::string negative_label{"Negative"};
    std::string neutral_label{"Stop"};
    std::string positive_label{"Positive"};
    bool hazardous{true};
    int confirmation_count{3};
    int confirmation_window_ms{3000};
    bool require_neutral{true};
};

} // namespace nomad::runtime
