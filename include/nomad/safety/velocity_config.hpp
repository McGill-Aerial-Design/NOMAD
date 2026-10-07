// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "nomad/safety/velocity.hpp"

namespace nomad::safety {

// Return the reviewed development limits used when no override is configured.
VelocityLimits reviewed_velocity_limits();

// Validate and construct limits supplied by a typed configuration source.
VelocityLimits load_velocity_limits(float max_velocity_xy, float max_velocity_z, float max_yaw_rate);

// Load velocity limits from environment values. An unset or empty value uses
// the reviewed default; any malformed configured value makes every limit
// invalid so evaluate_velocity rejects commands fail closed.
VelocityLimits load_velocity_limits(const char *xy_env, const char *z_env, const char *yaw_rate_env);

}  // namespace nomad::safety
