// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "nomad/safety/geofence.hpp"

#include <string>

namespace nomad::safety {

// Parse the NOMAD-side keep-in fence from environment values. An unset
// polygon returns a policy without a boundary (the FC fence is the only
// enforcement); any malformed configured value returns an empty boundary,
// which rejects every position target — a broken fence must fail closed.
GlobalFencePolicy load_fence_policy(const char *polygon_env, const char *margin_env);

}  // namespace nomad::safety
