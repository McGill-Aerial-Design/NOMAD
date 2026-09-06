#!/bin/sh
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
set -eu

if [ -z "${SITL_FENCE_DEFAULTS:-}" ] || [ ! -s "${SITL_FENCE_DEFAULTS}" ]; then
    echo "SITL_FENCE_DEFAULTS must point to a non-empty parameter file" >&2
    exit 1
fi

# Apply the fence parameters (FENCE_ENABLE/ACTION/TYPE) so the SITL vehicle
# starts with an enabled, writable polygon fence for the acceptance scenarios.
exec /ardupilot/Tools/autotest/sim_vehicle.py \
    --vehicle "${VEHICLE}" \
    -I"${INSTANCE:-0}" \
    --custom-location="${LAT},${LON},${ALT},${DIR}" \
    -w \
    --frame "${MODEL}" \
    --no-rebuild \
    --speedup "${SPEEDUP}" \
    --add-param-file="${SITL_FENCE_DEFAULTS}" \
    --out ${SITL_UDP_OUTPUT_ADDRESS}
