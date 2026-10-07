#!/bin/sh
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
set -eu

if [ -z "${SITL_PROFILE_DEFAULTS:-}" ] || [ ! -s "${SITL_PROFILE_DEFAULTS}" ]; then
    echo "SITL_PROFILE_DEFAULTS must point to a non-empty parameter file" >&2
    exit 1
fi

set -- /ardupilot/Tools/autotest/sim_vehicle.py \
    --vehicle ArduPlane \
    -I"${INSTANCE:-0}" \
    --custom-location="${LAT},${LON},${ALT},${DIR}" \
    -w \
    --frame quadplane-tilttri \
    --no-rebuild \
    --speedup "${SPEEDUP}" \
    --add-param-file="${SITL_PROFILE_DEFAULTS}" \
    --out "${SITL_UDP_OUTPUT_ADDRESS}"

if [ -n "${SITL_UDP_OBSERVER_ADDRESS:-}" ]; then
    set -- "$@" --out "${SITL_UDP_OBSERVER_ADDRESS}"
fi

exec "$@"
