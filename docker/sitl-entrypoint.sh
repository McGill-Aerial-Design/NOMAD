#!/bin/sh
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
set -eu

if [ -z "${SITL_FENCE_DEFAULTS:-}" ] || [ ! -s "${SITL_FENCE_DEFAULTS}" ]; then
    echo "SITL_FENCE_DEFAULTS must point to a non-empty parameter file" >&2
    exit 1
fi

if [ -z "${SITL_STREAM_DEFAULTS:-}" ] || [ ! -s "${SITL_STREAM_DEFAULTS}" ]; then
    echo "SITL_STREAM_DEFAULTS must point to a non-empty parameter file" >&2
    exit 1
fi

# Apply the fence parameters (FENCE_ENABLE/ACTION/TYPE) so the SITL vehicle
# starts with an enabled, writable polygon fence for the acceptance scenarios,
# and the SERIAL0 stream rates the core consumes. ArduPilot only streams
# position, attitude and extended status once a GCS requests the group, and the
# core deliberately does not request streams, so the vehicle must emit them.
exec /ardupilot/Tools/autotest/sim_vehicle.py \
    --vehicle "${VEHICLE}" \
    -I"${INSTANCE:-0}" \
    --custom-location="${LAT},${LON},${ALT},${DIR}" \
    -w \
    --frame "${MODEL}" \
    --no-rebuild \
    --speedup "${SPEEDUP}" \
    --add-param-file="${SITL_FENCE_DEFAULTS}" \
    --add-param-file="${SITL_STREAM_DEFAULTS}" \
    --out ${SITL_UDP_OUTPUT_ADDRESS}
