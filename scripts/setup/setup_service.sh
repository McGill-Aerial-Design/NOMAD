#!/bin/bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
# =============================================================================
# NOMAD systemd setup — installs the per-service units.
# Must be run with sudo: sudo bash scripts/setup/setup_service.sh
#
# This is a thin wrapper around infra/systemd/install.sh, kept for backward
# compatibility with existing setup runbooks. New runbooks should call the
# install script directly.
# =============================================================================
set -euo pipefail

if [ "$EUID" -ne 0 ]; then
    echo "Please run with sudo: sudo bash $0"
    exit 1
fi

# Resolve the repo root from this script's location (scripts/setup/ -> repo root).
NOMAD_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$NOMAD_DIR"

NOMAD_ENV="$NOMAD_DIR/config/nomad.env"
if [ ! -f "$NOMAD_ENV" ]; then
    echo "[setup] config not found: $NOMAD_ENV" >&2
    echo "[setup] copy config/nomad.env.example and load a product profile first." >&2
    exit 1
fi

# Keep profile selection in the canonical manager. `validate` is read-only and
# avoids changing config/nomad.env while setup is being reconciled.
if ! python3 "$NOMAD_DIR/scripts/profile.py" validate; then
    echo "[setup] active product profile validation failed" >&2
    exit 1
fi

# shellcheck disable=SC1090
. "$NOMAD_ENV"
case "${NOMAD_PROFILE:-}" in
    onboard_companion|groundstation_gpu|groundstation_minimal) ;;
    "")
        echo "[setup] NOMAD_PROFILE is missing; load a supported product profile first." >&2
        exit 1
        ;;
    *)
        echo "[setup] unsupported NOMAD_PROFILE: $NOMAD_PROFILE" >&2
        exit 1
        ;;
esac

# Optional compute, media, and ROS workloads remain off until their providers
# and runtime readiness have been qualified for the selected product profile.
OPTIONAL_FLAGS=(
    NOMAD_AUTOSTART_MEDIAMTX
    NOMAD_AUTOSTART_ISAAC_ROS_CONTAINER
    NOMAD_AUTOSTART_ROS_VEHICLE
    NOMAD_AUTOSTART_VIDEO_BRIDGE
)
for flag in "${OPTIONAL_FLAGS[@]}"; do
    case "${!flag:-}" in
        false|0) ;;
        true|1)
            echo "[setup] Unavailable: $flag is enabled before optional-provider qualification." >&2
            echo "[setup] Set $flag=false or load a supported profile with it disabled." >&2
            exit 1
            ;;
        *)
            echo "[setup] $flag must be explicitly false until its provider is qualified." >&2
            exit 1
            ;;
    esac
done

# Install / reconcile per-service systemd units.
bash "$NOMAD_DIR/infra/systemd/install.sh"

echo
echo "Done. Bring everything up with:"
echo "  sudo systemctl start nomad.target"
echo "  nomad status"
