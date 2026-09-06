#!/bin/bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
# =============================================================================
# Install NOMAD systemd units.
#
# Reads NOMAD_AUTOSTART_* flags from config/nomad.env. Only enables the units
# whose flag is true. nvblox is intentionally NOT enabled by default.
#
# Run on the Jetson with sudo:
#   sudo bash infra/systemd/install.sh
#
# Re-running is safe — units are re-copied and the enabled set is reconciled
# against current nomad.env.
# =============================================================================
set -euo pipefail

if [ "$EUID" -ne 0 ]; then
    echo "[install] re-running with sudo..."
    exec sudo -E bash "$0" "$@"
fi

THIS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$THIS_DIR/../.." && pwd)"
ENV_FILE="$REPO_ROOT/config/nomad.env"
DEST=/etc/systemd/system

if [ ! -f "$ENV_FILE" ]; then
    echo "[install] config not found: $ENV_FILE" >&2
    echo "[install] copy config/nomad.env.example to config/nomad.env and edit it first." >&2
    exit 1
fi

# Service identity used to render the unit templates (__NOMAD_USER__/__NOMAD_ROOT__/
# __NOMAD_HOME__). Defaults to the user that owns the checkout; override via the
# NOMAD_USER env var.
NOMAD_ROOT="$REPO_ROOT"
NOMAD_USER="${NOMAD_USER:-${SUDO_USER:-$(stat -c '%U' "$REPO_ROOT")}}"
NOMAD_HOME="$(getent passwd "$NOMAD_USER" | cut -d: -f6)"
[ -n "$NOMAD_HOME" ] || NOMAD_HOME="$(dirname "$REPO_ROOT")"
echo "[install] rendering units for user=$NOMAD_USER root=$NOMAD_ROOT home=$NOMAD_HOME"

echo "[install] repairing script executable permissions"
find "$REPO_ROOT/scripts" "$REPO_ROOT/infra/tailscale/scripts" \
    -type f \( -name '*.sh' -o -name nomad \) -exec chmod 0755 {} +
chmod 0755 "$REPO_ROOT/scripts/nomad" \
    "$REPO_ROOT/infra/systemd/install.sh" 2>/dev/null || true

# shellcheck disable=SC1090
. "$ENV_FILE"

UNITS=(
    nomad.target
    nomad-edge-core.service
    nomad-mavlink-router.service
    nomad-mediamtx.service
    nomad-isaac-ros-container.service
    nomad-zed-wrapper.service
    nomad-ros-vehicle.service
    nomad-video-bridge.service
    nomad-nvblox.service
)

# -----------------------------------------------------------------------------
# Install sudoers fragment so the service user can drive nomad-*.service units
# without a password prompt. This is what allows Edge Core's COMMAND_WHITELIST
# (status_/start_/stop_/restart_*) and Mission Planner's terminal panel to
# manage services via `sudo -n systemctl ...` without interactive auth.
#
# Scope is intentionally tight: only the nomad-* systemctl verbs, plus
# reboot/shutdown which Mission Planner already supports.
# -----------------------------------------------------------------------------
SUDOERS_DST=/etc/sudoers.d/nomad
SUDOERS_CONTENT=$(cat <<'EOF'
# Managed by infra/systemd/install.sh — do not edit by hand.
# Grants the __NOMAD_USER__ user passwordless control of NOMAD systemd units.
Cmnd_Alias NOMAD_SYSTEMCTL = \
    /bin/systemctl start nomad-*, \
    /bin/systemctl stop nomad-*, \
    /bin/systemctl restart nomad-*, \
    /bin/systemctl reload nomad-*, \
    /bin/systemctl start nomad.target, \
    /bin/systemctl stop nomad.target, \
    /bin/systemctl restart nomad.target, \
    /bin/systemctl enable nomad-*, \
    /bin/systemctl disable nomad-*, \
    /bin/systemctl daemon-reload
Cmnd_Alias NOMAD_POWER = /sbin/reboot, /sbin/shutdown
__NOMAD_USER__ ALL=(root) NOPASSWD: NOMAD_SYSTEMCTL, NOMAD_POWER
EOF
)
SUDOERS_TMP=$(mktemp)
printf '%s\n' "$SUDOERS_CONTENT" > "$SUDOERS_TMP"
sed -i "s/__NOMAD_USER__/$NOMAD_USER/g" "$SUDOERS_TMP"
chmod 0440 "$SUDOERS_TMP"
if visudo -cf "$SUDOERS_TMP" >/dev/null 2>&1; then
    install -m 0440 -o root -g root "$SUDOERS_TMP" "$SUDOERS_DST"
    echo "[install] installed $SUDOERS_DST"
else
    echo "[install] ERROR: sudoers fragment failed visudo validation; not installing" >&2
    visudo -cf "$SUDOERS_TMP" || true
    rm -f "$SUDOERS_TMP"
    exit 1
fi
rm -f "$SUDOERS_TMP"

# Log rotation for ~/nomad_logs (idempotent, like the units below).
install -m 0644 "$REPO_ROOT/infra/logrotate.conf" /etc/logrotate.d/nomad
echo "[install] installed /etc/logrotate.d/nomad"

echo "[install] rendering + installing units to $DEST"
for u in "${UNITS[@]}"; do
    sed -e "s#__NOMAD_ROOT__#$NOMAD_ROOT#g" \
        -e "s#__NOMAD_HOME__#$NOMAD_HOME#g" \
        -e "s#__NOMAD_USER__#$NOMAD_USER#g" \
        "$THIS_DIR/$u" > "$DEST/$u"
    chmod 0644 "$DEST/$u"
done

echo "[install] systemctl daemon-reload"
systemctl daemon-reload

# Always enable nomad.target so boot pulls in WantedBy=nomad.target units.
systemctl enable nomad.target

declare -A FLAG=(
    [nomad-edge-core.service]="$NOMAD_AUTOSTART_EDGE_CORE"
    [nomad-mavlink-router.service]="$NOMAD_AUTOSTART_MAVLINK_ROUTER"
    [nomad-mediamtx.service]="$NOMAD_AUTOSTART_MEDIAMTX"
    [nomad-isaac-ros-container.service]="$NOMAD_AUTOSTART_ISAAC_ROS_CONTAINER"
    [nomad-zed-wrapper.service]="$NOMAD_AUTOSTART_ZED_WRAPPER"
    [nomad-ros-vehicle.service]="$NOMAD_AUTOSTART_ROS_VEHICLE"
    [nomad-video-bridge.service]="$NOMAD_AUTOSTART_VIDEO_BRIDGE"
)

for u in "${!FLAG[@]}"; do
    if [ "${FLAG[$u]}" = "true" ]; then
        echo "[install] enable  $u"
        systemctl enable "$u"
    else
        echo "[install] disable $u (NOMAD_AUTOSTART_*=false)"
        systemctl disable "$u" 2>/dev/null || true
    fi
done

echo "[install] disable nomad-nvblox.service (manual-only; start from Mission Planner Service Control)"
systemctl disable nomad-nvblox.service 2>/dev/null || true

# Remove the legacy single-unit setup if it lingers.
if systemctl list-unit-files nomad.service >/dev/null 2>&1; then
    echo "[install] disabling legacy nomad.service (replaced by per-service units)"
    systemctl disable --now nomad.service 2>/dev/null || true
fi

cat <<EOF

[install] Done. Useful commands:
    systemctl status nomad.target
    nomad start all          # start the autostart set
    nomad status             # check each service
    journalctl -u nomad-edge-core -f

To run nvblox on this host:
    Start it from Mission Planner Service Control, or run:
    sudo systemctl start nomad-nvblox.service
EOF
