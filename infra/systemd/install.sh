#!/bin/bash
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
    exit 1
fi
# shellcheck disable=SC1090
. "$ENV_FILE"

UNITS=(
    nomad.target
    nomad-edge-core.service
    nomad-mavlink-router.service
    nomad-mediamtx.service
    nomad-isaac-ros-container.service
    nomad-zed-wrapper.service
    nomad-ros-http-bridge.service
    nomad-video-bridge.service
    nomad-nvblox.service
)

echo "[install] copying units to $DEST"
for u in "${UNITS[@]}"; do
    install -m 0644 "$THIS_DIR/$u" "$DEST/$u"
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
    [nomad-ros-http-bridge.service]="$NOMAD_AUTOSTART_ROS_HTTP_BRIDGE"
    [nomad-video-bridge.service]="$NOMAD_AUTOSTART_VIDEO_BRIDGE"
    [nomad-nvblox.service]="$NOMAD_AUTOSTART_NVBLOX"
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

To enable nvblox on this host:
    sudo systemctl enable --now nomad-nvblox.service
EOF
