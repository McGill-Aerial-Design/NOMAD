#!/bin/bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
# =============================================================================
# nomad-nvblox service (opt-in: NOT in the default autostart set)
#
# Owns: nvblox launch inside the Isaac ROS container (component_container_mt
# instance plus the helper nodes started by nomad_zed_nvblox.launch.py).
#
# Depends on: isaac_ros_container, zed_wrapper. nvblox reads from the live
# ZED topics, so refuse to start if ZED is not publishing.
# =============================================================================
set -u
SERVICE="nvblox"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=../lib/common.sh
. "$SCRIPT_DIR/../lib/common.sh"

# Anchored patterns: nvblox component container + nomad launch. Do NOT match
# the bare `component_container` because that's also used by ZED.
PATTERN='nomad_zed_nvblox\.launch\.py|nvblox_node|nvblox_examples_bringup'

LAUNCH_SCRIPT_PATH=/tmp/nomad_nvblox_launch.sh
LAUNCH_LOG=/tmp/nomad_nvblox.log

to_container_path() {
    printf '%s' "$1" | sed "s#^${NOMAD_REPO_ROOT:-$HOME/NOMAD}#/workspaces/isaac_ros-dev#"
}

NVBLOX_LAUNCH_CONTAINER="$(to_container_path "$NVBLOX_LAUNCH")"
NVBLOX_CONFIG_CONTAINER="$(to_container_path "$NVBLOX_CONFIG")"

zed_ready() {
    in_container "source /opt/ros/humble/setup.bash 2>/dev/null; source /workspaces/isaac_ros-dev/install/setup.bash 2>/dev/null; timeout 4 ros2 topic echo --once /zed/zed_node/odom >/dev/null 2>&1"
}

nvblox_built() {
    in_container "source /opt/ros/humble/setup.bash 2>/dev/null; source /workspaces/isaac_ros-dev/install/setup.bash 2>/dev/null; ros2 pkg list 2>/dev/null | grep -q nvblox_ros"
}

apply_overlay() {
    in_container "$(cat <<EOS
set -u
NOMAD_CFG="$NVBLOX_CONFIG_CONTAINER"
for base in \
    \$(python3 -c 'from ament_index_python.packages import get_package_share_directory; print(get_package_share_directory(\"nvblox_examples_bringup\"))' 2>/dev/null)/config/nvblox/nvblox_base.yaml \
    /workspaces/isaac_ros-dev/install/nvblox_examples_bringup/share/nvblox_examples_bringup/config/nvblox/nvblox_base.yaml; do
    if [ -f "\$base" ] && [ -f "\$NOMAD_CFG" ]; then
        cp "\$NOMAD_CFG" "\$base"
        echo "[nvblox] overlay applied to \$base"
        break
    fi
done
EOS
)"
}

write_launch_script() {
    local tmp
    tmp=$(mktemp)
    {
        echo "#!/bin/bash"
        # No `set -u`: ROS2's setup.bash isn't -u clean and would abort silently.
        ros_setup_prelude
        cat <<EOS
mkdir -p /workspaces/isaac_ros-dev/data 2>/dev/null || true
ros2 launch "$NVBLOX_LAUNCH_CONTAINER" \
    enable_nvblox:=true
EOS
    } > "$tmp"
    docker cp "$tmp" "$ISAAC_CONTAINER_NAME:$LAUNCH_SCRIPT_PATH" >/dev/null
    rm -f "$tmp"
    in_container "chmod +x $LAUNCH_SCRIPT_PATH"
}

svc_start() {
    require_container || return 1
    if ! nvblox_built; then
        log_fail "nvblox_ros package not built in this container. Run scripts/setup/provision_isaac_ros.sh."
        return 1
    fi
    if ! zed_ready; then
        log_fail "ZED is not publishing /zed/zed_node/odom; start zed_wrapper first."
        return 1
    fi

    kill_pattern_in_container "$PATTERN" 8
    apply_overlay
    write_launch_script

    log_info "launching nvblox"
    docker exec -d "$ISAAC_CONTAINER_NAME" \
        bash -c "nohup bash $LAUNCH_SCRIPT_PATH > $LAUNCH_LOG 2>&1 &"

    if wait_for "docker exec $ISAAC_CONTAINER_NAME pgrep -f 'nvblox_node|nomad_zed_nvblox'" 60; then
        log_ok "nvblox process up"
        return 0
    fi
    log_fail "nvblox did not appear within 60s"
    return 1
}

svc_stop() {
    if ! container_running; then
        log_ok "container not running; nothing to stop"
        return 0
    fi
    log_info "stopping nvblox"
    kill_pattern_in_container "$PATTERN" 10
    log_ok "stopped"
}

svc_status() { status_pattern_in_container 'nvblox_node|nomad_zed_nvblox\.launch\.py'; }

svc_logs() {
    require_container || return 1
    docker exec "$ISAAC_CONTAINER_NAME" tail -n 100 -F "$LAUNCH_LOG"
}

dispatch_service "$@"
