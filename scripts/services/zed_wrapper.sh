#!/bin/bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
# =============================================================================
# nomad-zed-wrapper service
#
# Owns: the in-container `ros2 launch zed_wrapper zed_camera.launch.py` process
# and the small NOMAD helper nodes that should live and die with the camera
# (optical-frame TF alias, servo_tf_publisher).
#
# Depends on: isaac_ros_container (no service-level pkills cross this boundary).
# Does NOT manage nvblox, the ros_http_bridge, or the video bridge.
# =============================================================================
set -u
SERVICE="zed-wrapper"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=../lib/common.sh
. "$SCRIPT_DIR/../lib/common.sh"

# Patterns this service owns. Each is anchored enough to avoid clashing with
# nvblox or the ROS-HTTP bridge.
PATTERN_LAUNCH='zed_camera\.launch\.py'
PATTERN_ZED_RUNTIME='component_container_isolated.*zed_container|zed_state_publisher'
PATTERN_HELPERS='servo_tf_publisher\.py|drone_state_publisher\.py|zed_left_camera_frame_optical'

LAUNCH_SCRIPT_PATH=/tmp/nomad_zed_wrapper_launch.sh
LAUNCH_LOG=/tmp/nomad_zed_wrapper.log

write_launch_script() {
    local tmp
    tmp=$(mktemp)
    {
        echo "#!/bin/bash"
        # NOTE: NO `set -u` here. ROS2's setup.bash references unset vars
        # internally and aborts immediately under -u, with stderr swallowed by
        # `2>/dev/null`. That manifests as a silent exit-1 from this launcher.
        ros_setup_prelude
        cat <<'EOS'
# Patch ZED wrapper config files in-place to NOMAD tuning before launch.
# These settings are read at startup and the YAML format is what the wrapper
# actually reads, so the patch must happen here (not in /config/ which is
# read-only-mounted).
python3 - <<'PYEOF'
from pathlib import Path
import os, re

GET = os.environ.get
DESIRED = {
    "common_stereo.yaml": {
        "pub_resolution":         f"'{GET('ZED_PUB_RESOLUTION','NATIVE')}'",
        "pub_downscale_factor":   GET('ZED_PUB_DOWNSCALE_FACTOR','1.0'),
        "publish_raw":            GET('ZED_PUBLISH_RAW','true'),
        "publish_left_right":     GET('ZED_PUBLISH_LEFT_RIGHT','true'),
        "publish_mag":            GET('ZED_PUBLISH_MAG','true'),
        "depth_confidence":       GET('ZED_DEPTH_CONFIDENCE','50'),
        "depth_texture_conf":     GET('ZED_DEPTH_TEXTURE_CONF','70'),
        "depth_mode":             f"'{GET('ZED_DEPTH_MODE','NEURAL_LIGHT')}'",
    },
    "zed2i.yaml": { "grab_resolution": f"'{GET('ZED_GRAB_RESOLUTION','HD1080')}'" },
    "zed2.yaml":  { "grab_resolution": f"'{GET('ZED_GRAB_RESOLUTION','HD1080')}'" },
}

def patch(path: Path, kv: dict):
    text = path.read_text()
    updated = text
    for k, v in kv.items():
        pat = rf"(?m)^(\s*{re.escape(k)}\s*:\s*).*$"
        if re.search(pat, updated):
            updated = re.sub(pat, rf"\g<1>{v}", updated)
        else:
            updated = updated.rstrip() + f"\n{k}: {v}\n"
    if updated != text:
        path.write_text(updated)
        print(f"[zed-wrapper] patched {path.name}")

roots = [Path("/workspaces/isaac_ros-dev/install"),
         Path("/workspaces/isaac_ros-dev/src"),
         Path("/opt/ros/humble/share")]
seen = set()
for root in roots:
    if not root.exists(): continue
    for yaml in root.glob("**/zed_wrapper/config/*.yaml"):
        if yaml in seen: continue
        seen.add(yaml)
        if yaml.name in DESIRED:
            patch(yaml, DESIRED[yaml.name])
PYEOF

# ZED + Isaac ROS Nitros: intra-process comms + transient_local durability
# crashes the composable container. Detect the launch-file knob and pass
# enable_ipc:=false when available; fall back to patching the launch file.
ZED_LAUNCH_FILE=$(python3 -c "from ament_index_python.packages import get_package_share_directory;import os; print(os.path.join(get_package_share_directory('zed_wrapper'),'launch','zed_camera.launch.py'))" 2>/dev/null)
IPC_ARGS=()
if [ -n "$ZED_LAUNCH_FILE" ] && grep -q "enable_ipc" "$ZED_LAUNCH_FILE" 2>/dev/null; then
    IPC_ARGS+=(enable_ipc:=false)
elif [ -n "$ZED_LAUNCH_FILE" ] && grep -q "use_intra_process_comms" "$ZED_LAUNCH_FILE" 2>/dev/null; then
    python3 - "$ZED_LAUNCH_FILE" <<'PYEOF'
import re,sys
from pathlib import Path
p=Path(sys.argv[1]); t=p.read_text()
u=re.sub(r"(['\"]use_intra_process_comms['\"]\s*:\s*)True", r"\1False", t)
if u!=t: p.write_text(u)
PYEOF
fi

ros2 launch zed_wrapper zed_camera.launch.py \
    camera_model:="${ZED_CAMERA_MODEL}" \
    camera_name:="${ZED_CAMERA_NAME}" \
    "${IPC_ARGS[@]}" &
ZED_PID=$!

# Helper nodes that follow the camera lifecycle. They are intentionally NOT
# managed as separate services because they have no value without the ZED
# topics; tying them here keeps the dependency explicit.
sleep 3
ros2 run tf2_ros static_transform_publisher \
    --x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0 \
    --frame-id zed_left_camera_frame_optical \
    --child-frame-id zed_left_camera_optical_frame \
    >/tmp/nomad_zed_optical_alias.log 2>&1 &

python3 /workspaces/isaac_ros-dev/edge_core/ros/servo_tf_publisher.py \
    --host "${NOMAD_DOCKER_HOST_IP:-172.17.0.1}" \
    --port "${NOMAD_API_PORT}" \
    --tf-rate 20.0 \
    --poll-rate 10.0 \
    --odom-topic "${ROS_HTTP_BRIDGE_VIO_TOPIC}" \
    >/tmp/nomad_servo_tf_publisher.log 2>&1 &

wait "$ZED_PID"
EOS
    } > "$tmp"
    docker cp "$tmp" "$ISAAC_CONTAINER_NAME:$LAUNCH_SCRIPT_PATH" >/dev/null
    rm -f "$tmp"
    in_container "chmod +x $LAUNCH_SCRIPT_PATH"
}

svc_start() {
    require_container || return 1

    # Only kill the patterns this service owns.
    kill_pattern_in_container "$PATTERN_LAUNCH" 5
    kill_pattern_in_container "$PATTERN_ZED_RUNTIME" 5
    kill_pattern_in_container "$PATTERN_HELPERS" 5

    # Clean stale FastRTPS shared-memory locks left by our own previous PIDs.
    in_container 'rm -f /dev/shm/fastrtps_* 2>/dev/null || true'

    write_launch_script

    # Propagate the ZED env knobs from nomad.env into the in-container run.
    local env_args=()
    for v in ZED_CAMERA_MODEL ZED_CAMERA_NAME ZED_PUB_RESOLUTION \
             ZED_PUB_DOWNSCALE_FACTOR ZED_PUBLISH_RAW ZED_PUBLISH_LEFT_RIGHT \
             ZED_PUBLISH_MAG ZED_DEPTH_CONFIDENCE ZED_DEPTH_TEXTURE_CONF \
             ZED_DEPTH_MODE \
             ZED_GRAB_RESOLUTION NOMAD_API_PORT ROS_HTTP_BRIDGE_VIO_TOPIC \
             NOMAD_API_KEY; do
        env_args+=("-e" "${v}=${!v}")
    done

    log_info "launching zed_camera.launch.py inside $ISAAC_CONTAINER_NAME"
    docker exec "${env_args[@]}" -d "$ISAAC_CONTAINER_NAME" \
        bash -c "nohup bash $LAUNCH_SCRIPT_PATH > $LAUNCH_LOG 2>&1 &"

    log_info "waiting for /zed/zed_node/odom to publish (timeout ${ZED_READY_TIMEOUT_S}s)"
    if wait_for "docker exec $ISAAC_CONTAINER_NAME bash -c 'source /opt/ros/humble/setup.bash 2>/dev/null; source /workspaces/isaac_ros-dev/install/setup.bash 2>/dev/null; timeout 8 ros2 topic info --no-daemon /zed/zed_node/odom 2>/dev/null | grep -Eq \"Publisher count: [1-9]\"'" \
            "$ZED_READY_TIMEOUT_S"; then
        log_ok "ZED publishing odometry"
        return 0
    fi
    log_warn "ZED did not publish odometry within ${ZED_READY_TIMEOUT_S}s (check: nomad logs zed_wrapper)"
    return 1
}

svc_stop() {
    if ! container_running; then
        log_ok "container not running; nothing to stop"
        return 0
    fi
    log_info "stopping zed launch + helper nodes"
    kill_pattern_in_container "$PATTERN_LAUNCH" 5
    kill_pattern_in_container "$PATTERN_ZED_RUNTIME" 5
    kill_pattern_in_container "$PATTERN_HELPERS" 5
    log_ok "stopped"
}

svc_status() { status_pattern_in_container "$PATTERN_LAUNCH"; }

svc_logs() {
    require_container || return 1
    docker exec "$ISAAC_CONTAINER_NAME" tail -n 100 -F "$LAUNCH_LOG"
}

dispatch_service "$@"
