#!/bin/bash
# =============================================================================
# nomad-isaac-ros-container service
#
# Owns: the long-running `nomad_isaac_ros` Docker container (sleep infinity).
# Does NOT own ZED, nvblox, the ros_http_bridge, or the video bridge — each of
# those has its own service that runs `docker exec` against this container.
#
# Stopping this service stops the container, which cascades to every in-
# container service (and they will be restarted by their own units after the
# container comes back up, via After=/Requires=).
# =============================================================================
set -u
SERVICE="isaac-ros-container"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=../lib/common.sh
. "$SCRIPT_DIR/../lib/common.sh"

resolve_image() {
    if docker image inspect "$ISAAC_IMAGE_NAME" >/dev/null 2>&1; then
        echo "$ISAAC_IMAGE_NAME"
    elif docker image inspect "$ISAAC_IMAGE_FALLBACK" >/dev/null 2>&1; then
        echo "$ISAAC_IMAGE_FALLBACK"
    else
        return 1
    fi
}

container_correctly_configured() {
    local init_ok mount_ok
    init_ok=$(docker inspect -f '{{.HostConfig.Init}}' "$ISAAC_CONTAINER_NAME" 2>/dev/null || echo false)
    mount_ok=$(docker inspect -f '{{range .Mounts}}{{.Destination}} {{end}}' "$ISAAC_CONTAINER_NAME" 2>/dev/null \
                 | grep -c '/sys/bus/usb' || true)
    [ "$init_ok" = "true" ] && [ "${mount_ok:-0}" -gt 0 ]
}

create_container() {
    local image="$1"
    log_info "creating container $ISAAC_CONTAINER_NAME from $image"
    docker run -d \
        --name "$ISAAC_CONTAINER_NAME" \
        --init \
        --runtime nvidia \
        --privileged \
        --network host \
        --ipc host \
        --shm-size 1g \
        -v "$ISAAC_WORKSPACE:/workspaces/isaac_ros-dev" \
        -v /dev:/dev \
        -v /sys/bus/usb:/sys/bus/usb \
        -v /run/udev:/run/udev:ro \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        -v /tmp/argus_socket:/tmp/argus_socket \
        -v /etc/localtime:/etc/localtime:ro \
        -v "$NOMAD_REPO_ROOT/config:/workspaces/isaac_ros-dev/config:ro" \
        -v "$NOMAD_REPO_ROOT/edge_core:/workspaces/isaac_ros-dev/edge_core:ro" \
        -e DISPLAY="${DISPLAY:-}" \
        -e NVIDIA_VISIBLE_DEVICES=all \
        -e NVIDIA_DRIVER_CAPABILITIES=all \
        -e EGL_PLATFORM=device \
        -e ROS_DOMAIN_ID="$ISAAC_ROS_DOMAIN_ID" \
        -e LD_LIBRARY_PATH=/usr/local/zed/lib:/opt/ros/humble/lib:/opt/ros/humble/lib/aarch64-linux-gnu \
        -e CMAKE_PREFIX_PATH=/usr/local/zed \
        -w /workspaces/isaac_ros-dev \
        "$image" \
        sleep infinity > /dev/null
}

svc_start() {
    if ! command -v docker >/dev/null 2>&1; then
        log_fail "docker is not installed"
        return 1
    fi
    local image
    if ! image="$(resolve_image)"; then
        log_fail "no Isaac ROS image available. Run scripts/setup/provision_isaac_ros.sh first."
        return 1
    fi

    if container_running; then
        if container_correctly_configured; then
            log_ok "already running"
            return 0
        fi
        log_warn "running but misconfigured (missing init reaper or /sys/bus/usb mount) — recreating"
        docker stop "$ISAAC_CONTAINER_NAME" >/dev/null 2>&1 || true
    fi

    if docker ps -a --format '{{.Names}}' | grep -qx "$ISAAC_CONTAINER_NAME"; then
        if container_correctly_configured; then
            log_info "container exists (stopped) — starting"
            docker start "$ISAAC_CONTAINER_NAME" >/dev/null
        else
            log_warn "container exists but misconfigured — recreating"
            docker rm -f "$ISAAC_CONTAINER_NAME" >/dev/null 2>&1 || true
            create_container "$image"
        fi
    else
        create_container "$image"
    fi

    if wait_for "docker exec $ISAAC_CONTAINER_NAME true" 15; then
        log_ok "container ready"
        return 0
    fi
    log_fail "container did not become ready in 15s"
    return 1
}

svc_stop() {
    if ! docker ps -a --format '{{.Names}}' | grep -qx "$ISAAC_CONTAINER_NAME"; then
        log_ok "container does not exist"
        return 0
    fi
    log_info "stopping container"
    docker stop "$ISAAC_CONTAINER_NAME" >/dev/null 2>&1 || true
    log_ok "stopped (kept for fast restart; remove with: docker rm $ISAAC_CONTAINER_NAME)"
}

svc_status() {
    if container_running; then
        log_ok "running ($(docker inspect -f '{{.Id}}' "$ISAAC_CONTAINER_NAME" 2>/dev/null | cut -c1-12))"
        return 0
    fi
    if docker ps -a --format '{{.Names}}' | grep -qx "$ISAAC_CONTAINER_NAME"; then
        log_warn "exists but stopped"
        return 1
    fi
    log_warn "does not exist"
    return 1
}

svc_logs() {
    if ! docker ps -a --format '{{.Names}}' | grep -qx "$ISAAC_CONTAINER_NAME"; then
        log_warn "container does not exist"
        return 1
    fi
    docker logs --tail 100 -f "$ISAAC_CONTAINER_NAME"
}

dispatch_service "$@"
