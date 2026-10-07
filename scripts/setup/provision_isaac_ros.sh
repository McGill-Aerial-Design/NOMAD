#!/bin/bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
#
# Verify the optional Isaac ROS base and build the generic NOMAD ROS adapter
# image. Camera, mapping, and obstacle-avoidance provisioning is intentionally
# not performed by this baseline.
set -euo pipefail

THIS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$THIS_DIR/../.." && pwd)"
# shellcheck source=../lib/common.sh
. "$REPO_ROOT/scripts/lib/common.sh"
load_nomad_env || exit 1

IMAGE="${ISAAC_IMAGE_NAME:-nomad-isaac-ros:latest}"
BASE_IMAGE="${ISAAC_IMAGE_FALLBACK:-isaac_ros_dev-aarch64}"

main() {
    command -v docker >/dev/null 2>&1 || { log_fail "docker is not installed"; return 1; }
    if ! docker image inspect "$BASE_IMAGE" >/dev/null 2>&1; then
        log_fail "Isaac ROS base image not found: $BASE_IMAGE"
        log_fail "Build the Isaac ROS base with NVIDIA's isaac_ros_common workflow first."
        return 1
    fi

    log_info "building generic NOMAD ROS adapter image: $IMAGE"
    docker build -f "$REPO_ROOT/docker/Dockerfile.jetson" \
        --build-arg "BASE_IMAGE=$BASE_IMAGE" \
        -t "$IMAGE" "$REPO_ROOT"
    log_ok "generic ROS adapter image built"
    log_info "No camera, mapping, or obstacle-avoidance provider is enabled."
}

main "$@"
