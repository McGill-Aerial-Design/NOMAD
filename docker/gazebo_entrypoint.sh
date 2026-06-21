#!/bin/bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
# ==============================================================
# Entrypoint for the NOMAD ArduPilot + Gazebo Harmonic SITL image.
#
# Sources ROS 2 Humble plus the ardupilot_gz colcon workspace (which provides
# SITL, the ardupilot_gazebo plugin, ardupilot_gz_bringup, and the micro-ROS
# agent), exports the Gazebo + NOMAD resource paths, prints the resolved
# environment for debuggability, then exec's the passed command.
# ==============================================================
set -e

# ROS 2 Humble (from the ros:humble base).
if [ -f /opt/ros/humble/setup.bash ]; then
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
fi

# The ardupilot_gz workspace (SITL + ardupilot_gazebo + bringup + micro_ros_agent).
# Built under the non-root build user's home (see Dockerfile.sim-gazebo).
for _ws in "${HOME}/ardu_ws/install/setup.bash" /home/nomad/ardu_ws/install/setup.bash /root/ardu_ws/install/setup.bash; do
    if [ -f "${_ws}" ]; then
        # shellcheck disable=SC1090,SC1091
        source "${_ws}"
        break
    fi
done

export GZ_VERSION="${GZ_VERSION:-harmonic}"
export PATH="${HOME}/.local/bin:/home/nomad/.local/bin:${PATH}"

# Make the NOMAD ZED-2i model + worlds discoverable by gz sim alongside the
# ardupilot_gazebo resources (which the workspace install already exports).
export GZ_SIM_RESOURCE_PATH="/opt/nomad/tools/sim/gazebo/models:/opt/nomad/tools/sim/gazebo/worlds:${GZ_SIM_RESOURCE_PATH}"

_is_truthy() {
    case "$(printf '%s' "${1:-}" | tr '[:upper:]' '[:lower:]')" in
        1|true|yes|on) return 0 ;;
        *) return 1 ;;
    esac
}

_start_virtual_desktop() {
    if _is_truthy "${NOMAD_GAZEBO_HEADLESS:-0}"; then
        return
    fi

    if _is_truthy "${NOMAD_GAZEBO_USE_HOST_DISPLAY:-0}"; then
        echo "[gazebo_entrypoint] using host display DISPLAY=${DISPLAY:-unset}"
        return
    fi

    export DISPLAY="${DISPLAY:-:99}"
    export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/tmp/runtime-${USER:-nomad}}"

    mkdir -p "${XDG_RUNTIME_DIR}"
    chmod 700 "${XDG_RUNTIME_DIR}"

    Xvfb "${DISPLAY}" \
        -screen 0 "${NOMAD_GAZEBO_SCREEN:-1600x900x24}" \
        -nolisten tcp +extension GLX +render -noreset \
        >/tmp/nomad-xvfb.log 2>&1 &

    # Give Xvfb a moment to create the display socket before Qt starts.
    sleep 1

    openbox >/tmp/nomad-openbox.log 2>&1 &

    x11vnc \
        -display "${DISPLAY}" \
        -forever -shared -nopw \
        -rfbport "${NOMAD_GAZEBO_VNC_PORT:-5900}" \
        -bg -o /tmp/nomad-x11vnc.log \
        >/dev/null 2>&1

    websockify \
        --web=/usr/share/novnc/ \
        "${NOMAD_GAZEBO_NOVNC_PORT:-6080}" \
        "localhost:${NOMAD_GAZEBO_VNC_PORT:-5900}" \
        >/tmp/nomad-novnc.log 2>&1 &

    echo "[gazebo_entrypoint] Gazebo GUI: http://localhost:${NOMAD_GAZEBO_NOVNC_PORT:-6080}/vnc.html"
}

_start_virtual_desktop

echo "[gazebo_entrypoint] ROS_DISTRO=${ROS_DISTRO:-unset} GZ_VERSION=${GZ_VERSION} ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}"

exec "$@"
