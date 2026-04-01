#!/bin/bash
# =============================================================================
# NOMAD Restart Script - kills all services and restarts cleanly
# =============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NOMAD_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
RUN_USER="${SUDO_USER:-$USER}"
LOG_FILE="/tmp/nomad_startup_${RUN_USER}_$(date +%Y%m%d_%H%M%S).log"

echo "Stopping all NOMAD services..."

# Stop systemd service when possible; otherwise terminate MainPID as fallback.
if systemctl is-active --quiet nomad 2>/dev/null; then
    STOPPED_SYSTEMD=false
    if [ "$EUID" -eq 0 ]; then
        if systemctl stop nomad 2>/dev/null; then
            STOPPED_SYSTEMD=true
        fi
    elif sudo -n true 2>/dev/null; then
        if sudo -n systemctl stop nomad 2>/dev/null; then
            STOPPED_SYSTEMD=true
        fi
    fi

    if [ "$STOPPED_SYSTEMD" = true ]; then
        echo "  Stopped systemd nomad.service"
        sleep 2
    else
        svc_pid=$(systemctl show nomad --property=MainPID --value 2>/dev/null || echo "")
        if [[ "$svc_pid" =~ ^[0-9]+$ ]] && [ "$svc_pid" -gt 1 ] && kill -0 "$svc_pid" 2>/dev/null; then
            echo "  [WARN] No sudo for systemctl stop. Terminating nomad MainPID $svc_pid instead."
            kill "$svc_pid" 2>/dev/null || true
            sleep 2
        else
            echo "  [WARN] Cannot stop systemd service and no MainPID fallback available."
        fi
    fi
fi

# Kill all NOMAD processes
pkill -f "start_nomad_full.sh" 2>/dev/null || true
pkill -f "edge_core.main" 2>/dev/null || true
pkill -f "mavlink-routerd" 2>/dev/null || true

# Release single-instance lock held by long-running wrapper processes
# (for example, when start_nomad_full transitions into `sleep infinity`).
if [ -f /tmp/nomad_start_nomad_full.lock ] && command -v fuser >/dev/null 2>&1; then
    LOCK_PIDS=$(fuser /tmp/nomad_start_nomad_full.lock 2>/dev/null || true)
    if [ -n "$LOCK_PIDS" ]; then
        echo "  Releasing start_nomad_full lock held by PID(s): $LOCK_PIDS"
        kill $LOCK_PIDS 2>/dev/null || true
        sleep 1
    fi
fi

# Kill ALL processes inside the Isaac ROS container (both launch paths)
if docker ps -a --format '{{.Names}}' 2>/dev/null | grep -q '^nomad_isaac_ros$'; then
    docker exec nomad_isaac_ros pkill -f "launch_nvblox_bridge\.sh|launch_zed_nvblox\.sh" 2>/dev/null || true
    docker exec nomad_isaac_ros pkill -f "simple_video_bridge" 2>/dev/null || true
    docker exec nomad_isaac_ros pkill -f "ros_http_bridge" 2>/dev/null || true
    docker exec nomad_isaac_ros pkill -f "ros2 launch" 2>/dev/null || true
    docker exec nomad_isaac_ros pkill -f "component_container" 2>/dev/null || true
    docker exec nomad_isaac_ros pkill -f "zed_node" 2>/dev/null || true

    # Clean up stale FastRTPS/DDS shared memory lock files inside container.
    docker exec nomad_isaac_ros bash -c 'rm -f /dev/shm/fastrtps_* 2>/dev/null' || true
fi

pkill -f "gst-launch" 2>/dev/null || true

# Wait and verify cleanup
sleep 2
if pgrep -f "edge_core.main" > /dev/null 2>&1; then
    echo "  Edge Core still alive, sending SIGKILL..."
    pkill -9 -f "edge_core.main" 2>/dev/null || true
    sleep 1
fi

echo "All NOMAD services stopped. Waiting 3 seconds before restart..."
sleep 3

# Start NOMAD services
if [ ! -d "$NOMAD_DIR" ]; then
    echo "[ERROR] NOMAD directory not found: $NOMAD_DIR"
    exit 1
fi

START_CMD="cd '$NOMAD_DIR' && nohup bash scripts/run/start_nomad_full.sh all > '$LOG_FILE' 2>&1 &"

if [ "$EUID" -eq 0 ] && [ -n "${SUDO_USER:-}" ]; then
    if sudo -u "$SUDO_USER" -H bash -lc "$START_CMD"; then
        echo "NOMAD services starting in background as $SUDO_USER. Check $LOG_FILE for progress."
    else
        echo "[ERROR] Failed to start NOMAD services as $SUDO_USER"
        exit 1
    fi
else
    if bash -lc "$START_CMD"; then
        echo "NOMAD services starting in background. Check $LOG_FILE for progress."
    else
        echo "[ERROR] Failed to start NOMAD services"
        exit 1
    fi
fi
