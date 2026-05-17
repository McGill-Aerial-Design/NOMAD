import asyncio
import json
import os
import re
import shlex
import shutil
import subprocess
import time
from datetime import datetime, timezone
from typing import Any, Optional

from fastapi import Body, HTTPException, Path, Query, Request, WebSocket
from fastapi.encoders import jsonable_encoder
from fastapi.responses import FileResponse
from fastapi.websockets import WebSocketDisconnect
from pydantic import BaseModel
from starlette.responses import JSONResponse

from ..api_models import (
    COMMAND_WHITELIST,
    MSGPACK_AVAILABLE,
    Task1CapturesList,
    Task2HitRequest,
    TerminalCommandRequest,
    TerminalCommandResponse,
    TerminalExecRequest,
    VIOAreaLoadRequest,
    VIOAreaSaveRequest,
    VIOUpdateRequest,
    NavPositionRequest,
    NavVelocityRequest,
)


class CameraTiltConfigPayload(BaseModel):
    channel: Optional[int] = None
    pwm_down: Optional[int] = None
    pwm_neutral: Optional[int] = None
    pwm_up: Optional[int] = None
    angle_range_deg: Optional[float] = None

try:
    import msgpack
except ImportError:  # pragma: no cover - optional Jetson dependency
    msgpack = None

def register_calibration_admin_servo_routes(app, ctx) -> None:
    logger = ctx.logger
    _probe_isaac_runtime_state = ctx.probe_isaac_runtime_state
    _require_terminal_api_key = ctx.require_terminal_api_key

    # ==================== Sensor Calibration Endpoints ====================

    @app.post("/api/calibration/zed/sensor-viewer/start", tags=["Calibration"])
    async def start_zed_sensor_viewer(request: Request):
        """
        Launch ZED Sensor Viewer on the Jetson desktop.

        This is intended for manual magnetometer calibration via noVNC.
        """
        _require_terminal_api_key()

        viewer_bin = shutil.which("ZED_Sensor_Viewer") or shutil.which(
            "zed_sensor_viewer"
        )
        fallback_bin = shutil.which("ZED_Calibration")
        if not viewer_bin and not fallback_bin:
            raise HTTPException(
                status_code=500,
                detail="Neither ZED_Sensor_Viewer nor ZED_Calibration was found in PATH",
            )

        display = (
            os.environ.get("NOMAD_CAL_DISPLAY") or os.environ.get("DISPLAY") or ":1"
        )
        request_host = request.url.hostname or ""
        novnc_host = request_host
        if not novnc_host or novnc_host in {"localhost", "127.0.0.1"}:
            novnc_host = (
                os.environ.get("TAILSCALE_IP")
                or os.environ.get("JETSON_IP")
                or "localhost"
            )
        novnc_url = (
            f"http://{novnc_host}:6080/vnc.html?autoconnect=0&reconnect=0&resize=scale"
        )

        deps_lib_dir = os.path.expanduser(
            "~/NOMAD/.deps/zed_viewer/root/usr/lib/aarch64-linux-gnu"
        )
        run_env = os.environ.copy()
        run_env["DISPLAY"] = display
        if os.path.isdir(deps_lib_dir):
            existing_ld = run_env.get("LD_LIBRARY_PATH", "")
            run_env["LD_LIBRARY_PATH"] = (
                f"{deps_lib_dir}:{existing_ld}" if existing_ld else deps_lib_dir
            )

        def _missing_shared_libs(binary_path: str) -> list[str]:
            try:
                result = subprocess.run(
                    ["ldd", binary_path],
                    capture_output=True,
                    text=True,
                    timeout=5,
                    env=run_env,
                )
                output = "\n".join(
                    part for part in [result.stdout, result.stderr] if part
                )
                missing = []
                for line in output.splitlines():
                    if "=> not found" not in line:
                        continue
                    lib_name = line.split("=>", 1)[0].strip()
                    if lib_name:
                        missing.append(lib_name)
                return missing
            except Exception:
                return []

        def _launch_and_verify(binary_path: str, process_name: str) -> int:
            proc = subprocess.Popen(
                [binary_path],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                env=run_env,
                start_new_session=True,
            )
            time.sleep(0.8)
            if proc.poll() is not None:
                raise RuntimeError(f"{process_name} process exited immediately")
            return proc.pid

        try:
            if viewer_bin:
                running_probe = subprocess.run(
                    ["pgrep", "-f", "ZED_Sensor_Viewer|zed_sensor_viewer"],
                    capture_output=True,
                    text=True,
                    timeout=3,
                )
                if running_probe.returncode == 0:
                    return {
                        "success": True,
                        "message": "ZED Sensor Viewer already running",
                        "display": display,
                        "tool": os.path.basename(viewer_bin),
                        "novnc_url": novnc_url,
                    }

            if fallback_bin:
                fallback_running = subprocess.run(
                    ["pgrep", "-f", "ZED_Calibration"],
                    capture_output=True,
                    text=True,
                    timeout=3,
                )
                if fallback_running.returncode == 0:
                    return {
                        "success": True,
                        "message": "ZED_Calibration already running",
                        "display": display,
                        "tool": os.path.basename(fallback_bin),
                        "fallback": True,
                        "novnc_url": novnc_url,
                    }
        except Exception:
            # Continue with launch attempt even if probe fails.
            pass

        viewer_missing_deps: list[str] = []
        if viewer_bin:
            viewer_missing_deps = _missing_shared_libs(viewer_bin)
            if not viewer_missing_deps:
                try:
                    pid = _launch_and_verify(viewer_bin, "ZED Sensor Viewer")
                    return {
                        "success": True,
                        "message": "ZED Sensor Viewer launched",
                        "display": display,
                        "tool": os.path.basename(viewer_bin),
                        "pid": pid,
                        "novnc_url": novnc_url,
                    }
                except Exception as e:
                    logger.warning(f"Sensor Viewer launch failed, trying fallback: {e}")
            else:
                logger.warning(
                    "Sensor Viewer missing shared libs: "
                    + ", ".join(viewer_missing_deps)
                )

        if fallback_bin:
            fallback_missing = _missing_shared_libs(fallback_bin)
            if fallback_missing:
                raise HTTPException(
                    status_code=500,
                    detail=(
                        "ZED_Calibration is missing shared libraries: "
                        + ", ".join(fallback_missing)
                    ),
                )

            try:
                pid = _launch_and_verify(fallback_bin, "ZED_Calibration")
                message = "ZED_Calibration launched"
                if viewer_missing_deps:
                    message = (
                        "ZED Sensor Viewer dependencies missing; "
                        "launched ZED_Calibration fallback"
                    )
                return {
                    "success": True,
                    "message": message,
                    "display": display,
                    "tool": os.path.basename(fallback_bin),
                    "fallback": True,
                    "missing_dependencies": viewer_missing_deps,
                    "pid": pid,
                    "novnc_url": novnc_url,
                }
            except Exception as e:
                logger.error(f"Failed to launch ZED_Calibration fallback: {e}")
                raise HTTPException(
                    status_code=500,
                    detail=(
                        "Failed to launch ZED calibration tools. "
                        + (
                            f"Sensor Viewer missing deps: {', '.join(viewer_missing_deps)}. "
                            if viewer_missing_deps
                            else ""
                        )
                        + f"Fallback error: {e}"
                    ),
                )

        raise HTTPException(
            status_code=500,
            detail=(
                "ZED Sensor Viewer could not be launched"
                + (
                    f" (missing dependencies: {', '.join(viewer_missing_deps)})"
                    if viewer_missing_deps
                    else ""
                )
            ),
        )

    @app.post("/api/calibration/imu/reset_biases", tags=["Calibration"])
    async def reset_imu_biases():
        """
        Reset ZED IMU bias values using the ZED calibration tool.

        This requires exclusive camera access. Refuse to run while the Isaac ROS
        container is active because ZED wrapper/nvblox may hold the camera.
        """
        _require_terminal_api_key()

        runtime_state = _probe_isaac_runtime_state(force_refresh=True)
        if runtime_state.get("container_running", False):
            raise HTTPException(
                status_code=409,
                detail="Stop Isaac ROS before resetting IMU biases; the ZED camera must be idle.",
            )

        candidates = [
            "ZED_Sensor_Calibration",
            "zed_sensor_calibration",
            "ZED_Calibration",
        ]
        calibration_bin = None
        for name in candidates:
            calibration_bin = shutil.which(name)
            if calibration_bin:
                break
        if not calibration_bin:
            raise HTTPException(
                status_code=500,
                detail="ZED IMU calibration tool not found in PATH",
            )

        def _run_reset() -> subprocess.CompletedProcess:
            return subprocess.run(
                [calibration_bin, "--cimu"],
                capture_output=True,
                text=True,
                timeout=60,
            )

        try:
            result = await asyncio.to_thread(_run_reset)
        except subprocess.TimeoutExpired:
            raise HTTPException(status_code=504, detail="ZED IMU bias reset timed out")
        except Exception as e:
            raise HTTPException(status_code=500, detail=f"Failed to run IMU reset: {e}")

        output = "\n".join(part for part in [result.stdout, result.stderr] if part).strip()
        if result.returncode != 0:
            raise HTTPException(
                status_code=500,
                detail=output or f"{os.path.basename(calibration_bin)} exited with {result.returncode}",
            )

        logger.info("ZED IMU biases reset using %s", calibration_bin)
        return {
            "success": True,
            "tool": os.path.basename(calibration_bin),
            "output": output,
            "timestamp": datetime.now(timezone.utc).isoformat(),
        }

    @app.post("/api/tools/rviz2/start", tags=["Tools"])
    async def start_rviz2(request: Request):
        """
        Launch RViz2 inside the Isaac ROS container and show it via noVNC.
        """
        _require_terminal_api_key()

        display = (
            os.environ.get("NOMAD_CAL_DISPLAY") or os.environ.get("DISPLAY") or ":1"
        )
        rviz_config_path = os.environ.get(
            "NOMAD_RVIZ_CONFIG_PATH",
            "/workspaces/isaac_ros-dev/install/nvblox_examples_bringup/share/nvblox_examples_bringup/config/visualization/zed_example.rviz",
        )
        rviz_fixed_frame = (os.environ.get("NOMAD_RVIZ_FIXED_FRAME") or "").strip()
        rviz_frame_candidates_env = os.environ.get(
            "NOMAD_RVIZ_FRAME_CANDIDATES",
            "map,odom,base_link,servo_mount,camera_link,zed_camera_center",
        )
        rviz_voxel_only = os.environ.get(
            "NOMAD_RVIZ_VOXEL_ONLY", "1"
        ).strip().lower() not in {"0", "false", "no", "off"}
        rviz_frame_candidates = [
            token.strip().lstrip("/")
            for token in rviz_frame_candidates_env.split(",")
            if token.strip()
        ]
        if rviz_fixed_frame:
            preferred = rviz_fixed_frame.lstrip("/")
            rviz_frame_candidates = [
                preferred,
                *[c for c in rviz_frame_candidates if c != preferred],
            ]
        request_host = request.url.hostname or ""
        novnc_host = request_host
        if not novnc_host or novnc_host in {"localhost", "127.0.0.1"}:
            novnc_host = (
                os.environ.get("TAILSCALE_IP")
                or os.environ.get("JETSON_IP")
                or "localhost"
            )
        novnc_url = (
            f"http://{novnc_host}:6080/vnc.html?autoconnect=0&reconnect=0&resize=scale"
        )

        run_env = os.environ.copy()
        run_env["DISPLAY"] = display
        run_env.setdefault("QT_X11_NO_MITSHM", "1")
        run_env.setdefault("XDG_RUNTIME_DIR", "/tmp")

        start_novnc_cmd = COMMAND_WHITELIST.get("start_novnc")
        if start_novnc_cmd:
            novnc_result = subprocess.run(
                ["bash", "-lc", start_novnc_cmd],
                capture_output=True,
                text=True,
                timeout=25,
                env=run_env,
            )
            if novnc_result.returncode != 0:
                novnc_output = "\n".join(
                    part for part in [novnc_result.stdout, novnc_result.stderr] if part
                ).strip()
                raise HTTPException(
                    status_code=500,
                    detail=(
                        "Failed to start noVNC desktop stack before launching RViz2. "
                        + (novnc_output or "No additional output available")
                    ),
                )

        try:
            running_probe = subprocess.run(
                ["pgrep", "-x", "rviz2"],
                capture_output=True,
                text=True,
                timeout=3,
            )
            if running_probe.returncode == 0:
                return {
                    "success": True,
                    "message": "RViz2 already running",
                    "display": display,
                    "tool": "rviz2",
                    "novnc_url": novnc_url,
                }
        except Exception:
            # Continue with launch attempt even if process probe fails.
            pass

        runtime_state = _probe_isaac_runtime_state(force_refresh=True)
        if not runtime_state.get("container_running", False):
            raise HTTPException(
                status_code=503,
                detail=(
                    "Isaac ROS container is not running. Start Task 2 / Isaac ROS "
                    "before launching RViz2."
                ),
            )

        container = "nomad_isaac_ros"
        selected_fixed_frame = (
            rviz_fixed_frame.lstrip("/") if rviz_fixed_frame else "map"
        )

        frame_probe_cmd = """
source /opt/ros/humble/setup.bash 2>/dev/null || true
source /workspaces/isaac_ros-dev/install/setup.bash 2>/dev/null || true
collect_frames() {
  topic="$1"
  timeout 6 ros2 topic echo "$topic" --once 2>/dev/null \
    | awk '/frame_id:|child_frame_id:/{gsub("\"", "", $2); gsub("^/", "", $2); print $2}' \
    || true
}
{
  collect_frames /tf_static
  collect_frames /tf
} | sed '/^$/d' | sort -u
"""
        try:
            frame_probe = subprocess.run(
                ["docker", "exec", container, "bash", "-lc", frame_probe_cmd],
                capture_output=True,
                text=True,
                timeout=15,
            )
            available_frames = {
                line.strip().lstrip("/")
                for line in (frame_probe.stdout or "").splitlines()
                if line.strip()
            }
            if available_frames:
                for candidate in rviz_frame_candidates:
                    if candidate in available_frames:
                        selected_fixed_frame = candidate
                        break
                else:
                    selected_fixed_frame = sorted(available_frames)[0]
        except Exception:
            # Keep configured default if probing fails.
            pass

        config_probe = subprocess.run(
            [
                "docker",
                "exec",
                container,
                "bash",
                "-lc",
                f"test -f {shlex.quote(rviz_config_path)}",
            ],
            capture_output=True,
            text=True,
            timeout=5,
        )
        if config_probe.returncode != 0:
            raise HTTPException(
                status_code=500,
                detail=(
                    "RViz2 config file not found in Isaac ROS container: "
                    f"{rviz_config_path}"
                ),
            )

        # Compatibility fix for Humble images where PointCloudBox requests
        # rviz/glsl150/box.geom, which may be missing in the container runtime.
        patch_cmd = """
set -e
src=/workspaces/isaac_ros-dev/src/isaac_ros_nvblox/nvblox_rviz_plugin/src/nvblox_plugin_visual.cpp
if [ ! -f "$src" ]; then
  echo __NVBLOX_PLUGIN_SRC_MISSING__
  exit 0
fi
if grep -q '"rviz/PointCloudBox"' "$src"; then
  sed -i 's/"rviz\\/PointCloudBox"/"rviz\\/PointCloudPoint"/g' "$src"
  source /opt/ros/humble/setup.bash
  cd /workspaces/isaac_ros-dev
  colcon build --packages-select nvblox_rviz_plugin > /tmp/nomad_rviz_patch_build.log 2>&1
  echo __NVBLOX_PATCH_APPLIED__
else
  echo __NVBLOX_PATCH_ALREADY_PRESENT__
fi
"""
        patch_result = subprocess.run(
            ["docker", "exec", "-u", "root", container, "bash", "-lc", patch_cmd],
            capture_output=True,
            text=True,
            timeout=90,
        )
        patch_output = "\n".join(
            part for part in [patch_result.stdout, patch_result.stderr] if part
        ).strip()
        if patch_result.returncode != 0:
            raise HTTPException(
                status_code=500,
                detail=(
                    "Failed to apply RViz2 nvblox compatibility patch before launch. "
                    + (patch_output or "No patch output available")
                ),
            )

        try:
            running_probe = subprocess.run(
                ["docker", "exec", container, "pgrep", "-x", "rviz2"],
                capture_output=True,
                text=True,
                timeout=3,
            )
            if running_probe.returncode == 0:
                return {
                    "success": True,
                    "message": "RViz2 already running",
                    "display": display,
                    "tool": "rviz2",
                    "fixed_frame": selected_fixed_frame,
                    "voxel_only": rviz_voxel_only,
                    "novnc_url": novnc_url,
                }
        except Exception:
            # Continue with launch attempt even if process probe fails.
            pass

        display_arg = shlex.quote(display)
        rviz_config_arg = shlex.quote(rviz_config_path)
        rviz_fixed_frame_arg = shlex.quote(selected_fixed_frame)
        voxel_only_cmd = ""
        if rviz_voxel_only:
            voxel_only_cmd = """
# Hide triangle mesh in RViz and prefer voxel/occupancy visualization.
awk '
/Class: nvblox_rviz_plugin\\/NvbloxMesh/ { mesh=1 }
mesh && /^[[:space:]]*Enabled:/ { sub(/Enabled:.*/, "Enabled: false"); mesh=0 }
{ print }
' /tmp/nomad/active_zed_example.rviz > /tmp/nomad/active_zed_example.rviz.tmp && mv /tmp/nomad/active_zed_example.rviz.tmp /tmp/nomad/active_zed_example.rviz || true
# Reduce Jetson load by disabling mesh updates when nvblox supports dynamic params.
# Run in background so RViz launch is not blocked while nvblox starts up.
(
    mesh_node=""
    for i in 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20; do
        mesh_node=$(ros2 node list 2>/dev/null | grep nvblox | head -n1 || true)
        if [ -z "$mesh_node" ]; then
            mesh_node=$(ros2 service list 2>/dev/null | sed -n 's#^\\(.*\\)/set_parameters$#\\1#p' | grep nvblox | head -n1 || true)
        fi
        if [ -n "$mesh_node" ]; then
            ros2 param set "$mesh_node" update_mesh_rate_hz 0.0
            exit 0
        fi
        sleep 1
    done
    echo "nvblox node not found"
) > /tmp/nomad/nvblox_mesh_rate.log 2>&1 &
"""
        launch_cmd = f"""
export DISPLAY={display_arg}
source /opt/ros/humble/setup.bash 2>/dev/null || true
source /workspaces/isaac_ros-dev/install/setup.bash 2>/dev/null || true
if ! command -v rviz2 >/dev/null 2>&1; then
    echo __RVIZ2_NOT_FOUND__
    exit 127
fi
if [ ! -f {rviz_config_arg} ]; then
        echo __RVIZ_CONFIG_NOT_FOUND__
        exit 66
fi
mkdir -p /tmp/nomad
cp {rviz_config_arg} /tmp/nomad/active_zed_example.rviz
sed -i -E "s/^([[:space:]]*Fixed Frame:).*/\\1 {rviz_fixed_frame_arg}/" /tmp/nomad/active_zed_example.rviz || true
{voxel_only_cmd}
nohup rviz2 -d /tmp/nomad/active_zed_example.rviz > /tmp/rviz2.log 2>&1 &
echo $!
"""

        try:
            launch_result = subprocess.run(
                ["docker", "exec", container, "bash", "-lc", launch_cmd],
                capture_output=True,
                text=True,
                timeout=20,
            )

            launch_output = "\n".join(
                part for part in [launch_result.stdout, launch_result.stderr] if part
            ).strip()

            if launch_result.returncode != 0:
                if (
                    "__RVIZ2_NOT_FOUND__" in launch_output
                    or launch_result.returncode == 127
                ):
                    raise HTTPException(
                        status_code=500,
                        detail=(
                            "rviz2 is not available in the current ROS environment. "
                            "Install rviz2 on the Jetson or source the correct ROS setup."
                        ),
                    )
                if "__RVIZ_CONFIG_NOT_FOUND__" in launch_output:
                    raise HTTPException(
                        status_code=500,
                        detail=(
                            "RViz2 config file not found in Isaac ROS container: "
                            f"{rviz_config_path}"
                        ),
                    )
                raise HTTPException(
                    status_code=500,
                    detail=(
                        "Failed to start RViz2 via ROS environment. "
                        + (launch_output or "No launch output available")
                    ),
                )

            pid = None
            for line in reversed((launch_result.stdout or "").splitlines()):
                token = line.strip()
                if token.isdigit():
                    pid = int(token)
                    break

            if pid is None:
                raise HTTPException(
                    status_code=500,
                    detail=(
                        "RViz2 launch command did not return a process id. "
                        + (launch_output or "No launch output available")
                    ),
                )

            time.sleep(1.0)
            alive_check = subprocess.run(
                [
                    "docker",
                    "exec",
                    container,
                    "bash",
                    "-lc",
                    f"kill -0 {pid} 2>/dev/null",
                ],
                capture_output=True,
                text=True,
                timeout=3,
            )
            if alive_check.returncode != 0:
                log_tail = subprocess.run(
                    [
                        "docker",
                        "exec",
                        container,
                        "bash",
                        "-lc",
                        "tail -40 /tmp/rviz2.log 2>/dev/null || true",
                    ],
                    capture_output=True,
                    text=True,
                    timeout=5,
                )
                tail_text = (log_tail.stdout or "").strip()
                raise HTTPException(
                    status_code=500,
                    detail=(
                        "RViz2 exited immediately after launch. "
                        + (
                            f"Recent log output:\n{tail_text}"
                            if tail_text
                            else "No RViz2 log output found"
                        )
                    ),
                )

            return {
                "success": True,
                "message": "RViz2 launched",
                "display": display,
                "tool": "rviz2",
                "fixed_frame": selected_fixed_frame,
                "voxel_only": rviz_voxel_only,
                "pid": pid,
                "novnc_url": novnc_url,
            }
        except HTTPException:
            raise
        except Exception as e:
            raise HTTPException(
                status_code=500,
                detail=(f"Failed to launch RViz2. Error: {e}"),
            )

    @app.post("/api/tools/rviz2/stop", tags=["Tools"])
    async def stop_rviz2():
        """
        Stop RViz2 processes to reduce CPU/GPU load.

        Attempts to stop RViz2 inside the Isaac ROS container and on the host.
        """
        _require_terminal_api_key()

        container = "nomad_isaac_ros"
        runtime_state = _probe_isaac_runtime_state(force_refresh=True)

        container_status = "not_running"
        container_output = ""
        if runtime_state.get("container_running", False):
            try:
                container_stop = subprocess.run(
                    [
                        "docker",
                        "exec",
                        "-u",
                        "root",
                        container,
                        "bash",
                        "-lc",
                        "pkill -9 -f '(^|/)rviz2([[:space:]]|$)' >/dev/null 2>&1 || true; "
                        "for i in 1 2 3 4 5 6; do "
                        "if ! pgrep -f '(^|/)rviz2([[:space:]]|$)' >/dev/null 2>&1; then echo stopped; exit 0; fi; "
                        "sleep 0.5; "
                        "done; "
                        "echo running",
                    ],
                    capture_output=True,
                    text=True,
                    timeout=12,
                )
                container_output = "\n".join(
                    part
                    for part in [container_stop.stdout, container_stop.stderr]
                    if part
                ).strip()
                container_status = (
                    "stopped" if "stopped" in (container_output or "") else "running"
                )
            except Exception as e:
                container_status = "error"
                container_output = str(e)

        host_stop = subprocess.run(
            [
                "bash",
                "-lc",
                "pkill -9 -f '(^|/)rviz2([[:space:]]|$)' >/dev/null 2>&1 || true; "
                "for i in 1 2 3 4 5 6; do "
                "if ! pgrep -f '(^|/)rviz2([[:space:]]|$)' >/dev/null 2>&1; then echo stopped; exit 0; fi; "
                "sleep 0.5; "
                "done; "
                "echo running",
            ],
            capture_output=True,
            text=True,
            timeout=12,
        )
        host_output = "\n".join(
            part for part in [host_stop.stdout, host_stop.stderr] if part
        ).strip()
        host_status = "stopped" if "stopped" in (host_output or "") else "running"

        success = host_status == "stopped" and container_status in {
            "stopped",
            "not_running",
        }

        return {
            "success": success,
            "message": (
                "RViz2 stopped"
                if success
                else "RViz2 stop requested but one or more processes are still running"
            ),
            "host_status": host_status,
            "container_status": container_status,
            "container_output": container_output,
        }

    # ==================== Admin Endpoints ====================

    @app.post("/api/admin/git-update", tags=["Admin"])
    async def git_update():
        """
        Update the NOMAD codebase from Git.

        Performs:
        1. git stash (save any local changes)
        2. git pull origin main
        3. chmod +x on all .sh scripts (recursive)

        Returns the output of each command.
        """
        _require_terminal_api_key()

        results = {
            "success": True,
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "steps": [],
        }
        errors: list[str] = []

        nomad_dir = os.path.expanduser("~/NOMAD")

        # git stash/pull can take tens of seconds when the working tree is
        # large; run them in a worker thread so MAVLink bridging and video
        # commands keep flowing on the event loop.
        def _run(cmd: list[str], timeout: int) -> subprocess.CompletedProcess:
            return subprocess.run(
                cmd,
                cwd=nomad_dir,
                capture_output=True,
                text=True,
                timeout=timeout,
            )

        try:
            # Step 1: git stash
            stash_result = await asyncio.to_thread(_run, ["git", "stash"], 30)
            stash_output_text = "\n".join(
                part for part in [stash_result.stdout, stash_result.stderr] if part
            ).lower()
            stash_created = (
                stash_result.returncode == 0
                and "no local changes to save" not in stash_output_text
            )
            results["steps"].append(
                {
                    "step": "git stash",
                    "success": stash_result.returncode == 0,
                    "output": stash_result.stdout.strip(),
                    "error": stash_result.stderr.strip()
                    if stash_result.returncode != 0
                    else None,
                }
            )

            # Step 2: git pull
            pull_result = await asyncio.to_thread(
                _run, ["git", "pull", "origin", "main"], 60
            )
            results["steps"].append(
                {
                    "step": "git pull origin main",
                    "success": pull_result.returncode == 0,
                    "output": pull_result.stdout.strip(),
                    "error": pull_result.stderr.strip()
                    if pull_result.returncode != 0
                    else None,
                }
            )

            if pull_result.returncode != 0:
                errors.append("Git pull failed")
            else:
                # Step 3: chmod +x scripts
                chmod_result = await asyncio.to_thread(
                    _run,
                    ["bash", "-c", "find scripts -name '*.sh' -exec chmod +x {} +"],
                    10,
                )
                results["steps"].append(
                    {
                        "step": "chmod +x scripts/**/*.sh",
                        "success": chmod_result.returncode == 0,
                        "output": "Scripts made executable"
                        if chmod_result.returncode == 0
                        else chmod_result.stdout.strip(),
                        "error": chmod_result.stderr.strip()
                        if chmod_result.returncode != 0
                        else None,
                    }
                )
                if chmod_result.returncode != 0:
                    chmod_error = (
                        chmod_result.stderr.strip()
                        or chmod_result.stdout.strip()
                        or "unknown error"
                    )
                    errors.append(f"chmod +x scripts/**/*.sh failed: {chmod_error}")

            if stash_created:
                stash_pop_result = await asyncio.to_thread(
                    _run, ["git", "stash", "pop"], 30
                )
                results["steps"].append(
                    {
                        "step": "git stash pop",
                        "success": stash_pop_result.returncode == 0,
                        "output": stash_pop_result.stdout.strip(),
                        "error": stash_pop_result.stderr.strip()
                        if stash_pop_result.returncode != 0
                        else None,
                    }
                )
                if stash_pop_result.returncode != 0:
                    stash_error = (
                        stash_pop_result.stderr.strip()
                        or stash_pop_result.stdout.strip()
                        or "unknown error"
                    )
                    errors.append(f"Git stash pop failed: {stash_error}")

            if errors:
                results["success"] = False
                results["error"] = "; ".join(errors)

            return results

        except subprocess.TimeoutExpired:
            results["success"] = False
            results["error"] = "Command timed out"
            return results
        except Exception as e:
            results["success"] = False
            results["error"] = str(e)
            return results

    @app.get("/api/admin/git-status", tags=["Admin"])
    async def git_status():
        """
        Get current Git status and branch info.

        Returns current branch, commit hash, and any uncommitted changes.
        """
        _require_terminal_api_key()

        nomad_dir = os.path.expanduser("~/NOMAD")

        try:
            # Get current branch
            branch_result = subprocess.run(
                ["git", "branch", "--show-current"],
                cwd=nomad_dir,
                capture_output=True,
                text=True,
                timeout=5,
            )

            # Get current commit
            commit_result = subprocess.run(
                ["git", "log", "--oneline", "-1"],
                cwd=nomad_dir,
                capture_output=True,
                text=True,
                timeout=5,
            )

            # Get status
            status_result = subprocess.run(
                ["git", "status", "--porcelain"],
                cwd=nomad_dir,
                capture_output=True,
                text=True,
                timeout=5,
            )

            return {
                "branch": branch_result.stdout.strip(),
                "commit": commit_result.stdout.strip(),
                "has_changes": len(status_result.stdout.strip()) > 0,
                "changes": status_result.stdout.strip().split("\n")
                if status_result.stdout.strip()
                else [],
                "timestamp": datetime.now(timezone.utc).isoformat(),
            }

        except Exception as e:
            return {
                "error": str(e),
                "branch": "unknown",
                "commit": "unknown",
                "has_changes": False,
            }

    @app.post("/api/admin/upload-gdrive-token", tags=["Admin"])
    async def upload_gdrive_token(request: Request):
        """
        Receive Google Drive OAuth2 token JSON from Mission Planner
        and save to ~/.nomad/gdrive_token.json on the Jetson.

        The token is generated via the one-time OAuth2 setup flow
        (python edge_core/gdrive_upload.py --setup <client_secret.json>)
        and contains a refresh_token for headless use.
        """
        _require_terminal_api_key()

        body = await request.body()
        if not body:
            raise HTTPException(status_code=400, detail="Empty request body")

        try:
            token_data = json.loads(body)
        except json.JSONDecodeError:
            raise HTTPException(status_code=400, detail="Invalid JSON")

        if "token" not in token_data and "refresh_token" not in token_data:
            raise HTTPException(
                status_code=400,
                detail="Missing required token fields (token or refresh_token)",
            )

        token_dir = os.path.expanduser("~/.nomad")
        os.makedirs(token_dir, exist_ok=True)
        token_path = os.path.join(token_dir, "gdrive_token.json")

        try:
            with open(token_path, "w") as f:
                json.dump(token_data, f, indent=2)
            os.chmod(token_path, 0o600)
            logger.info(f"Google Drive token saved to {token_path}")
            return {
                "success": True,
                "path": token_path,
            }
        except Exception as e:
            raise HTTPException(status_code=500, detail=f"Failed to save token: {e}")

    # ==================== Servo Control Endpoints ====================
    # Control Cube Orange servo outputs and relays via MAVLink.
    #
    # Camera tilt servo calibration (ZED camera on channel 14):
    #   700 us  → pointing down  (−45° pitch)
    #   1250 us → straight/level ( 0° pitch)
    #   1450 us → pointing up    (+45° pitch)
    # The range is NOT symmetric around 1500us (standard servo neutral) because
    # the camera arm is mechanically offset. Conversion uses piecewise linear
    # interpolation through the three calibration points.
    #
    # These defaults can be overridden at runtime via POST /api/servo/camera/config
    # (Mission Planner sends this on connect so the Jetson always uses UI settings).

    _camera_tilt_config: dict = {
        "channel":      14,    # ArduPilot servo output channel (1-indexed)
        "pwm_down":     700,   # PWM us → camera pointing down (−angle_range_deg)
        "pwm_neutral":  1250,  # PWM us → camera level (0° pitch)
        "pwm_up":       1450,  # PWM us → camera pointing up (+angle_range_deg)
        "angle_range_deg": 45, # ±45° physical range
    }

    def _pwm_to_servo_angle(pwm_us: int, cfg: dict) -> float:
        """Convert FC SERVO_OUTPUT_RAW PWM to angle_deg (0-180, 90=level).

        Uses piecewise linear interpolation through the three calibration points:
          pwm_down → 90 - angle_range_deg
          pwm_neutral → 90
          pwm_up → 90 + angle_range_deg
        """
        r = cfg["angle_range_deg"]
        n = cfg["pwm_neutral"]
        d = cfg["pwm_down"]
        u = cfg["pwm_up"]
        if pwm_us <= n:
            span = max(n - d, 1)
            pitch = -r * (n - pwm_us) / span
        else:
            span = max(u - n, 1)
            pitch = r * (pwm_us - n) / span
        return max(90.0 - r, min(90.0 + r, 90.0 + pitch))

    @app.get("/api/servo/status", tags=["Servo"])
    async def get_servo_status():
        """
        Get status of all servos.

        Returns current angle and enabled state for each servo.
        """
        try:
            from ..servo_controller import get_servo_controller

            controller = get_servo_controller()
            if not controller or not controller.is_available():
                return {
                    "available": False,
                    "error": "Servo controller not initialized",
                    "servos": {},
                }

            status = controller.get_status()
            status["available"] = True
            return status

        except ImportError:
            return {
                "available": False,
                "error": "Servo controller module not available",
                "servos": {},
            }
        except Exception as e:
            logger.error(f"Servo status error: {e}")
            return {"available": False, "error": str(e), "servos": {}}

    @app.post("/api/servo/channel/{channel}/pwm", tags=["Servo"])
    async def set_servo_channel_pwm(
        channel: int = Path(..., ge=1, le=16, description="Cube servo output channel"),
        pwm: int = Query(..., ge=500, le=2500, description="PWM command in microseconds"),
    ):
        """Set a Cube Orange servo output via MAV_CMD_DO_SET_SERVO."""
        try:
            from ..servo_controller import get_servo_controller

            controller = get_servo_controller()
            if not controller or not controller.is_available():
                raise HTTPException(
                    status_code=503, detail="Servo controller not available"
                )
            if not controller.set_channel_pwm(channel, pwm):
                raise HTTPException(status_code=500, detail="Failed to send Cube servo PWM")
            return {"status": "ok", "channel": channel, "pwm": pwm}

        except HTTPException:
            raise
        except Exception as e:
            logger.error(f"Set Cube servo PWM error: {e}")
            raise HTTPException(status_code=500, detail=str(e))

    @app.post("/api/servo/camera/tilt", tags=["Servo"])
    async def set_camera_tilt(
        angle: float = Query(..., ge=0, le=180, description="Tilt angle 0-180 degrees"),
    ):
        """
        Set camera tilt servo angle.

        Args:
            angle: Target angle in degrees
                - 0 = Looking down
                - 90 = Level (straight ahead)
                - 180 = Looking up

        Returns:
            Success status and new angle
        """
        try:
            from ..servo_controller import get_servo_controller

            controller = get_servo_controller()
            if not controller or not controller.is_available():
                raise HTTPException(
                    status_code=503, detail="Servo controller not available"
                )

            success = controller.set_camera_tilt(angle)

            if success:
                return {
                    "status": "ok",
                    "angle": angle,
                    "message": f"Camera tilt set to {angle} degrees",
                }
            else:
                raise HTTPException(status_code=500, detail="Failed to set camera tilt")

        except HTTPException:
            raise
        except Exception as e:
            logger.error(f"Camera tilt error: {e}")
            raise HTTPException(status_code=500, detail=str(e))

    @app.get("/api/servo/camera/tilt", tags=["Servo"])
    async def get_camera_tilt(request: Request):
        """
        Get current camera tilt angle.

        Returns feedback_angle from actual MAVLink SERVO_OUTPUT_RAW when available
        (piecewise linear conversion through down/neutral/up calibration points).
        Falls back to commanded angle from the servo controller.
        servo_tf_publisher polls this endpoint to drive the SLAM camera TF.
        """
        response: dict = {"angle": 90.0, "feedback_angle": None, "available": False}
        cfg = _camera_tilt_config

        # Primary: read actual PWM from FC SERVO_OUTPUT_RAW
        try:
            mav = request.app.state.mavlink_service
            if mav is not None:
                pwm = mav.get_servo_output_pwm(cfg["channel"])
                if pwm is not None and pwm > 0:
                    angle = _pwm_to_servo_angle(pwm, cfg)
                    response["feedback_angle"] = angle
                    response["angle"] = angle
                    response["pwm_us"] = pwm
                    response["available"] = True
        except Exception as e:
            logger.debug(f"Servo OUTPUT_RAW feedback error: {e}")

        # Fallback: commanded angle from servo controller
        try:
            from ..servo_controller import get_servo_controller
            controller = get_servo_controller()
            if controller and controller.is_available():
                commanded = controller.get_camera_tilt()
                if commanded is not None:
                    response["commanded_angle"] = commanded
                    response["available"] = True
                    if response["feedback_angle"] is None:
                        response["angle"] = commanded
        except Exception:
            pass

        return response

    @app.get("/api/servo/camera/config", tags=["Servo"])
    async def get_camera_tilt_config():
        """Return current camera tilt servo calibration config."""
        return dict(_camera_tilt_config)

    @app.post("/api/servo/camera/config", tags=["Servo"])
    async def set_camera_tilt_config(
        config: Optional[CameraTiltConfigPayload] = Body(None),
        channel: Optional[int] = Query(None, ge=1, le=16, description="ArduPilot servo output channel (1-indexed)"),
        pwm_down: Optional[int] = Query(None, ge=500, le=2500, description="PWM us for camera fully down"),
        pwm_neutral: Optional[int] = Query(None, ge=500, le=2500, description="PWM us for camera level"),
        pwm_up: Optional[int] = Query(None, ge=500, le=2500, description="PWM us for camera fully up"),
        angle_range_deg: Optional[float] = Query(None, ge=1.0, le=90.0, description="Physical range in degrees each way"),
    ):
        """Update camera tilt servo calibration (called by Mission Planner on connect)."""
        body = config or CameraTiltConfigPayload()
        next_config = {
            "channel": channel if channel is not None else body.channel,
            "pwm_down": pwm_down if pwm_down is not None else body.pwm_down,
            "pwm_neutral": pwm_neutral if pwm_neutral is not None else body.pwm_neutral,
            "pwm_up": pwm_up if pwm_up is not None else body.pwm_up,
            "angle_range_deg": angle_range_deg if angle_range_deg is not None else body.angle_range_deg,
        }
        for key, value in next_config.items():
            if value is None:
                next_config[key] = _camera_tilt_config[key]

        channel_i = int(next_config["channel"])
        pwm_down_i = int(next_config["pwm_down"])
        pwm_neutral_i = int(next_config["pwm_neutral"])
        pwm_up_i = int(next_config["pwm_up"])
        angle_range_f = float(next_config["angle_range_deg"])

        if not 1 <= channel_i <= 16:
            raise HTTPException(status_code=422, detail="channel must be 1-16")
        if not all(500 <= pwm <= 2500 for pwm in (pwm_down_i, pwm_neutral_i, pwm_up_i)):
            raise HTTPException(status_code=422, detail="PWM values must be 500-2500 us")
        if not 1.0 <= angle_range_f <= 90.0:
            raise HTTPException(status_code=422, detail="angle_range_deg must be 1-90")

        _camera_tilt_config.update({
            "channel": channel_i,
            "pwm_down": pwm_down_i,
            "pwm_neutral": pwm_neutral_i,
            "pwm_up": pwm_up_i,
            "angle_range_deg": angle_range_f,
        })

        applied = False
        try:
            from ..servo_controller import get_servo_controller

            controller = get_servo_controller()
            if controller is not None:
                applied = controller.configure_camera_tilt_mavlink(channel_i)
        except Exception as e:
            logger.error(f"Failed to apply camera tilt servo channel: {e}")

        logger.info(
            f"Camera tilt config updated: ch={channel_i} down={pwm_down_i}us "
            f"neutral={pwm_neutral_i}us up={pwm_up_i}us range=±{angle_range_f}°"
        )
        response = dict(_camera_tilt_config)
        response["applied"] = applied
        return response

    @app.post("/api/servo/shooter/trigger", tags=["Servo"])
    async def trigger_water_shooter(
        duration_ms: int = Query(
            200, ge=50, le=2000, description="Trigger duration in milliseconds"
        ),
        relay_number: Optional[int] = Query(
            None, ge=0, le=15, description="Cube relay number to trigger"
        ),
    ):
        """
        Trigger water pump through Cube Orange relay output.

        Args:
            duration_ms: How long to activate shooter (50-2000ms)

        Returns:
            Success status
        """
        try:
            from ..servo_controller import get_servo_controller

            controller = get_servo_controller()
            if not controller or not controller.is_available():
                raise HTTPException(
                    status_code=503, detail="Servo controller not available"
                )

            if relay_number is not None:
                controller.configure_water_pump_relay(relay_number)
            success = controller.trigger_water_shooter(duration_ms)

            if success:
                return {
                    "status": "ok",
                    "duration_ms": duration_ms,
                    "message": f"Water shooter triggered for {duration_ms}ms",
                }
            else:
                raise HTTPException(
                    status_code=500, detail="Failed to trigger water shooter"
                )

        except HTTPException:
            raise
        except Exception as e:
            logger.error(f"Water shooter error: {e}")
            raise HTTPException(status_code=500, detail=str(e))

    @app.post("/api/servo/enable", tags=["Servo"])
    async def enable_servos():
        """Enable all servo PWM outputs."""
        try:
            from ..servo_controller import get_servo_controller

            controller = get_servo_controller()
            if not controller:
                raise HTTPException(
                    status_code=503, detail="Servo controller not available"
                )

            controller.enable_all()
            return {"status": "ok", "message": "All servos enabled"}

        except HTTPException:
            raise
        except Exception as e:
            logger.error(f"Enable servos error: {e}")
            raise HTTPException(status_code=500, detail=str(e))

    @app.post("/api/servo/disable", tags=["Servo"])
    async def disable_servos():
        """Disable all servo PWM outputs (safety)."""
        try:
            from ..servo_controller import get_servo_controller

            controller = get_servo_controller()
            if not controller:
                raise HTTPException(
                    status_code=503, detail="Servo controller not available"
                )

            controller.disable_all()
            return {"status": "ok", "message": "All servos disabled"}

        except HTTPException:
            raise
        except Exception as e:
            logger.error(f"Disable servos error: {e}")
            raise HTTPException(status_code=500, detail=str(e))

