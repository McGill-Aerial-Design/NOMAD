# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
import asyncio
import os
import subprocess
import time

from fastapi import Request

try:
    import msgpack
except ImportError:  # pragma: no cover - optional Jetson dependency
    msgpack = None


_ISAAC_CONTAINER_NAME = os.environ.get("ISAAC_CONTAINER_NAME", "nomad_isaac_ros")


def register_services_routes(app, ctx) -> None:
    def _probe_isaac_container() -> dict:
        """Check if the Isaac ROS container is running."""
        try:
            r = subprocess.run(
                ["docker", "ps", "--filter", f"name={_ISAAC_CONTAINER_NAME}", "--format", "{{.Status}}"],
                capture_output=True,
                text=True,
                timeout=5,
            )
            return {"container_running": bool(r.stdout.strip())}
        except Exception:
            return {"container_running": False}

    # TTL cache for the subprocess probe results (systemctl/pgrep). Mission
    # Planner polls /api/services/status at ~1-2 Hz, which would otherwise fan
    # out to ~10 subprocesses per poll and thrash the OS process table.
    _proc_cache: dict = {"ts": 0.0, "data": None, "lock": asyncio.Lock()}
    _PROC_CACHE_TTL_S = 2.0

    # ==================== Services Status Endpoint ====================

    @app.get("/api/services/status", tags=["System"])
    async def services_status(request: Request):
        """
        Get status of all NOMAD services.

        Returns status of:
        - mavlink-router: MAVLink routing to CubePilot
        - mediamtx: RTSP video server
        - novnc: Browser-based remote desktop service
        - edge_core: This API service (always running if you see this)
        - isaac_ros: Isaac ROS bridge status
        - vio: VIO pipeline status
        """

        # ----------------------------------------------------------------
        # All subprocess.run() calls are blocking and must NOT run on the
        # FastAPI event loop — offload them to the default thread-pool so
        # other requests (telemetry, health, video) stay responsive.
        # ----------------------------------------------------------------
        def _blocking_proc_checks() -> tuple:
            """Collect service states that require blocking subprocesses."""
            svc: dict = {}

            # --- mavlink-router ---
            try:
                systemd_status = "inactive"
                systemd_running = False
                try:
                    r = subprocess.run(
                        ["systemctl", "is-active", "mavlink-router"],
                        capture_output=True,
                        text=True,
                        timeout=2,
                    )
                    systemd_status = r.stdout.strip() or "inactive"
                    systemd_running = r.returncode == 0
                except Exception:
                    pass

                process_running = False
                try:
                    pr = subprocess.run(
                        ["pgrep", "-f", "mavlink-routerd"],
                        capture_output=True,
                        text=True,
                        timeout=3,
                    )
                    process_running = pr.returncode == 0
                except Exception:
                    pass

                cubepilot_present = os.path.exists("/dev/ttyACM0")
                if not cubepilot_present:
                    svc["mavlink_router"] = {"status": "no_cubepilot", "running": False, "cubepilot_present": False}
                else:
                    mavlink_running = systemd_running or process_running
                    svc["mavlink_router"] = {
                        "status": "active" if mavlink_running else systemd_status,
                        "running": mavlink_running,
                        "cubepilot_present": True,
                    }
            except Exception as e:
                svc["mavlink_router"] = {"status": "error", "error": str(e)}

            # --- mediamtx ---
            try:
                systemd_running = False
                systemd_status = "inactive"
                try:
                    r = subprocess.run(
                        ["systemctl", "is-active", "mediamtx"],
                        capture_output=True,
                        text=True,
                        timeout=2,
                    )
                    systemd_status = r.stdout.strip() or "inactive"
                    systemd_running = r.returncode == 0
                except Exception:
                    pass

                process_running = False
                try:
                    pr = subprocess.run(
                        ["pgrep", "-f", "mediamtx"],
                        capture_output=True,
                        text=True,
                        timeout=3,
                    )
                    process_running = pr.returncode == 0
                except Exception:
                    pass

                mediamtx_running = systemd_running or process_running
                svc["mediamtx"] = {
                    "status": "active" if mediamtx_running else systemd_status,
                    "running": mediamtx_running,
                }
            except Exception as e:
                svc["mediamtx"] = {"status": "error", "error": str(e)}

            # --- noVNC ---
            try:
                user_running = False
                user_status = "inactive"
                try:
                    r = subprocess.run(
                        ["systemctl", "--user", "is-active", "novnc"],
                        capture_output=True,
                        text=True,
                        timeout=2,
                    )
                    user_status = r.stdout.strip() or "inactive"
                    user_running = r.returncode == 0
                except Exception:
                    pass

                system_running = False
                system_status = "inactive"
                try:
                    r = subprocess.run(
                        ["systemctl", "is-active", "novnc"],
                        capture_output=True,
                        text=True,
                        timeout=2,
                    )
                    system_status = r.stdout.strip() or "inactive"
                    system_running = r.returncode == 0
                except Exception:
                    pass

                websockify_running = x11vnc_running = False
                try:
                    websockify_running = (
                        subprocess.run(
                            ["pgrep", "-f", "[w]ebsockify.*6080"],
                            capture_output=True,
                            text=True,
                            timeout=3,
                        ).returncode
                        == 0
                    )
                    x11vnc_running = (
                        subprocess.run(
                            ["pgrep", "-f", "[x]11vnc.*-rfbport 5900"],
                            capture_output=True,
                            text=True,
                            timeout=3,
                        ).returncode
                        == 0
                    )
                except Exception:
                    pass

                novnc_running = user_running or system_running or (websockify_running and x11vnc_running)
                svc["novnc"] = {
                    "status": "active"
                    if novnc_running
                    else (user_status if user_status != "inactive" else system_status),
                    "running": novnc_running,
                    "url": "http://localhost:6080/vnc.html",
                }
            except Exception as e:
                svc["novnc"] = {"status": "error", "error": str(e)}

            # --- Isaac ROS container check ---
            runtime_state = _probe_isaac_container()
            return svc, runtime_state

        # Run blocking work in thread pool — event loop stays free.
        # TTL-cache the result so high-frequency polls don't spawn ~10
        # subprocesses per request.
        now = time.time()
        if _proc_cache["data"] is not None and (now - _proc_cache["ts"]) < _PROC_CACHE_TTL_S:
            proc_services, runtime_state = _proc_cache["data"]
        else:
            async with _proc_cache["lock"]:
                now = time.time()
                if _proc_cache["data"] is not None and (now - _proc_cache["ts"]) < _PROC_CACHE_TTL_S:
                    proc_services, runtime_state = _proc_cache["data"]
                else:
                    loop = asyncio.get_running_loop()
                    proc_services, runtime_state = await loop.run_in_executor(None, _blocking_proc_checks)
                    _proc_cache["data"] = (proc_services, runtime_state)
                    _proc_cache["ts"] = time.time()

        services = proc_services
        container_running = runtime_state["container_running"]

        # Edge Core is always running (we're responding)
        services["edge_core"] = {"status": "active", "running": True}

        # Isaac ROS status
        isaac_bridge = request.app.state.isaac_bridge
        if isaac_bridge:
            services["isaac_ros"] = {
                "status": "active",
                "running": True,
                "container_running": container_running,
                **isaac_bridge.get_status(),
            }
        else:
            services["isaac_ros"] = {
                "status": "active" if container_running else "not_initialized",
                "running": container_running,
                "container_running": container_running,
            }

        # VIO status — provided by modules that register VIO state
        external_vio_state = getattr(request.app.state, "external_vio_state", None)
        if external_vio_state:
            services["vio"] = {
                "status": "active",
                "running": True,
                "source": external_vio_state.get("source", "external"),
                "confidence": external_vio_state.get("confidence", 0),
            }
        else:
            services["vio"] = {
                "status": "not_initialized",
                "running": False,
            }

        services["isaac_ros_container"] = {
            "status": "running" if container_running else "not_running",
            "running": container_running,
        }

        return services
