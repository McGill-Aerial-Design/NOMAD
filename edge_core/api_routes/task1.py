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

from fastapi import HTTPException, Query, Request, WebSocket
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

try:
    import msgpack
except ImportError:  # pragma: no cover - optional Jetson dependency
    msgpack = None

def register_task1_routes(app, ctx) -> None:
    logger = ctx.logger

    def _call_ros2_service_in_isaac_container_or_raise(*args, **kwargs):
        return ctx.call_ros2_service_in_isaac_container_or_raise(*args, **kwargs)

    def _is_transient_target_capture_error(detail: str) -> bool:
        msg = (detail or "").strip().lower()
        transient_markers = (
            "no rgb or depth image available",
            "camera intrinsics not yet received",
            "ros2 service not available: /target_localizer/capture_target",
            "failed to probe ros2 service: /target_localizer/capture_target",
            "isaac ros container stack is not running",
            "isaac ros nvblox stack is not running",
            "ros2 service call timed out: /target_localizer/capture_target",
        )
        return any(marker in msg for marker in transient_markers)

    async def _call_target_capture_with_retries(
        *,
        max_attempts: int = 3,
        retry_delay_s: float = 2.0,
        timeout_s: float = 45.0,
    ) -> str:
        attempts = max(1, int(max_attempts))
        last_exc: HTTPException | None = None
        loop = asyncio.get_running_loop()

        def _blocking_capture() -> str:
            return _call_ros2_service_in_isaac_container_or_raise(
                service_name="/target_localizer/capture_target",
                service_type="std_srvs/srv/Trigger",
                request_payload={},
                timeout_s=timeout_s,
                skip_type_check=True,
                force_refresh_runtime=False,
            )

        for attempt in range(1, attempts + 1):
            try:
                return await loop.run_in_executor(None, _blocking_capture)
            except HTTPException as exc:
                last_exc = exc
                detail = str(exc.detail)
                should_retry = (
                    attempt < attempts and _is_transient_target_capture_error(detail)
                )
                if should_retry:
                    logger.warning(
                        "Task1 capture transient failure (attempt %s/%s): %s",
                        attempt,
                        attempts,
                        detail,
                    )
                    await asyncio.sleep(max(0.1, float(retry_delay_s)))
                    continue
                raise

        if last_exc is not None:
            raise last_exc
        raise HTTPException(status_code=503, detail="Target capture unavailable")

    # ==================== Task 1: Recon (Outdoor) ====================

    @app.post("/api/task/1/capture", tags=["Task 1"])
    async def task1_capture(request: Request):
        """
        Trigger target detection for Task 1 recon mission.

        Delegates to the target_localizer ROS 2 node running in the Isaac
        ROS container. The node captures the current ZED frame, runs HSV
        circle detection, back-projects to 3D, and generates a description.
        """
        try:
            output = await _call_target_capture_with_retries(
                max_attempts=1,
                retry_delay_s=0.5,
                timeout_s=12.0,
            )
            return {
                "success": True,
                "output": output,
                "mode": "target_localizer",
            }
        except HTTPException as exc:
            if (
                (os.environ.get("NOMAD_ENABLE_TASK1_CAPTURE_FALLBACK") or "")
                .strip()
                .lower()
                not in {"1", "true", "yes", "on"}
            ):
                raise

            # Fallback to the live detections cache so Task 1 capture can still
            # produce useful output when the target_localizer service is unavailable.
            detail = str(exc.detail)
            with request.app.state.detection_state_lock:
                current = list(request.app.state.detected_objects)
                history = list(request.app.state.detection_history)
                last_update = request.app.state.detection_last_update

            fallback_candidates = current if current else history[-10:]
            age_seconds = time.time() - last_update if last_update > 0 else None

            if fallback_candidates:
                best = max(
                    fallback_candidates,
                    key=lambda d: float(d.get("confidence", 0.0) or 0.0),
                )
                label = best.get("label") or best.get("class_name") or "target"
                confidence = float(best.get("confidence", 0.0) or 0.0)
                x = best.get("x")
                y = best.get("y")
                z = best.get("z")
                pos = ""
                if x is not None and y is not None and z is not None:
                    pos = f" at ({float(x):.2f}, {float(y):.2f}, {float(z):.2f})"

                output = (
                    f"Fallback capture: top detection '{label}' "
                    f"(confidence {confidence:.2f}){pos}."
                )
                return {
                    "success": True,
                    "output": output,
                    "mode": "detections_fallback",
                    "detail": detail,
                    "current_count": len(current),
                    "history_count": len(history),
                    "age_seconds": age_seconds,
                }

            return {
                "success": False,
                "output": "Capture fallback has no detections available.",
                "mode": "detections_fallback",
                "detail": detail,
                "current_count": len(current),
                "history_count": len(history),
                "age_seconds": age_seconds,
            }

    @app.get("/api/task/1/images/{filename}", tags=["Task 1"])
    async def task1_get_image(filename: str):
        """
        Retrieve a saved Task 1 image (legacy endpoint for backward compatibility).

        Returns the image file captured during a Task 1 recon mission.
        For new folder-based structure, use /api/task/1/images/{folder}/{filename}
        """
        image_dir = "./data/task1_captures"
        image_path = os.path.join(image_dir, filename)

        # Validate filename to prevent directory traversal
        if ".." in filename or "/" in filename or "\\" in filename:
            raise HTTPException(status_code=400, detail="Invalid filename")

        # Check if file exists
        if not os.path.exists(image_path):
            # Backward-compatible fallback for folder-based captures.
            if os.path.isdir(image_dir):
                for folder in sorted(os.listdir(image_dir), reverse=True):
                    candidate_path = os.path.join(image_dir, folder, filename)
                    if os.path.isfile(candidate_path):
                        image_path = candidate_path
                        break
                else:
                    raise HTTPException(status_code=404, detail="Image not found")
            else:
                raise HTTPException(status_code=404, detail="Image not found")

        return FileResponse(image_path, media_type="image/jpeg")

    @app.get("/api/task/1/captures", tags=["Task 1"], response_model=Task1CapturesList)
    async def list_task1_captures():
        """
        List all Task 1 capture folders.

        Returns: List of folder names (timestamps) sorted by date descending.
        Example: ["20260202_120000", "20260202_115500", ...]
        """
        timestamp_pattern = re.compile(r"^\d{8}_\d{6}$")
        seen: set[str] = set()
        valid_folders: list[str] = []

        for base_dir in _task1_capture_base_dirs():
            if not os.path.isdir(base_dir):
                continue
            try:
                for entry in os.listdir(base_dir):
                    if entry in seen:
                        continue
                    if not timestamp_pattern.match(entry):
                        continue
                    if os.path.isdir(os.path.join(base_dir, entry)):
                        seen.add(entry)
                        valid_folders.append(entry)
            except Exception as e:
                logger.warning(f"Failed to scan captures in {base_dir}: {e}")

        valid_folders.sort(reverse=True)
        return Task1CapturesList(captures=valid_folders, count=len(valid_folders))

    @app.get("/api/task/1/images/{folder}/{filename}", tags=["Task 1"])
    async def get_task1_image_with_folder(folder: str, filename: str):
        """
        Download specific file from Task 1 capture folder.

        Args:
            folder: Folder name (e.g., "20260202_120000")
            filename: File name (e.g., "photo.jpg", "metadata.json")

        Returns: File content with appropriate content type
        """
        # Security: Validate folder name matches timestamp pattern
        timestamp_pattern = re.compile(r"^\d{8}_\d{6}$")
        if not timestamp_pattern.match(folder):
            raise HTTPException(status_code=400, detail="Invalid folder name format")

        # Security: Validate filename - whitelist known artifacts and target frames
        allowed_files = [
            "photo.jpg",
            "metadata.json",
            "description.txt",
            "detections_overlay.jpg",
            "building_3d_snapshot.jpg",
        ]
        # Accept both legacy numeric (target_00.jpg) and letter IDs (target_A.jpg,
        # target_AB.jpg) produced by target_localizer_node.py. Letters wrap
        # A..Z, AA..ZZ, etc.
        dynamic_target_image = re.compile(
            r"^target_(?:\d+|[A-Za-z]+)\.(jpg|jpeg)$", re.IGNORECASE
        )
        if filename not in allowed_files and not dynamic_target_image.match(filename):
            raise HTTPException(
                status_code=400,
                detail=(
                    "Invalid filename. Allowed static files: "
                    f"{', '.join(allowed_files)}, plus target_<id>.jpg"
                ),
            )

        # Security: Prevent path traversal
        if ".." in folder or "/" in folder or "\\" in folder:
            raise HTTPException(status_code=400, detail="Invalid folder name")
        if ".." in filename or "/" in filename or "\\" in filename:
            raise HTTPException(status_code=400, detail="Invalid filename")

        # Resolve from host captures first, then mirror from container if needed.
        file_path = _resolve_task1_capture_file(folder, filename)
        if not file_path:
            raise HTTPException(
                status_code=404, detail=f"File not found: {folder}/{filename}"
            )

        # Determine media type
        media_type = "application/octet-stream"
        if filename.endswith(".jpg") or filename.endswith(".jpeg"):
            media_type = "image/jpeg"
        elif filename.endswith(".json"):
            media_type = "application/json"
        elif filename.endswith(".txt"):
            media_type = "text/plain"

        return FileResponse(file_path, media_type=media_type)

    def _task1_capture_base_dirs() -> list[str]:
        """Candidate host directories where Task 1 captures may be stored."""
        env_dir = os.environ.get("NOMAD_TASK1_CAPTURE_DIR", "").strip()
        candidates = [
            *([] if not env_dir else [env_dir]),
            # Primary: writable bind-mount path
            # container /workspaces/isaac_ros-dev (rw) -> host /home/mad/workspaces/isaac_ros-dev
            # config/ and edge_core/ subdirs are overlaid read-only; data/ uses the rw base mount
            os.path.expanduser("~/workspaces/isaac_ros-dev/data/task1_captures"),
            # Legacy fallbacks
            "./data/task1_captures",
            os.path.expanduser("~/NOMAD/data/task1_captures"),
        ]
        seen: set[str] = set()
        resolved: list[str] = []
        for path in candidates:
            abs_path = os.path.abspath(path)
            if abs_path in seen:
                continue
            seen.add(abs_path)
            resolved.append(abs_path)
        return resolved

    def _latest_task1_capture_from_host() -> tuple[Optional[str], list[str]]:
        """Return newest timestamp folder and its jpg list from host storage."""
        timestamp_pattern = re.compile(r"^\d{8}_\d{6}$")
        latest_folder: Optional[str] = None
        latest_images: list[str] = []

        for base_dir in _task1_capture_base_dirs():
            if not os.path.isdir(base_dir):
                continue

            try:
                for folder in os.listdir(base_dir):
                    folder_path = os.path.join(base_dir, folder)
                    if not os.path.isdir(folder_path):
                        continue
                    if not timestamp_pattern.match(folder):
                        continue

                    images = sorted(
                        [
                            name
                            for name in os.listdir(folder_path)
                            if name.lower().endswith(".jpg")
                        ]
                    )

                    if latest_folder is None or folder > latest_folder:
                        latest_folder = folder
                        latest_images = images
            except Exception:
                continue

        return latest_folder, latest_images

    def _latest_task1_capture_from_container() -> tuple[Optional[str], list[str]]:
        """Return newest timestamp folder and jpg list from Isaac container storage."""
        base_path = "/workspaces/isaac_ros-dev/data/task1_captures"

        try:
            folder_probe = subprocess.run(
                [
                    "docker",
                    "exec",
                    "nomad_isaac_ros",
                    "bash",
                    "-lc",
                    (
                        "base='/workspaces/isaac_ros-dev/data/task1_captures'; "
                        "[ -d \"$base\" ] || exit 2; "
                        "ls -1 \"$base\" 2>/dev/null | "
                        "grep -E '^[0-9]{8}_[0-9]{6}$' | sort -r | head -n1"
                    ),
                ],
                capture_output=True,
                text=True,
                timeout=6,
            )
        except Exception:
            return None, []

        folder = (folder_probe.stdout or "").strip()
        if folder_probe.returncode != 0 or not folder:
            return None, []

        safe_base = shlex.quote(base_path)
        safe_folder = shlex.quote(folder)
        image_cmd = (
            f"base={safe_base}; "
            f"folder={safe_folder}; "
            "[ -d \"$base/$folder\" ] || exit 2; "
            "ls -1 \"$base/$folder\" 2>/dev/null | grep -Ei '\\.(jpg|jpeg)$' | sort"
        )

        try:
            image_probe = subprocess.run(
                ["docker", "exec", "nomad_isaac_ros", "bash", "-lc", image_cmd],
                capture_output=True,
                text=True,
                timeout=6,
            )
        except Exception:
            return folder, []

        if image_probe.returncode != 0:
            return folder, []

        images = [line.strip() for line in (image_probe.stdout or "").splitlines() if line.strip()]
        return folder, images

    def _mirror_task1_capture_from_container(folder: str, filenames: list[str]) -> None:
        """Copy all files from a container capture folder to host storage."""
        host_bases = _task1_capture_base_dirs()
        if not host_bases or not filenames:
            return
        host_folder = os.path.join(host_bases[0], folder)
        os.makedirs(host_folder, exist_ok=True)
        for filename in filenames:
            host_path = os.path.join(host_folder, filename)
            if os.path.isfile(host_path):
                continue
            container_path = f"/workspaces/isaac_ros-dev/data/task1_captures/{folder}/{filename}"
            try:
                subprocess.run(
                    ["docker", "cp", f"nomad_isaac_ros:{container_path}", host_path],
                    capture_output=True,
                    text=True,
                    timeout=8,
                )
            except Exception:
                pass

    def _copy_task1_file_from_container(
        folder: str,
        filename: str,
    ) -> Optional[str]:
        """Mirror a Task 1 capture file from Isaac container into host storage."""
        host_bases = _task1_capture_base_dirs()
        if not host_bases:
            return None

        host_folder = os.path.join(host_bases[0], folder)
        os.makedirs(host_folder, exist_ok=True)
        host_path = os.path.join(host_folder, filename)

        if os.path.isfile(host_path):
            return host_path

        container_path = f"/home/mad/NOMAD/data/task1_captures/{folder}/{filename}"
        try:
            copy_result = subprocess.run(
                ["docker", "cp", f"nomad_isaac_ros:{container_path}", host_path],
                capture_output=True,
                text=True,
                timeout=8,
            )
            if copy_result.returncode == 0 and os.path.isfile(host_path):
                return host_path
        except Exception:
            pass

        return None

    def _resolve_task1_capture_file(folder: str, filename: str) -> Optional[str]:
        """Resolve capture file on host, with container mirror fallback."""
        for base_dir in _task1_capture_base_dirs():
            candidate = os.path.join(base_dir, folder, filename)
            if os.path.isfile(candidate):
                return candidate

        return _copy_task1_file_from_container(folder, filename)

    # ==================== Task 1: Target Localizer (ROS 2) ====================

    _FACE_RE = re.compile(r"(?:on|near)\s+the\s+(north|south|east|west)\s+face", re.IGNORECASE)

    def _parse_building_face(output: str) -> str:
        """Extract building face from target_localizer description, or 'Unknown'."""
        m = _FACE_RE.search(output or "")
        return m.group(1).lower() if m else "Unknown"

    _DISTANCE_CM_RE = re.compile(
        r"\[(?:distance|center_distance)=(\d+)cm\]", re.IGNORECASE
    )

    def _parse_distance_cm(output: str) -> Optional[int]:
        """Extract the first distance_cm tag embedded in the capture message."""
        m = _DISTANCE_CM_RE.search(output or "")
        return int(m.group(1)) if m else None

    class Task1CaptureRequest(BaseModel):
        # Match the field names the Mission Planner C# client sends.
        heading_deg: Optional[float] = None
        gimbal_pitch_deg: Optional[float] = None
        lidar_distance_m: Optional[float] = None
        force_crosshair: Optional[bool] = None

    @app.post("/api/task/1/target/capture", tags=["Task 1"])
    async def task1_capture_target(request: Request):
        """
        Trigger the target_localizer ROS 2 node to detect and describe targets.

        Calls the ~/capture_target service which runs HSV circle detection on
        the current ZED frame, back-projects to 3D, determines the building face,
        and generates a ConOps-compliant description.

        Returns structured metadata for Mission Planner compatibility.

        The C# client may include heading/gimbal/lidar overrides in the body —
        when present, push them into the shared state manager before triggering
        the capture so the ROS 2 node sees the operator-supplied values.
        """
        # Parse the body defensively: an empty / missing body is fine, and any
        # field omitted from the JSON simply leaves state untouched.
        try:
            raw = await request.json()
        except Exception:
            raw = None
        overrides: Optional[Task1CaptureRequest] = None
        if isinstance(raw, dict):
            try:
                overrides = Task1CaptureRequest.model_validate(raw)
            except Exception:
                overrides = None

        if overrides is not None:
            update_fields = {
                k: v
                for k, v in overrides.model_dump().items()
                if v is not None and k != "force_crosshair"
            }
            if update_fields:
                try:
                    request.app.state.state_manager.update_state(**update_fields)
                except Exception as exc:
                    logger.warning("Task1 capture override update failed: %s", exc)

        force_crosshair = bool(overrides.force_crosshair) if overrides is not None else False
        capture_options_path = os.path.join(_TARGET_SIDECAR_DIR, "capture_options.json")

        # Drone-state sidecar: target_localizer normally gets GPS/heading/alt
        # via MAVROS topics published by drone_state_publisher (running inside
        # the Isaac ROS container). If that bridge is wedged the localizer
        # has no fix even though Edge Core itself sees a valid one over
        # MAVLink, and every capture fails with "no GPS fix". Write the
        # current state here so the localizer can fall back to it.
        try:
            state_for_sidecar = request.app.state.state_manager.get_state()
            if (
                state_for_sidecar.gps_lat is not None
                and state_for_sidecar.gps_lon is not None
                and state_for_sidecar.gps_fix
            ):
                _atomic_write_json(
                    os.path.join(_TARGET_SIDECAR_DIR, "capture_drone_state.json"),
                    {
                        "lat": float(state_for_sidecar.gps_lat),
                        "lon": float(state_for_sidecar.gps_lon),
                        "gps_fix": True,
                        "heading_deg": float(state_for_sidecar.heading_deg)
                            if getattr(state_for_sidecar, "heading_deg", None) is not None
                            else None,
                        "alt_agl_m": float(state_for_sidecar.alt_agl_m)
                            if getattr(state_for_sidecar, "alt_agl_m", None) is not None
                            else None,
                        "created_at": datetime.now(timezone.utc).isoformat(),
                    },
                    indent=None,
                )
        except Exception as exc:
            logger.warning("Task1 capture drone-state sidecar write failed: %s", exc)

        if force_crosshair:
            _atomic_write_json(
                capture_options_path,
                {"force_crosshair": True, "created_at": datetime.now(timezone.utc).isoformat()},
                indent=None,
            )
        else:
            # Clear any stale one-shot option left by an interrupted forced capture.
            try:
                if os.path.exists(capture_options_path):
                    os.remove(capture_options_path)
            except OSError as exc:
                logger.warning("Task1 capture option cleanup failed: %s", exc)

        # The helper maps any `success: false` response (including the
        # application-level "no circles detected" case) to HTTPException 502,
        # which the client then shows as a scary "HTTP 502 Bad Gateway".
        # Catch that and return a structured success=False payload instead, so
        # legitimate no-detect captures don't look like gateway outages.
        try:
            try:
                output = await _call_target_capture_with_retries(
                    max_attempts=4,
                    retry_delay_s=1.5,
                    timeout_s=30.0,
                )
            finally:
                if force_crosshair:
                    try:
                        if os.path.exists(capture_options_path):
                            os.remove(capture_options_path)
                    except OSError as exc:
                        logger.warning("Task1 forced capture option cleanup failed: %s", exc)
        except HTTPException as exc:
            detail_text = str(exc.detail or "").strip()
            low = detail_text.lower()
            application_level_failure = exc.status_code == 502 and any(
                marker in low
                for marker in (
                    "no colored circles",
                    "no circles",
                    "no rgb",
                    "camera intrinsics",
                    "stream appears frozen",
                    "frame is",
                    # Crosshair-fallback diagnostics (no GPS / no depth / etc).
                    # These are real per-capture preconditions, not gateway
                    # outages, so surface them as success=False payloads.
                    "crosshair capture failed",
                    "no gps fix",
                    "no valid depth",
                    "back-projection",
                )
            )
            if application_level_failure:
                now = datetime.now(timezone.utc)
                return {
                    "success": False,
                    "error": detail_text or "Capture returned no targets.",
                    "output": detail_text,
                    "image_name": None,
                    "capture_folder": None,
                    "timestamp": now.isoformat(),
                }
            raise

        # Get current state for metadata
        state = app.state.state_manager.get_state()
        now = datetime.now(timezone.utc)

        latest_folder: Optional[str] = None
        images: list[str] = []

        # Give host storage a brief grace period for docker volume mount sync.
        # 10×100ms = 1s max; fall through to docker cp mirror if still missing.
        for _ in range(10):
            latest_folder, images = _latest_task1_capture_from_host()
            if latest_folder and images:
                break
            await asyncio.sleep(0.1)

        # If host capture path is not visible, mirror from container-local storage.
        if not latest_folder or not images:
            container_folder, container_images = _latest_task1_capture_from_container()
            if container_folder:
                # Trigger a docker cp mirror so host sees the files
                # without waiting for volume sync.
                _mirror_task1_capture_from_container(container_folder, container_images)
                # Re-check host storage after mirror
                latest_folder, images = _latest_task1_capture_from_host()
                if not latest_folder or not images:
                    latest_folder = container_folder
                    images = container_images

        distance_cm = _parse_distance_cm(output)

        def _meta(image_name=None, capture_folder=None):
            return {
                "success": True,
                "output": output,
                "image_name": image_name,
                "capture_folder": capture_folder,
                "timestamp": now.isoformat(),
                "distance_m": round(distance_cm / 100.0, 2) if distance_cm is not None else None,
                "position": {
                    "lat": state.gps_lat or 0.0,
                    "lon": state.gps_lon or 0.0,
                    "alt": state.gps_alt or 0.0,
                },
                "alt_agl_m": round(state.alt_agl_m, 2) if state.alt_agl_m is not None else None,
                "heading_deg": state.heading_deg or 0.0,
                "pitch_deg": state.pitch_deg or 0.0,
                "roll_deg": state.roll_deg or 0.0,
                "camera_pitch_deg": state.gimbal_pitch_deg or 0.0,
                "building_location": _parse_building_face(output),
            }

        if latest_folder and images:
            return _meta(image_name=images[0], capture_folder=latest_folder)

        # Fallback if no images found
        return _meta()

    @app.post("/api/task/1/target/save", tags=["Task 1"])
    async def task1_save_targets():
        """
        Save all captured targets to the competition .txt file.

        Calls the ~/save_targets service which writes Task_1_MAD_targets.txt
        and a debug log to the configured output directory.
        """
        output = _call_ros2_service_in_isaac_container_or_raise(
            service_name="/target_localizer/save_targets",
            service_type="std_srvs/srv/Trigger",
            request_payload={},
            timeout_s=10.0,
        )
        return {"success": True, "output": output}

    @app.post("/api/task/1/target/clear", tags=["Task 1"])
    async def task1_clear_targets():
        """
        Delete all captured targets from the in-memory list.

        Calls the ~/clear_targets service on the target_localizer node.
        Useful when starting a fresh run or after a false positive capture.
        """
        output = _call_ros2_service_in_isaac_container_or_raise(
            service_name="/target_localizer/clear_targets",
            service_type="std_srvs/srv/Trigger",
            request_payload={},
            timeout_s=10.0,
        )
        return {"success": True, "output": output}

    @app.delete("/api/task/1/target/{target_id}", tags=["Task 1"])
    async def task1_delete_target(target_id: str):
        """
        Delete a single target by ID (A-Z).

        Removes the target from the in-memory list, re-letters remaining targets,
        and deletes the associated capture folder on the Jetson.
        """
        target_id = target_id.strip().upper()
        if not target_id or len(target_id) != 1 or not target_id.isalpha():
            raise HTTPException(status_code=400, detail=f"Invalid target_id: {target_id!r}")
        delete_path = os.path.join(_TARGET_SIDECAR_DIR, "delete_target.json")
        _atomic_write_json(delete_path, {"target_id": target_id}, indent=None)
        output = _call_ros2_service_in_isaac_container_or_raise(
            service_name="/target_localizer/delete_target",
            service_type="std_srvs/srv/Trigger",
            request_payload={},
            timeout_s=15.0,
        )
        return {"success": True, "target_id": target_id, "output": output}

    @app.post("/api/task/1/target/ground_alt", tags=["Task 1"])
    async def task1_set_ground_alt():
        """
        Set the current drone AGL as ground level (0m reference).

        The pilot lands the drone on the ground, then calls this endpoint.
        All subsequent target heights are reported relative to this level.
        Calls the ~/set_ground_alt service on the target_localizer node.
        """
        output = _call_ros2_service_in_isaac_container_or_raise(
            service_name="/target_localizer/set_ground_alt",
            service_type="std_srvs/srv/Trigger",
            request_payload={},
            timeout_s=10.0,
        )
        return {"success": True, "output": output}

# ==================== Target sidecar files ====================
    # The building model now lives entirely on the ground station. The Jetson
    # only keeps a small sidecar directory for per-target action files
    # (delete_target.json, plane_override.json) that target_localizer reads
    # opportunistically.
    _TARGET_SIDECAR_DIR = os.environ.get(
        "NOMAD_TARGET_OUTPUT_DIR",
        os.path.expanduser("~/NOMAD/config"),
    )

    def _atomic_write_json(path: str, data, *, indent: Optional[int] = 2) -> None:
        """
        Crash-safe JSON write: serialize to a sibling .tmp, fsync, then
        os.replace() onto the target. A power loss or kill during the write
        can only ever leave a stale .tmp — the live file is never half-written.
        """
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        tmp = f"{path}.tmp"
        with open(tmp, "w") as f:
            if indent is None:
                json.dump(data, f)
            else:
                json.dump(data, f, indent=indent)
            f.flush()
            try:
                os.fsync(f.fileno())
            except OSError:
                pass
        os.replace(tmp, path)

    # NOTE: /api/task/1/target/{id}/plane_override and /api/task/1/target/regenerate
    # used to live here. They depended on the Jetson-side building model to
    # reclassify face/plane and rewrite the spatial description. The ground
    # station now owns the building model and rewrites descriptions locally
    # whenever the operator edits the Plane cell, so these endpoints have no
    # job left on the Jetson and were removed along with the building APIs.

    @app.post("/api/task/1/target/detection_status/update", tags=["Task 1"])
    async def task1_detection_status_push(request: Request):
        """Internal: target_localizer_node pushes detection status here every ~0.5s."""
        try:
            data = await request.json()
        except Exception:
            raise HTTPException(status_code=400, detail="Invalid JSON body")
        app.state.task1_det_cache = {"success": True, **data}
        app.state.task1_det_cache_ts = time.time()
        return {"ok": True}

    @app.get("/api/task/1/target/detections", tags=["Task 1"])
    async def task1_detection_status():
        """
        Get the current realtime HSV circle detection status.

        Updated every ~0.5s by target_localizer_node via HTTP push.
        The cache holds circle count, per-circle distances, and crosshair depth.
        """
        cache = getattr(app.state, "task1_det_cache", None) or {"circle_count": 0}
        cache_age = time.time() - getattr(app.state, "task1_det_cache_ts", 0.0)
        return {**cache, "cache_age_s": round(cache_age, 1)}

    @app.get("/api/task/1/target/list", tags=["Task 1"])
    async def task1_list_targets():
        """
        List all captured targets with their descriptions, approval status,
        and warnings (e.g., above building height).

        This endpoint reads the target list from the target_localizer node
        via the print_model service, plus the target file if saved.
        Returns structured data for the Mission Planner submission table.
        """
        try:
            output = _call_ros2_service_in_isaac_container_or_raise(
                service_name="/target_localizer/save_targets",
                service_type="std_srvs/srv/Trigger",
                request_payload={},
                timeout_s=10.0,
            )
            return {"success": True, "output": output}
        except HTTPException:
            return {"success": True, "targets": [], "message": "Target localizer service unavailable"}

    @app.get("/api/task/1/target/list_structured", tags=["Task 1"])
    async def task1_list_targets_structured():
        """
        Return captured targets as structured JSON for the 3D building viewer.

        The target_localizer node's save_targets service writes a debug file
        with absolute GPS coordinates. We parse that file and surface
        ``[{id, color, lat, lon, height_agl, image, timestamp}]`` so the GCS
        can place markers on the 3D building model (and convert lat/lon to
        its own building-relative ENU frame using the locally stored corners).
        """
        team_name = os.environ.get("NOMAD_TEAM_NAME", "MAD")
        output_dir = os.environ.get(
            "NOMAD_TARGET_OUTPUT_DIR_NODE",
            os.path.expanduser("~/targets"),
        )
        debug_path = os.path.join(output_dir, f"Task_1_{team_name}_targets_debug.txt")

        # Best-effort refresh: trigger a save so the file reflects the latest
        # captures. Failure is non-fatal — we still try to read whatever's on
        # disk so the viewer keeps working when the ROS service is down.
        try:
            _call_ros2_service_in_isaac_container_or_raise(
                service_name="/target_localizer/save_targets",
                service_type="std_srvs/srv/Trigger",
                request_payload={},
                timeout_s=5.0,
            )
        except Exception:
            pass

        if not os.path.isfile(debug_path):
            return {"success": True, "targets": [], "source": debug_path, "missing": True}

        targets: list[dict[str, Any]] = []
        try:
            with open(debug_path, "r") as f:
                content = f.read()
        except OSError as e:
            return {"success": False, "targets": [], "error": str(e)}

        # Parse the per-target blocks the node writes:
        #   Target A:
        #     Color: red
        #     GPS: (45.3167432, -75.7577383)
        #     Height AGL: 2.10m
        #     ...
        import re as _re
        for block in _re.split(r"\n(?=Target\s)", content):
            m_id = _re.match(r"Target\s+([^:]+):", block)
            if not m_id:
                continue
            tid = m_id.group(1).strip()
            entry: dict[str, Any] = {"id": tid}

            for key, pattern in (
                ("color", r"Color:\s*(\S+)"),
                ("image", r"Image:\s*(.+)"),
                ("timestamp", r"Timestamp:\s*(.+)"),
            ):
                m = _re.search(pattern, block)
                if m:
                    entry[key] = m.group(1).strip()

            m_height = _re.search(r"Height AGL:\s*([-\d.]+)", block)
            if m_height:
                try:
                    entry["height_agl"] = float(m_height.group(1))
                except ValueError:
                    pass

            m_gps = _re.search(
                r"GPS:\s*\(\s*([-\d.]+)\s*,\s*([-\d.]+)\s*\)",
                block,
            )
            if m_gps:
                try:
                    entry["lat"] = float(m_gps.group(1))
                    entry["lon"] = float(m_gps.group(2))
                except ValueError:
                    pass

            targets.append(entry)

        return {"success": True, "targets": targets, "count": len(targets), "source": debug_path}

    # NOTE: All /api/task/1/building/* endpoints and /api/task/1/target/model
    # were removed when the building model moved to the ground station.
    # The GCS persists corners under %MyDocuments%/NOMAD/Task1/building_corners.json
    # and never asks the Jetson about building geometry anymore.



