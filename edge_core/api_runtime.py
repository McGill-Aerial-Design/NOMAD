"""Shared Edge Core API runtime state and high-rate IPC wiring."""

from __future__ import annotations

import math
import os
import threading
import time
from datetime import datetime, timezone
from typing import Any, Optional

from .api_models import NavVelocityRequest, VIOUpdateRequest

try:
    from .ipc import (
        DEFAULT_ROS_HIGH_RATE_ENDPOINT,
        HIGH_RATE_MSG_TYPE_CMD_VEL,
        HIGH_RATE_MSG_TYPE_DETECTIONS,
        HIGH_RATE_MSG_TYPE_VIO,
        IPCMessage,
        ZMQSubscriber,
    )

    IPC_AVAILABLE = True
    IPC_IMPORT_ERROR = ""
except Exception as exc:  # pragma: no cover - optional runtime dependency
    IPC_AVAILABLE = False
    IPC_IMPORT_ERROR = str(exc)
    DEFAULT_ROS_HIGH_RATE_ENDPOINT = "tcp://127.0.0.1:5557"
    HIGH_RATE_MSG_TYPE_VIO = "ROS_VIO_UPDATE"
    HIGH_RATE_MSG_TYPE_CMD_VEL = "ROS_CMD_VEL"
    HIGH_RATE_MSG_TYPE_DETECTIONS = "ROS_DETECTIONS"
    IPCMessage = Any  # type: ignore
    ZMQSubscriber = Any  # type: ignore


class EdgeApiRuntime:
    """Owns high-rate API state that is shared by routes and IPC listeners."""

    def __init__(self, app: Any, state_manager: Any, logger: Any) -> None:
        self.app = app
        self.state_manager = state_manager
        self.logger = logger
        self.configure_state()

    def configure_state(self) -> None:
        """Initialize runtime knobs and locks on app.state."""
        app_state = self.app.state
        app_state.detection_last_source_timestamp = None
        app_state.vio_last_timestamp_by_source: dict[str, float] = {}
        app_state.vio_last_receive_monotonic_by_source: dict[str, float] = {}
        app_state.vio_max_age_s = self._read_positive_float("NOMAD_VIO_MAX_AGE_S", 1.0)

        app_state.high_rate_zmq_enabled = os.environ.get(
            "NOMAD_HIGH_RATE_ZMQ_ENABLED", "1"
        ).strip().lower() not in ("0", "false", "no")
        app_state.high_rate_zmq_sub_mode = (
            os.environ.get("NOMAD_HIGH_RATE_ZMQ_SUB_MODE", "bind").strip().lower()
        )
        if app_state.high_rate_zmq_sub_mode not in ("bind", "connect"):
            self.logger.warning(
                "Invalid NOMAD_HIGH_RATE_ZMQ_SUB_MODE='%s'; falling back to 'bind'",
                app_state.high_rate_zmq_sub_mode,
            )
            app_state.high_rate_zmq_sub_mode = "bind"

        configured_endpoint = os.environ.get("NOMAD_HIGH_RATE_ZMQ_ENDPOINT", "").strip()
        if configured_endpoint:
            app_state.high_rate_zmq_endpoint = configured_endpoint
        elif app_state.high_rate_zmq_sub_mode == "bind":
            try:
                scheme, endpoint_rest = DEFAULT_ROS_HIGH_RATE_ENDPOINT.split("://", 1)
                _, default_port = endpoint_rest.rsplit(":", 1)
                app_state.high_rate_zmq_endpoint = f"{scheme}://0.0.0.0:{default_port}"
            except Exception:
                app_state.high_rate_zmq_endpoint = "tcp://0.0.0.0:5557"
        else:
            app_state.high_rate_zmq_endpoint = DEFAULT_ROS_HIGH_RATE_ENDPOINT

        app_state.high_rate_zmq_stop_event = threading.Event()
        app_state.high_rate_zmq_thread = None
        app_state.high_rate_zmq_warn_interval_s = 2.0
        app_state.high_rate_zmq_last_warn: dict[str, float] = {}
        app_state.nav_cmd_vel_max_age_s = self._read_positive_float(
            "NOMAD_CMD_VEL_MAX_AGE_S", 0.5
        )
        app_state.nav_cmd_vel_last_timestamp_by_source: dict[str, float] = {}
        app_state.nav_cmd_vel_order_lock = threading.Lock()

    def _read_positive_float(self, env_name: str, default: float) -> float:
        raw = os.environ.get(env_name, str(default)).strip()
        try:
            value = float(raw)
            if value <= 0.0:
                raise ValueError("value must be positive")
            return value
        except Exception:
            self.logger.warning(
                "Invalid %s='%s'; falling back to %.2fs",
                env_name,
                raw,
                default,
            )
            return default

    def high_rate_warn(self, key: str, message: str) -> None:
        """Throttle repeated high-rate ZMQ warning logs."""
        now = time.time()
        last = self.app.state.high_rate_zmq_last_warn.get(key, 0.0)
        if now - last >= self.app.state.high_rate_zmq_warn_interval_s:
            self.logger.warning(message)
            self.app.state.high_rate_zmq_last_warn[key] = now

    def apply_vio_update_from_request(self, vio_request: VIOUpdateRequest) -> int:
        """Apply VIO update to shared app state and return trajectory length."""
        app_state = self.app.state
        receive_monotonic = time.monotonic()
        receive_iso = datetime.now(timezone.utc).isoformat()
        with app_state.vio_state_lock:
            source = (vio_request.source or "external").strip() or "external"
            last_timestamp = app_state.vio_last_timestamp_by_source.get(source)
            if last_timestamp is not None and vio_request.timestamp <= last_timestamp:
                return len(app_state.vio_trajectory)
            app_state.vio_last_timestamp_by_source[source] = vio_request.timestamp
            app_state.vio_last_receive_monotonic_by_source[source] = receive_monotonic

            app_state.external_vio_state = {
                "timestamp": vio_request.timestamp,
                "received_at": receive_iso,
                "receive_monotonic": receive_monotonic,
                "x": vio_request.x,
                "y": vio_request.y,
                "z": vio_request.z,
                "roll": vio_request.roll,
                "pitch": vio_request.pitch,
                "yaw": vio_request.yaw,
                "vx": vio_request.vx,
                "vy": vio_request.vy,
                "vz": vio_request.vz,
                "confidence": vio_request.confidence,
                "source": source,
            }

            self.state_manager.update_state(
                vio_x=vio_request.x,
                vio_y=vio_request.y,
                vio_z=vio_request.z,
                vio_yaw=vio_request.yaw,
                vio_confidence=vio_request.confidence,
            )
            nav_controller = getattr(app_state, "nav_controller", None)
            if nav_controller and hasattr(nav_controller, "set_vio_state"):
                try:
                    nav_controller.set_vio_state(
                        confidence=vio_request.confidence,
                        healthy=bool(vio_request.confidence > 0.5),
                        received_monotonic=receive_monotonic,
                    )
                except TypeError:
                    nav_controller.set_vio_state(
                        confidence=vio_request.confidence,
                        healthy=bool(vio_request.confidence > 0.5),
                    )
                except Exception as exc:
                    self.high_rate_warn(
                        "nav_vio_state_update_failed",
                        f"Failed to update NavController VIO health: {exc}",
                    )

            app_state.slam_vio_ros_frame = {
                "x": vio_request.ros_x,
                "y": vio_request.ros_y,
                "z": vio_request.ros_z,
                "roll": vio_request.ros_roll,
                "pitch": vio_request.ros_pitch,
                "yaw": vio_request.ros_yaw,
                "body_roll": vio_request.body_roll,
                "body_pitch": vio_request.body_pitch,
                "body_yaw": vio_request.body_yaw,
                "timestamp": vio_request.timestamp,
                "received_at": receive_iso,
                "receive_monotonic": receive_monotonic,
                "frame_id": getattr(vio_request, "frame_id", "map"),
            }

            app_state.vio_trajectory.append(
                {
                    "x": vio_request.x,
                    "y": vio_request.y,
                    "z": vio_request.z,
                    "timestamp": vio_request.timestamp,
                }
            )
            if len(app_state.vio_trajectory) > app_state.vio_trajectory_max_points:
                app_state.vio_trajectory = app_state.vio_trajectory[
                    -app_state.vio_trajectory_max_points :
                ]

            return len(app_state.vio_trajectory)

    def get_vio_snapshot(self, include_trajectory: bool = False) -> dict[str, Any]:
        """Read VIO state under one lock and annotate freshness."""
        app_state = self.app.state
        now = time.monotonic()
        max_age_s = float(getattr(app_state, "vio_max_age_s", 1.0))
        with app_state.vio_state_lock:
            external_vio_state = (
                dict(app_state.external_vio_state)
                if app_state.external_vio_state
                else None
            )
            slam_vio_ros_frame = (
                dict(app_state.slam_vio_ros_frame)
                if app_state.slam_vio_ros_frame
                else None
            )
            vio_trajectory = list(app_state.vio_trajectory) if include_trajectory else None

        for snapshot in (external_vio_state, slam_vio_ros_frame):
            if not snapshot:
                continue
            received = snapshot.get("receive_monotonic")
            age_s = None
            fresh = False
            try:
                age_s = max(0.0, now - float(received))
                fresh = age_s <= max_age_s
            except (TypeError, ValueError):
                pass
            snapshot["age_seconds"] = age_s
            snapshot["fresh"] = fresh
            snapshot["max_age_seconds"] = max_age_s

        return {
            "external_vio_state": external_vio_state,
            "slam_vio_ros_frame": slam_vio_ros_frame,
            "vio_trajectory": vio_trajectory,
        }

    def dispatch_nav_velocity(self, nav_request: NavVelocityRequest) -> bool:
        """Forward velocity command to NavController using existing API semantics."""
        nav_controller = self.app.state.nav_controller
        if not nav_controller:
            raise RuntimeError("Navigation controller not initialized")

        source = (nav_request.source or "nav2").strip() or "nav2"
        cmd_timestamp = float(nav_request.timestamp)
        if not math.isfinite(cmd_timestamp):
            raise ValueError("Rejected cmd_vel with non-finite timestamp")

        now = time.time()
        max_age_s = self.app.state.nav_cmd_vel_max_age_s
        age_s = now - cmd_timestamp
        if age_s > max_age_s:
            raise ValueError(
                f"Rejected stale cmd_vel from source '{source}': "
                f"age={age_s:.3f}s exceeds max_age={max_age_s:.3f}s"
            )
        if cmd_timestamp > now + max_age_s:
            raise ValueError(
                f"Rejected cmd_vel from source '{source}': "
                f"timestamp is too far in the future (max_skew={max_age_s:.3f}s)"
            )

        with self.app.state.nav_cmd_vel_order_lock:
            last_timestamp = self.app.state.nav_cmd_vel_last_timestamp_by_source.get(source)
            if last_timestamp is not None and cmd_timestamp <= last_timestamp:
                raise ValueError(
                    f"Rejected non-monotonic cmd_vel from source '{source}': "
                    f"timestamp={cmd_timestamp:.6f} <= last_timestamp={last_timestamp:.6f}"
                )

            accepted = nav_controller.send_velocity(
                vx=nav_request.vx,
                vy=nav_request.vy,
                vz=nav_request.vz,
                yaw_rate=nav_request.yaw_rate,
                source=source,
            )
            if accepted:
                self.app.state.nav_cmd_vel_last_timestamp_by_source[source] = cmd_timestamp

            return accepted

    def apply_detections_update(
        self, detections: list, source_timestamp: Optional[Any] = None
    ) -> None:
        """Apply detection update to app state."""
        app_state = self.app.state
        with app_state.detection_state_lock:
            normalized_source_timestamp: Optional[float] = None
            if source_timestamp is not None:
                try:
                    normalized_source_timestamp = float(source_timestamp)
                    if not math.isfinite(normalized_source_timestamp):
                        normalized_source_timestamp = None
                except (TypeError, ValueError):
                    normalized_source_timestamp = None

            if normalized_source_timestamp is not None:
                last_source_timestamp = app_state.detection_last_source_timestamp
                if last_source_timestamp is not None and math.isclose(
                    normalized_source_timestamp,
                    last_source_timestamp,
                    rel_tol=0.0,
                    abs_tol=1e-6,
                ):
                    return
                app_state.detection_last_source_timestamp = normalized_source_timestamp

            dict_detections = [det for det in detections if isinstance(det, dict)] if isinstance(detections, list) else []
            app_state.detected_objects = dict_detections
            app_state.detection_last_update = time.time()

            history = app_state.detection_history
            for det in dict_detections:
                if det.get("image_only"):
                    continue
                x_val = det.get("x")
                y_val = det.get("y")
                z_val = det.get("z")
                if x_val is None or y_val is None or z_val is None:
                    continue
                try:
                    if not (
                        isinstance(x_val, (int, float))
                        and isinstance(y_val, (int, float))
                        and isinstance(z_val, (int, float))
                    ):
                        continue
                    if not (
                        math.isfinite(x_val)
                        and math.isfinite(y_val)
                        and math.isfinite(z_val)
                    ):
                        continue
                except (TypeError, ValueError):
                    continue

                is_duplicate = False
                for existing in history:
                    dx = x_val - existing["x"]
                    dy = y_val - existing["y"]
                    dz = z_val - existing["z"]
                    dist = (dx * dx + dy * dy + dz * dz) ** 0.5
                    if dist < 0.5 and det.get("label") == existing.get("label"):
                        existing["seen_count"] = existing.get("seen_count", 1) + 1
                        if det.get("confidence", 0) > existing.get("confidence", 0):
                            existing.update(det)
                            existing["seen_count"] = existing.get("seen_count", 1)
                        is_duplicate = True
                        break

                if not is_duplicate:
                    det["seen_count"] = 1
                    det["first_seen"] = time.time()
                    history.append(det)
                    if len(history) > app_state.detection_history_max:
                        history.pop(0)

    def handle_high_rate_ipc_message(self, message: IPCMessage) -> None:
        """Handle a single high-rate IPC message from ros_http_bridge."""
        if message.msg_type == HIGH_RATE_MSG_TYPE_VIO:
            try:
                vio_request = VIOUpdateRequest(**message.data)
            except Exception as exc:
                self.high_rate_warn("vio-parse", f"Invalid high-rate VIO payload: {exc}")
                return
            self.apply_vio_update_from_request(vio_request)
            return

        if message.msg_type == HIGH_RATE_MSG_TYPE_CMD_VEL:
            try:
                nav_request = NavVelocityRequest(**message.data)
            except Exception as exc:
                self.high_rate_warn("cmd-parse", f"Invalid high-rate cmd_vel payload: {exc}")
                return
            try:
                self.dispatch_nav_velocity(nav_request)
            except ValueError as exc:
                self.high_rate_warn("cmd-gate", f"Dropping high-rate cmd_vel: {exc}")
            except RuntimeError as exc:
                self.high_rate_warn("cmd-nav", f"Ignoring high-rate cmd_vel: {exc}")
            except Exception as exc:
                self.high_rate_warn("cmd-send", f"High-rate cmd_vel dispatch failed: {exc}")
            return

        if message.msg_type == HIGH_RATE_MSG_TYPE_DETECTIONS:
            try:
                detections = message.data.get("detections", [])
                source_timestamp = message.data.get("source_timestamp")
                self.apply_detections_update(detections, source_timestamp=source_timestamp)
            except Exception as exc:
                self.high_rate_warn("det-zmq", f"High-rate detection update failed: {exc}")

    def _high_rate_zmq_listener_loop(self, stop_event: threading.Event) -> None:
        if not IPC_AVAILABLE:
            self.logger.warning(
                f"High-rate ZMQ listener disabled: IPC unavailable ({IPC_IMPORT_ERROR})"
            )
            return

        endpoint = self.app.state.high_rate_zmq_endpoint
        socket_mode = self.app.state.high_rate_zmq_sub_mode
        self.logger.info(f"High-rate ZMQ listener starting on {endpoint} ({socket_mode})")

        subscriber: Optional[ZMQSubscriber] = None
        try:
            while not stop_event.is_set():
                try:
                    if subscriber is None:
                        subscriber = ZMQSubscriber(
                            endpoint=endpoint,
                            timeout_ms=500,
                            socket_mode=socket_mode,
                            rcv_hwm=1,
                            conflate=True,
                            linger_ms=0,
                        )
                        subscriber.start()
                        self.logger.info(
                            f"High-rate ZMQ listener ready on {endpoint} ({socket_mode})"
                        )

                    message = subscriber.receive()
                    if message is None:
                        continue
                    self.handle_high_rate_ipc_message(message)
                except Exception as exc:
                    self.high_rate_warn(
                        "listener-loop",
                        f"High-rate ZMQ listener error on {endpoint}: {exc}",
                    )
                    if subscriber is not None:
                        try:
                            subscriber.stop()
                        except Exception:
                            pass
                        subscriber = None
                    if stop_event.wait(1.0):
                        break
        finally:
            if subscriber is not None:
                try:
                    subscriber.stop()
                except Exception:
                    pass
            self.logger.info("High-rate ZMQ listener stopped")

    def start_high_rate_zmq_listener(self) -> None:
        if not self.app.state.high_rate_zmq_enabled:
            self.logger.info("High-rate ZMQ listener disabled by NOMAD_HIGH_RATE_ZMQ_ENABLED")
            return

        thread = self.app.state.high_rate_zmq_thread
        if thread and thread.is_alive():
            return

        stop_event = self.app.state.high_rate_zmq_stop_event
        stop_event.clear()
        thread = threading.Thread(
            target=self._high_rate_zmq_listener_loop,
            args=(stop_event,),
            name="high-rate-zmq-listener",
            daemon=True,
        )
        self.app.state.high_rate_zmq_thread = thread
        thread.start()

    def stop_high_rate_zmq_listener(self) -> None:
        self.app.state.high_rate_zmq_stop_event.set()
        thread = self.app.state.high_rate_zmq_thread
        if thread and thread.is_alive():
            thread.join(timeout=2.0)
        self.app.state.high_rate_zmq_thread = None
