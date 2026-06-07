# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Voxel mesh extraction, packaging, and background serialization for ROS-HTTP Bridge."""

from __future__ import annotations

import logging
import threading
import time

logger = logging.getLogger("ros_http_bridge.mesh_packer")

try:
    import msgpack

    MSGPACK_AVAILABLE = True
except ImportError:
    msgpack = None
    MSGPACK_AVAILABLE = False


class VoxelMeshPacker:
    """Handles voxel marker conversion, msgpack/JSON serialization, and background posting."""

    def __init__(self, bridge_node) -> None:
        self.node = bridge_node
        self._mesh_pending_dict: dict | None = None
        self._mesh_pending_meta: tuple[str, int] = ("voxel", 0)
        self._mesh_pending_lock = threading.Lock()
        self._mesh_send_event = threading.Event()
        self._mesh_sender_stop = threading.Event()

        self._mesh_sender_thread = threading.Thread(target=self._mesh_sender_loop, daemon=True, name="mesh-sender")
        self._mesh_sender_thread.start()

    def stop(self) -> None:
        """Stop the background sender thread."""
        self._mesh_sender_stop.set()
        self._mesh_send_event.set()
        if self._mesh_sender_thread.is_alive():
            self._mesh_sender_thread.join(timeout=3.0)

    def queue_mesh(self, mesh_data: dict) -> None:
        """Queue a mesh payload for background serialization and transmission."""
        with self._mesh_pending_lock:
            if self._mesh_pending_dict is not None:
                self.node._mesh_coalesced_count += 1
            self._mesh_pending_dict = mesh_data
            self._mesh_pending_meta = (mesh_data.get("mode", "voxel"), mesh_data.get("total_voxels", 0))
        self._mesh_send_event.set()

    def _mesh_sender_loop(self) -> None:
        """Background thread that serializes and transmits queued mesh payloads."""
        import json

        while not self._mesh_sender_stop.is_set():
            self._mesh_send_event.wait(timeout=1.0)
            if self._mesh_sender_stop.is_set():
                break
            self._mesh_send_event.clear()

            with self._mesh_pending_lock:
                mesh_dict = self._mesh_pending_dict
                meta = self._mesh_pending_meta
                self._mesh_pending_dict = None

            if mesh_dict is None:
                continue

            # Serialize payload on the background thread
            try:
                if MSGPACK_AVAILABLE:
                    data = msgpack.packb(mesh_dict, use_bin_type=True)
                    ctype = "application/x-msgpack"
                else:
                    data = json.dumps(mesh_dict).encode("utf-8")
                    ctype = "application/json"
            except Exception as e:
                self.node._bump_send_errors()
                logger.error(f"Mesh serialization failed: {e}")
                continue

            # Transmit (still inside the loop so every queued mesh is sent)
            try:
                if self.node._http_post("/api/slam/mesh/update", data, timeout=2.0, content_type=ctype):
                    self.node._mesh_send_count += 1
                    self.node._last_mesh_send_time = time.monotonic()
                    if self.node._mesh_send_count % 10 == 1:
                        mode, count = meta
                        logger.info(f"Mesh sent: {count} voxels (mode={mode})")
                else:
                    self.node._bump_send_errors()
            except Exception as e:
                self.node._bump_send_errors()
                if self.node._send_errors % 50 == 1:
                    logger.warning(f"Failed to transmit mesh payload: {e}")
