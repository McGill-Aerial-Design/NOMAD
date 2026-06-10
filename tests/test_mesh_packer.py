# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Tests for edge_core.ros_http_bridge.mesh_packer.VoxelMeshPacker.

The background sender thread is stopped up front so the queueing/coalescing logic
can be exercised deterministically.
"""

from __future__ import annotations

import pytest

from edge_core.ros_http_bridge.mesh_packer import VoxelMeshPacker


class _FakeNode:
    """Minimal stand-in for the bridge node the packer writes stats onto."""

    def __init__(self) -> None:
        self._mesh_coalesced_count = 0
        self._mesh_send_count = 0
        self._send_errors = 0
        self._last_mesh_send_time = 0.0
        self.posts: list[tuple[str, bytes]] = []

    def _http_post(self, path, data, timeout=2.0, content_type=""):  # pragma: no cover - not hit (thread stopped)
        self.posts.append((path, data))
        return True

    def _bump_send_errors(self) -> None:  # pragma: no cover
        self._send_errors += 1


@pytest.fixture
def packer():
    node = _FakeNode()
    p = VoxelMeshPacker(node)
    p.stop()  # halt the background sender so state is inspectable
    yield p, node


def test_queue_records_pending_and_meta(packer):
    p, _node = packer
    p.queue_mesh({"mode": "voxel", "total_voxels": 42, "blocks": []})
    assert p._mesh_pending_dict is not None
    assert p._mesh_pending_meta == ("voxel", 42)


def test_queue_coalesces_when_unsent(packer):
    p, node = packer
    p.queue_mesh({"mode": "voxel", "total_voxels": 1})
    p.queue_mesh({"mode": "voxel", "total_voxels": 2})
    p.queue_mesh({"mode": "voxel", "total_voxels": 3})
    # Two of the three were coalesced into the still-pending slot.
    assert node._mesh_coalesced_count == 2
    assert p._mesh_pending_meta == ("voxel", 3)


def test_meta_defaults_when_keys_absent(packer):
    p, _node = packer
    p.queue_mesh({})
    assert p._mesh_pending_meta == ("voxel", 0)


def test_stop_is_idempotent(packer):
    p, _node = packer
    p.stop()  # already stopped in fixture; must not raise
    assert not p._mesh_sender_thread.is_alive()
