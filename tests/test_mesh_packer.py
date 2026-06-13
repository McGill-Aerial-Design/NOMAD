# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Tests for edge_core.ros_http_bridge.mesh_packer.VoxelMeshPacker.

The background sender thread is stopped up front so the queueing/coalescing logic
can be exercised deterministically.
"""

from __future__ import annotations

import threading

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


# -- background sender loop (real thread, synchronized via an Event) ---------


class _SignalNode:
    """Node whose hooks set an Event so the sender loop can be awaited."""

    def __init__(self, *, post_result=True, post_raises=False):
        self._mesh_coalesced_count = 0
        self._mesh_send_count = 0
        self._send_errors = 0
        self._last_mesh_send_time = 0.0
        self._post_result = post_result
        self._post_raises = post_raises
        self.posted = threading.Event()
        self.errored = threading.Event()
        self.last_post: tuple | None = None

    def _http_post(self, path, data, timeout=2.0, content_type=""):
        self.last_post = (path, content_type, data)
        if self._post_raises:
            raise RuntimeError("transmit blew up")
        if self._post_result:
            self.posted.set()
        return self._post_result

    def _bump_send_errors(self):
        self._send_errors += 1
        self.errored.set()


def test_sender_serializes_and_transmits_queued_mesh():
    node = _SignalNode()
    p = VoxelMeshPacker(node)  # real sender thread runs
    try:
        p.queue_mesh({"mode": "voxel", "total_voxels": 3, "blocks": [1, 2, 3]})
        assert node.posted.wait(3.0), "queued mesh was never transmitted"
        path, ctype, _data = node.last_post
        assert path == "/api/slam/mesh/update"
        assert ctype in ("application/json", "application/x-msgpack")
        assert node._mesh_send_count == 1
        assert node._last_mesh_send_time > 0
    finally:
        p.stop()


def test_sender_counts_serialization_failure():
    node = _SignalNode()
    p = VoxelMeshPacker(node)
    try:
        # A set is serializable by neither json nor msgpack -> serialization raises.
        p.queue_mesh({"mode": "voxel", "total_voxels": 1, "blocks": {1, 2, 3}})
        assert node.errored.wait(3.0), "serialization failure was not recorded"
        assert node._send_errors >= 1
        assert node._mesh_send_count == 0
    finally:
        p.stop()


def test_sender_counts_transmit_rejection():
    node = _SignalNode(post_result=False)
    p = VoxelMeshPacker(node)
    try:
        p.queue_mesh({"mode": "voxel", "total_voxels": 2})
        assert node.errored.wait(3.0), "transmit rejection was not recorded"
        assert node._send_errors >= 1
    finally:
        p.stop()


def test_sender_counts_transmit_exception():
    node = _SignalNode(post_raises=True)
    p = VoxelMeshPacker(node)
    try:
        p.queue_mesh({"mode": "voxel", "total_voxels": 9})
        assert node.errored.wait(3.0), "transmit exception was not recorded"
        assert node._send_errors >= 1
    finally:
        p.stop()
