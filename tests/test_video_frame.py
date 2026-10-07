# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Tests for the ROS image to RGB frame conversion rules."""

from __future__ import annotations

import pytest
from video_bridge_fakes import image_msg

from python.tools.video_frame import FrameConversionError, image_to_rgb


@pytest.mark.parametrize(
    ("encoding", "data", "expected"),
    [
        ("rgb8", bytes([1, 2, 3]), bytes([1, 2, 3])),
        ("bgr8", bytes([1, 2, 3]), bytes([3, 2, 1])),
        ("rgba8", bytes([1, 2, 3, 4]), bytes([1, 2, 3])),
        ("bgra8", bytes([1, 2, 3, 4]), bytes([3, 2, 1])),
        ("mono8", bytes([7]), bytes([7, 7, 7])),
    ],
)
def test_image_to_rgb_encodings(encoding, data, expected):
    msg = image_msg(encoding=encoding, width=1, height=1, step=len(data), data=data)
    assert image_to_rgb(msg, 1, 1).tobytes() == expected


@pytest.mark.parametrize("encoding", ["RGB8", "BGR8"])
def test_image_to_rgb_accepts_upper_case_encoding(encoding):
    msg = image_msg(encoding=encoding, width=1, height=1, step=3, data=bytes([1, 2, 3]))
    assert image_to_rgb(msg, 1, 1).shape == (1, 1, 3)


def test_image_to_rgb_resizes_with_nearest_neighbor():
    msg = image_msg(
        encoding="rgb8",
        width=2,
        height=2,
        step=6,
        data=bytes([1, 0, 0, 2, 0, 0, 3, 0, 0, 4, 0, 0]),
    )
    assert image_to_rgb(msg, 1, 1).shape == (1, 1, 3)


def test_image_to_rgb_keeps_requested_size():
    msg = image_msg(encoding="mono8", width=2, height=2, step=2, data=bytes([1, 2, 3, 4]))
    assert image_to_rgb(msg, 4, 4).shape == (4, 4, 3)


def test_image_to_rgb_rejects_unknown_encoding():
    msg = image_msg(encoding="yuyv", width=1, height=1, step=2, data=b"\0\0")
    with pytest.raises(FrameConversionError, match="Unsupported image encoding"):
        image_to_rgb(msg, 1, 1)


def test_image_to_rgb_rejects_short_buffer():
    msg = image_msg(encoding="rgb8", width=2, height=2, step=6, data=bytes(6))
    with pytest.raises(FrameConversionError, match="Image buffer mismatch"):
        image_to_rgb(msg, 2, 2)


def test_image_to_rgb_rejects_row_step_smaller_than_row():
    msg = image_msg(encoding="rgb8", width=2, height=1, step=3, data=bytes(6))
    with pytest.raises(FrameConversionError, match="Image buffer mismatch"):
        image_to_rgb(msg, 2, 1)


def test_image_to_rgb_drops_row_padding():
    msg = image_msg(encoding="rgb8", width=1, height=2, step=5, data=bytes([1, 2, 3, 9, 9, 4, 5, 6, 9, 9]))
    assert image_to_rgb(msg, 1, 2).tobytes() == bytes([1, 2, 3, 4, 5, 6])
