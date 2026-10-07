# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Convert ROS sensor_msgs/Image payloads into right-sized RGB frames.

Pure numpy helpers with no ROS, GStreamer or HTTP dependency, so the encoding
rules can be tested and read on their own.
"""

from __future__ import annotations

import numpy as np

__all__ = ["FrameConversionError", "image_to_rgb"]

_ENCODING_CHANNELS = {
    "rgb8": 3,
    "bgr8": 3,
    "rgba8": 4,
    "bgra8": 4,
    "mono8": 1,
}


class FrameConversionError(ValueError):
    """An image message cannot be converted into an RGB frame."""


def _channels_for(encoding: str) -> int:
    return _ENCODING_CHANNELS.get(encoding, 0)


def _unpack_rows(buf, width: int, height: int, row_bytes: int, channels: int):
    """Drop row padding, or return None when the buffer cannot hold the image."""
    expected = height * row_bytes
    if row_bytes < width * channels or buf.size < expected:
        return None
    raw = buf[:expected].reshape((height, row_bytes))
    return raw[:, : width * channels].reshape((height, width, channels))


def _drop_alpha_or_swap(packed, encoding: str):
    if encoding == "rgb8":
        return packed
    if encoding == "bgr8":
        return packed[:, :, ::-1]
    if encoding == "rgba8":
        return packed[:, :, :3]
    if encoding == "bgra8":
        return packed[:, :, [2, 1, 0]]
    return np.repeat(packed, 3, axis=2)  # mono8


def _resize_nearest(rgb, src_w: int, src_h: int, target_w: int, target_h: int):
    y_index = np.linspace(0, src_h - 1, target_h).astype(np.intp)
    x_index = np.linspace(0, src_w - 1, target_w).astype(np.intp)
    return rgb[y_index][:, x_index]


def image_to_rgb(msg, target_width: int, target_height: int):
    """Return an RGB frame of the requested size.

    Raises FrameConversionError naming the observed values when the encoding is
    unsupported or the buffer is shorter than the declared image.
    """
    encoding = (msg.encoding or "").lower()
    channels = _channels_for(encoding)
    if channels == 0:
        raise FrameConversionError(f"Unsupported image encoding: {msg.encoding}")

    row_bytes, width, height = int(msg.step), int(msg.width), int(msg.height)
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    packed = _unpack_rows(buf, width, height, row_bytes, channels)
    if packed is None:
        raise FrameConversionError(
            f"Image buffer mismatch (encoding={encoding}, step={row_bytes}, w={width}, h={height}, "
            f"bytes={buf.size}); dropping frame"
        )

    rgb = _drop_alpha_or_swap(packed, encoding)
    if width != target_width or height != target_height:
        rgb = _resize_nearest(rgb, width, height, target_width, target_height)
    return np.ascontiguousarray(rgb)
