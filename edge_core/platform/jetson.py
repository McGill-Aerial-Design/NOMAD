# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Jetson-specific startup helpers."""

from __future__ import annotations

import ctypes
import os
from pathlib import Path


def preload_local_libraries() -> None:
    """Make user-local native libraries discoverable before hardware imports."""
    local_lib = Path(os.path.expanduser("~/.local/lib"))
    if not local_lib.is_dir():
        return

    ld_library_path = os.environ.get("LD_LIBRARY_PATH", "")
    local_lib_text = str(local_lib)
    if local_lib_text not in ld_library_path:
        os.environ["LD_LIBRARY_PATH"] = f"{local_lib_text}:{ld_library_path}" if ld_library_path else local_lib_text

    turbojpeg = local_lib / "libturbojpeg.so.0"
    if not turbojpeg.is_file():
        return

    try:
        ctypes.cdll.LoadLibrary(str(turbojpeg))
    except OSError:
        pass
