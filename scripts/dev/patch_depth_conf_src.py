# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
import re
from pathlib import Path

candidates = [
    Path("/workspaces/isaac_ros-dev/install/zed_wrapper/share/zed_wrapper/config/common_stereo.yaml"),
    Path("/workspaces/isaac_ros-dev/src/zed-ros2-wrapper/zed_wrapper/config/common_stereo.yaml"),
]

for p in candidates:
    if not p.exists():
        print(f"Not found: {p}")
        continue
    t = p.read_text()
    t2 = re.sub(r"(?m)^(\s*depth_confidence\s*:\s*)[0-9]+", r"\g<1>50", t)
    t2 = re.sub(r"(?m)^(\s*depth_texture_conf\s*:\s*)[0-9]+", r"\g<1>70", t2)
    if t2 != t:
        p.write_text(t2)
        print(f"Patched {p}")
    else:
        print(f"Already patched: {p}")
    for line in p.read_text().splitlines():
        if "depth_confidence" in line or "depth_texture" in line:
            print(f"  {line.strip()}")
