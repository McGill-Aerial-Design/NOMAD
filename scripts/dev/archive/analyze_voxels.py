# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
import json
import sys

d = json.loads(sys.stdin.read())
v = d["mesh"]["voxels"]
xs = [x["p"][0] for x in v if x.get("p")]
ys = [x["p"][1] for x in v if x.get("p")]
zs = [x["p"][2] for x in v if x.get("p")]
dp = d["drone_position"]
print(f"Drone:  ({dp['x']:.2f}, {dp['y']:.2f}, {dp['z']:.2f})")
print(f"Voxels X: [{min(xs):.2f}, {max(xs):.2f}]")
print(f"Voxels Y: [{min(ys):.2f}, {max(ys):.2f}]")
print(f"Voxels Z: [{min(zs):.2f}, {max(zs):.2f}]")
print(f"Count: {len(xs)}")
print(f"Voxel size: {d['mesh']['voxel_size']}")
