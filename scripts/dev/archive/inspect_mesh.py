#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Inspect current SLAM mesh data from Edge Core API."""

import json
import sys
import urllib.request

host = sys.argv[1] if len(sys.argv) > 1 else "localhost"
url = f"http://{host}:8000/api/slam/mesh"
try:
    resp = urllib.request.urlopen(url, timeout=3)
except Exception as e:
    print(f"ERROR connecting to {url}: {e}")
    sys.exit(1)

full = json.loads(resp.read())
print(f"available: {full.get('available')}")
print(f"timestamp: {full.get('timestamp')}")

d = full.get("mesh") or {}
mode = d.get("mode", "?")
print(f"mode: {mode}")

if mode == "triangle":
    verts = d.get("vertices", [])
    indices = d.get("indices", [])
    colors = d.get("colors", [])
    print(f"total_vertices: {d.get('total_vertices', len(verts))}")
    print(f"total_triangles: {d.get('total_triangles', len(indices) // 3)}")
    print(f"blocks_processed: {d.get('blocks_processed', '?')}")
    print(f"block_size: {d.get('block_size', '?')}")
    if verts:
        print(f"sample vertices: {verts[:5]}")
    if indices:
        print(f"sample indices: {indices[:12]}")
    if colors:
        print(f"sample colors: {colors[:5]}")
elif mode == "voxel":
    vox = d.get("voxels", [])
    print(f"voxel_size: {d.get('voxel_size')}")
    print(f"total_voxels: {len(vox)}")
    if vox:
        print(f"sample voxels: {vox[:5]}")
elif mode == "block":
    blocks = d.get("blocks", [])
    print(f"block_size: {d.get('block_size')}")
    print(f"total_blocks: {len(blocks)}")
    if blocks:
        print(f"sample blocks: {blocks[:5]}")
else:
    print(f"raw mesh data keys: {list(d.keys()) if d else 'None'}")

print(f"drone_pos: {full.get('drone_position')}")
print(f"drone_att: {full.get('drone_attitude')}")
