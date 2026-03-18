#!/usr/bin/env python3
import urllib.request, json
resp = urllib.request.urlopen("http://localhost:8000/api/task/2/slam/mesh")
full = json.loads(resp.read())
mesh = full.get("mesh", {})
print("mesh mode:", mesh.get("mode"))
print("voxels count:", len(mesh.get("voxels", [])))
print("blocks count:", len(mesh.get("blocks", [])))
print("timestamp:", full.get("timestamp"))
print("total_voxels:", mesh.get("total_voxels"))
print("total_blocks:", mesh.get("total_blocks"))
if mesh.get("voxels"):
    print("first voxel:", mesh["voxels"][0] if mesh["voxels"] else None)
if mesh.get("blocks"):
    print("first block:", mesh["blocks"][0] if mesh["blocks"] else None)
