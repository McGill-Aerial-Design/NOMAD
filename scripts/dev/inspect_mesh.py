#!/usr/bin/env python3
import urllib.request, json
resp = urllib.request.urlopen("http://localhost:8000/api/task/2/slam/mesh")
full = json.loads(resp.read())
d = full.get("mesh", {})
print("mode:", d.get("mode"))
print("voxel_size:", d.get("voxel_size"))
vox = d.get("voxels", [])
print("total_voxels:", len(vox))
if vox:
    print("sample voxels:", vox[:5])
print("drone_pos:", full.get("drone_position"))
print("drone_att:", full.get("drone_attitude"))
