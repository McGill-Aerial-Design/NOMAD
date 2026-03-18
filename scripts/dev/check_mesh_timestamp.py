#!/usr/bin/env python3
import urllib.request, json
resp = urllib.request.urlopen("http://localhost:8000/api/task/2/slam/mesh")
full = json.loads(resp.read())
mesh = full.get("mesh", {})
print("Overall timestamp:", full.get("timestamp"))
print("Mesh received_at:", mesh.get("received_at"))
print("Block count:", len(mesh.get("blocks", [])))
print("Mesh timestamp:", mesh.get("timestamp"))
print("")
if mesh.get("blocks"):
    print("Sample block:", mesh["blocks"][0])
