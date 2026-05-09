import json, subprocess
result = subprocess.run(['docker', 'inspect', 'nomad_isaac_ros'], capture_output=True, text=True)
d = json.loads(result.stdout)
for m in d[0]['Mounts']:
    print(m.get('Source',''), '->', m.get('Destination',''))
