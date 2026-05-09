import re
from pathlib import Path

p = Path('/workspaces/isaac_ros-dev/install/zed_wrapper/share/zed_wrapper/config/common_stereo.yaml')
t = p.read_text()
t2 = re.sub(r'(?m)^(\s*depth_confidence\s*:\s*)[0-9]+', r'\g<1>50', t)
t2 = re.sub(r'(?m)^(\s*depth_texture_conf\s*:\s*)[0-9]+', r'\g<1>70', t2)
if t2 != t:
    p.write_text(t2)
    print('Patched depth_confidence=50 depth_texture_conf=70')
else:
    print('Already patched')
# Verify
for line in p.read_text().splitlines():
    if 'depth_confidence' in line or 'depth_texture' in line:
        print(line.strip())
