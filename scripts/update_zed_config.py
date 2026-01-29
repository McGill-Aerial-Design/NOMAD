#!/usr/bin/env python3
import re
import os

# Use expanduser to support ~/ paths, making the script portable across users
# This resolves to /home/mad/... if running as mad user, or /home/<user>/... for others
config_path = os.path.expanduser('~/ros2/isaac_ros_ws/src/zed-ros2-wrapper/zed_wrapper/config/common_stereo.yaml')

with open(config_path, 'r') as f:
    content = f.read()

# Update od_enabled
content = re.sub(r"od_enabled: false", "od_enabled: true", content)

# Update model to custom
content = re.sub(r"model: 'MULTI_CLASS_BOX_FAST'", "model: 'CUSTOM_YOLOLIKE_BOX_OBJECTS'", content, count=1)

# Update custom_onnx_file - use container path
content = re.sub(r"custom_onnx_file: '[^']*'", "custom_onnx_file: '/workspaces/isaac_ros-dev/best.onnx'", content, count=1)

# Lower confidence threshold for our custom model
content = re.sub(r"confidence_threshold: 75\.0", "confidence_threshold: 40.0", content, count=1)

with open(config_path, 'w') as f:
    f.write(content)

print('Config updated successfully')
