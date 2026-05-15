#!/bin/bash
# Test camera open from HOST (not container)
export LD_LIBRARY_PATH=/home/mad/.local/lib:$LD_LIBRARY_PATH
python3 -c "
import ctypes
ctypes.cdll.LoadLibrary('/home/mad/.local/lib/libturbojpeg.so.0')
import pyzed.sl as sl
print('SDK:', sl.Camera.get_sdk_version())
c = sl.Camera()
p = sl.InitParameters()
p.depth_mode = sl.DEPTH_MODE.NONE
p.camera_resolution = sl.RESOLUTION.VGA
p.camera_fps = 15
s = c.open(p)
print('Result:', s)
if s == sl.ERROR_CODE.SUCCESS:
    info = c.get_camera_information()
    print('Model:', info.camera_model)
    print('Serial:', info.serial_number)
    print('FW:', info.camera_configuration.firmware_version)
    c.close()
    print('Camera works on host!')
else:
    print('Camera FAILED on host too')
" 2>&1
