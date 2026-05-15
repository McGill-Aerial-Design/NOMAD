#!/bin/bash
echo "=== EGL diagnostics ==="
echo "DISPLAY=$DISPLAY"
echo "WAYLAND_DISPLAY=$WAYLAND_DISPLAY"
ls -la /dev/dri/ 2>/dev/null
echo ""

echo "=== Test EGL with eglinfo ==="
apt list --installed 2>/dev/null | grep -i egl | head -5

echo ""
echo "=== Try with DRI render node ==="
python3 << 'PYEOF'
import ctypes
import os

# Try different EGL configurations
configs = [
    {"EGL_PLATFORM": "device", "DISPLAY": ""},
    {"EGL_PLATFORM": "surfaceless", "DISPLAY": ""},
    {"EGL_PLATFORM": "device", "DISPLAY": ":0"},
    {},  # Default
]

for i, env in enumerate(configs):
    for k, v in env.items():
        os.environ[k] = v
    
    try:
        egl = ctypes.CDLL("libEGL.so.1")
        
        # eglGetDisplay
        EGL_DEFAULT_DISPLAY = 0
        display = egl.eglGetDisplay(EGL_DEFAULT_DISPLAY)
        
        # eglInitialize
        major = ctypes.c_int()
        minor = ctypes.c_int()
        result = egl.eglInitialize(display, ctypes.byref(major), ctypes.byref(minor))
        
        if result:
            print(f"Config {i} ({env}): EGL {major.value}.{minor.value} - OK!")
            egl.eglTerminate(display)
        else:
            err = egl.eglGetError()
            print(f"Config {i} ({env}): eglInitialize failed (error {err:#x})")
    except Exception as e:
        print(f"Config {i}: Exception: {e}")
    
    # Clean env
    for k in env:
        if k in os.environ:
            del os.environ[k]
PYEOF

echo ""
echo "=== nvbufsurftransform test ==="
python3 << 'PYEOF'
import ctypes
import os

os.environ["DISPLAY"] = ""

try:
    lib = ctypes.CDLL("libnvbufsurftransform.so")
    print("nvbufsurftransform loaded successfully")
    
    # The library calls EGL internally on first use
    # Let's see if the error message changes
except Exception as e:
    print(f"Error: {e}")
PYEOF

echo ""
echo "=== Try ZED with X11 dummy display ==="
apt install -y xvfb > /dev/null 2>&1
if command -v Xvfb &> /dev/null; then
    Xvfb :99 -screen 0 1024x768x24 &
    XVFB_PID=$!
    sleep 1
    export DISPLAY=:99
    echo "Xvfb started on :99"
    
    python3 << 'PYEOF'
import pyzed.sl as sl
cam = sl.Camera()
devs = cam.get_device_list()
print("ZED cameras with Xvfb:", len(devs))
for d in devs:
    print(f"  {d.camera_model} SN:{d.serial_number}")
PYEOF
    
    kill $XVFB_PID 2>/dev/null
else
    echo "Xvfb not available, trying with :0"
    export DISPLAY=:0
    python3 << 'PYEOF'
import pyzed.sl as sl
cam = sl.Camera()
devs = cam.get_device_list()
print("ZED cameras with DISPLAY=:0:", len(devs))
PYEOF
fi
