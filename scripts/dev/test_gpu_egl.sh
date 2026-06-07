#!/bin/bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
echo "=== GPU test ==="
python3 -c "
try:
    import torch
    print('PyTorch CUDA:', torch.cuda.is_available(), torch.cuda.get_device_name(0) if torch.cuda.is_available() else 'N/A')
except:
    print('PyTorch not available')
" 2>&1

echo ""
echo "=== CUDA test ==="
python3 -c "
import ctypes
cuda = ctypes.CDLL('libcudart.so')
count = ctypes.c_int()
cuda.cudaGetDeviceCount(ctypes.byref(count))
print('CUDA devices:', count.value)
" 2>&1

echo ""
echo "=== NVIDIA EGL library ==="
ls -la /usr/lib/aarch64-linux-gnu/tegra-egl/libEGL_nvidia.so.0
ls -la /usr/lib/aarch64-linux-gnu/nvidia/libEGL* 2>/dev/null || echo "No nvidia/libEGL"
ldconfig -p | grep -i libEGL

echo ""
echo "=== DRM device permissions ==="
ls -la /dev/dri/
id

echo ""
echo "=== NVIDIA driver test ==="
python3 << 'PYEOF'
import ctypes

# Test loading the NVIDIA EGL directly
try:
    nvidia_egl = ctypes.CDLL("/usr/lib/aarch64-linux-gnu/tegra-egl/libEGL_nvidia.so.0")
    print("NVIDIA EGL loaded OK")
except Exception as e:
    print(f"NVIDIA EGL load failed: {e}")

# Check nvidia drm module
import os
for f in os.listdir("/dev/dri/"):
    path = f"/dev/dri/{f}"
    try:
        fd = os.open(path, os.O_RDWR)
        os.close(fd)
        print(f"  {path}: accessible")
    except Exception as e:
        print(f"  {path}: {e}")

# Test EGL with explicit NVIDIA platform
egl = ctypes.CDLL("libEGL.so.1")

# Try getting platform display
try:
    eglGetPlatformDisplay = egl.eglGetPlatformDisplay
    eglGetPlatformDisplay.restype = ctypes.c_void_p
    eglGetPlatformDisplay.argtypes = [ctypes.c_uint, ctypes.c_void_p, ctypes.c_void_p]

    EGL_PLATFORM_DEVICE_EXT = 0x313F
    display = eglGetPlatformDisplay(EGL_PLATFORM_DEVICE_EXT, 0, None)
    print(f"Platform device display: {display}")

    major = ctypes.c_int()
    minor = ctypes.c_int()
    result = egl.eglInitialize(display, ctypes.byref(major), ctypes.byref(minor))
    if result:
        print(f"EGL initialized: {major.value}.{minor.value}")
        egl.eglTerminate(display)
    else:
        err = egl.eglGetError()
        print(f"Platform display init failed: {hex(err)}")
except Exception as e:
    print(f"Platform display: {e}")
PYEOF

echo ""
echo "=== Check if nvidia_drm/nvidia modules loaded ==="
lsmod 2>/dev/null | grep nvidia || cat /proc/modules 2>/dev/null | grep nvidia | head -5
