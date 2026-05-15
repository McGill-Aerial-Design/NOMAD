#!/bin/bash
echo "=== EGL vendor search paths ==="
cat /usr/lib/aarch64-linux-gnu/tegra-egl/ld.so.conf 2>/dev/null
echo ""
cat /etc/ld.so.conf 2>/dev/null
echo ""
cat /etc/ld.so.conf.d/*.conf 2>/dev/null
echo ""
ldconfig -v 2>/dev/null | grep tegra

echo ""
echo "=== EGL vendor JSON ==="
for f in /usr/share/glvnd/egl_vendor.d/*.json; do echo "--- $f ---"; cat "$f"; echo; done

echo ""
echo "=== Check GLVND dispatch ==="
python3 << 'PYEOF'
import ctypes
import os

# The issue: libEGL.so.1 is from GLVND, it dispatches to vendor ICDs
# GLVND uses __EGL_VENDOR_LIBRARY_DIRS to find vendor JSONs
# Then vendor JSON points to the actual platform library

print("__EGL_VENDOR_LIBRARY_DIRS:", os.environ.get("__EGL_VENDOR_LIBRARY_DIRS", "UNSET"))

# Force NVIDIA vendor
os.environ["__EGL_VENDOR_LIBRARY_FILENAMES"] = "/usr/lib/aarch64-linux-gnu/tegra-egl/nvidia.json"

egl = ctypes.CDLL("libEGL.so.1")
display = egl.eglGetDisplay(0)
print("Display:", display)

major = ctypes.c_int()
minor = ctypes.c_int()
r = egl.eglInitialize(display, ctypes.byref(major), ctypes.byref(minor))
print("eglInitialize:", r, f"{major.value}.{minor.value}" if r else hex(egl.eglGetError()))
if r:
    egl.eglTerminate(display)

# Now test with the nvidia vendor JSON pointing to the tegra-egl path
del os.environ["__EGL_VENDOR_LIBRARY_FILENAMES"]

# Try 10_nvidia.json
os.environ["__EGL_VENDOR_LIBRARY_FILENAMES"] = "/usr/share/glvnd/egl_vendor.d/10_nvidia.json"
display2 = egl.eglGetDisplay(0)
r2 = egl.eglInitialize(display2, ctypes.byref(major), ctypes.byref(minor))
print("With 10_nvidia.json:", r2, f"{major.value}.{minor.value}" if r2 else hex(egl.eglGetError()))
if r2:
    egl.eglTerminate(display2)
PYEOF

echo ""
echo "=== Check nvidia.json vendor path resolution ==="
cat /usr/lib/aarch64-linux-gnu/tegra-egl/nvidia.json
echo ""
ls -la /usr/lib/aarch64-linux-gnu/tegra-egl/libEGL_nvidia.so.0
echo ""
echo "Can ld find libEGL_nvidia.so.0?"
ldconfig -p | grep libEGL_nvidia

echo ""
echo "=== Test: is tegra-egl in linker path? ==="
python3 -c "
import ctypes
try:
    lib = ctypes.CDLL('libEGL_nvidia.so.0')
    print('Direct load of libEGL_nvidia: OK')
except Exception as e:
    print(f'Failed: {e}')
"

echo ""
echo "=== Host EGL test (outside container) ==="
python3 << 'PYEOF'
import ctypes
egl = ctypes.CDLL("libEGL.so.1")
display = egl.eglGetDisplay(0)
major = ctypes.c_int()
minor = ctypes.c_int()
r = egl.eglInitialize(display, ctypes.byref(major), ctypes.byref(minor))
print("Host EGL:", r, f"{major.value}.{minor.value}" if r else hex(egl.eglGetError()))
if r:
    egl.eglTerminate(display)
PYEOF
