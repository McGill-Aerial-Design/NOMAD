# Jetson Orin Nano Video - FINAL Working Config

## CRITICAL FACTS
- Orin Nano has NO HW encoder (NVENC) - must use SOFTWARE only
- x264enc NOT available in container
- openh264enc IS available - USE THIS
- Everything in single container: nomad_isaac_ros_32
- MediaMTX running inside container at /usr/local/bin/mediamtx

## Working Software Encoder
**Use openh264enc** (confirmed available):
```bash
gst-inspect-1.0 openh264enc  # Confirmed exists
```

## Files to Update  
`software_rtsp_bridge.py` - Change x264enc to openh264enc

## Container Status
- MediaMTX: Running (PID 213177) at /usr/local/bin/mediamtx
- Config: /tmp/mediamtx_minimal.yml  
- Port: 8554 (RTSP)  
- ZED: Publishing 30fps on /zed/zed_node/rgb/image_rect_color

## Next Steps
1. Fix software_rtsp_bridge.py to use openh264enc (not x264enc)
2. Deploy and start
3. Test VLC at rtsp://100.75.218.89:8554/primary
4. Should work!

## VLC Command
```powershell
& "C:\Program Files\VideoLAN\VLC\vlc.exe" "rtsp://100.75.218.89:8554/primary" --network-caching=300 --rtsp-tcp --no-audio
```
