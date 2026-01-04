# NOMAD Competition Day Deployment Checklist

**Competition:** AEAC 2026  
**Team:** McGill Aerial Design  
**System:** NOMAD (Networked Operations for MAD)

---

## Pre-Competition Setup (1 Week Before)

### Jetson Configuration

- [ ] **Install Base System**
  - [ ] Flash JetPack 5.1+ to Jetson Orin Nano
  - [ ] Create user `nomad` with sudo privileges
  - [ ] Set hostname: `nomad-jetson`
  - [ ] Update system: `sudo apt-get update && sudo apt-get upgrade`

- [ ] **Install Dependencies**
  ```bash
  # ZED SDK
  wget https://download.stereolabs.com/zedsdk/4.0/l4t35.2/jetsons
  chmod +x ZED_SDK_*.run
  ./ZED_SDK_*.run -- silent
  
  # Python packages
  python3 -m pip install -r edge_core/requirements.txt
  
  # MAVLink Router
  sudo apt-get install mavlink-router
  ```

- [ ] **Clone Repository**
  ```bash
  cd /home/nomad
  git clone <REPO_URL> NOMAD
  cd NOMAD
  ```

- [ ] **Configure Environment**
  ```bash
  cp config/env/jetson.env.example config/env/.env
  nano config/env/.env
  ```
  
  Edit values:
  - `NOMAD_HOST=0.0.0.0`
  - `NOMAD_PORT=8000`
  - `MAVLINK_UART_DEV=/dev/ttyTHS1`
  - `MAVLINK_UART_BAUD=921600`
  - `ZED_SERIAL_NUMBER=<YOUR_ZED_SN>`

- [ ] **Install SystemD Service**
  ```bash
  sudo cp infra/nomad.service /etc/systemd/system/
  sudo systemctl daemon-reload
  sudo systemctl enable nomad
  sudo systemctl start nomad
  sudo systemctl status nomad
  ```

- [ ] **Install Tailscale**
  ```bash
  curl -fsSL https://tailscale.com/install.sh | sh
  sudo tailscale up --authkey=<YOUR_KEY> --hostname=nomad-jetson
  tailscale status
  # Note the Tailscale IP (e.g., 100.100.10.5)
  ```

- [ ] **Configure MAVLink Router**
  ```bash
  sudo cp transport/mavlink_router/main.conf /etc/mavlink-router/main.conf
  sudo systemctl enable mavlink-router
  sudo systemctl start mavlink-router
  sudo systemctl status mavlink-router
  ```

- [ ] **Test Services**
  ```bash
  # Test API
  curl http://127.0.0.1:8000/health
  
  # Test MAVLink (if FC connected)
  sudo apt-get install mavproxy
  mavproxy.py --master=udp:127.0.0.1:14550
  ```

### Ground Station Configuration

- [ ] **Install Software**
  - [ ] Mission Planner 1.3.x
  - [ ] Tailscale client
  - [ ] VLC Media Player or FFplay
  - [ ] Visual Studio 2022 (for plugin development)

- [ ] **Build Mission Planner Plugin**
  ```powershell
  cd mission_planner\src
  .\build_and_deploy.ps1
  ```

- [ ] **Install Tailscale**
  - [ ] Download from https://tailscale.com/download
  - [ ] Login to same tailnet as Jetson
  - [ ] Note Jetson's Tailscale IP

- [ ] **Configure Mission Planner Plugin**
  - [ ] Launch Mission Planner
  - [ ] Go to NOMAD → Settings
  - [ ] Set Jetson IP to Tailscale address
  - [ ] Set Jetson Port to `8000`
  - [ ] Set RTSP URLs:
    - Primary: `rtsp://<jetson-tailscale-ip>:8554/live`
    - Secondary: `rtsp://<jetson-tailscale-ip>:8554/gimbal`
  - [ ] Save settings

### 4G/LTE Setup

- [ ] **Install SIM Card**
  - [ ] Insert SIM into 4G modem on Jetson
  - [ ] Configure APN settings
  - [ ] Test connectivity: `ping 8.8.8.8`

- [ ] **Verify Tailscale Over 4G**
  - [ ] Disconnect Jetson from WiFi
  - [ ] Check Tailscale status: `tailscale status`
  - [ ] From laptop, ping Jetson: `ping <jetson-tailscale-ip>`
  - [ ] Test API: `curl http://<jetson-tailscale-ip>:8000/health`

### Hardware Integration

- [ ] **Connect Cube Orange to Jetson**
  - [ ] UART: Cube TELEM2 → Jetson THS1
  - [ ] Baud rate: 921600
  - [ ] Verify: `sudo cat /dev/ttyTHS1` (should show MAVLink bytes)

- [ ] **Mount ZED Camera**
  - [ ] Secure ZED 2i to drone frame
  - [ ] Connect USB 3.0 to Jetson
  - [ ] Verify: `ls /dev/video*`
  - [ ] Test: `ZED_Explorer` or `zed-diagnostic`

- [ ] **Connect Gimbal**
  - [ ] Servo to Cube Orange PWM out
  - [ ] Test gimbal movement via Mission Planner

- [ ] **Power System**
  - [ ] Jetson powered from battery via DC-DC converter
  - [ ] Test power-on sequence
  - [ ] Verify boot time (<60 seconds)

### Pre-Flight Validation

- [ ] **System Health Check**
  ```bash
  ssh nomad@<jetson-tailscale-ip>
  
  # Check services
  sudo systemctl status nomad
  sudo systemctl status mavlink-router
  sudo systemctl status tailscaled
  
  # Check hardware
  jtop  # CPU/GPU/RAM/Temp
  
  # Check logs
  sudo journalctl -u nomad -n 50
  ```

- [ ] **Mission Planner Connection Test**
  - [ ] Launch Mission Planner
  - [ ] Open NOMAD Control Panel
  - [ ] Verify connection (green status)
  - [ ] Check telemetry updates
  - [ ] Test health monitor
  - [ ] Test video streams (both cameras)

- [ ] **WASD Control Test (SITL or Ground Test)**
  - [ ] Connect to drone in Guided mode
  - [ ] Enable WASD control
  - [ ] Test W/A/S/D/Q/E keys
  - [ ] Verify MAVLink inspector shows velocity commands
  - [ ] Test RC override (should override WASD)

---

## Competition Day Morning (3 Hours Before Flight)

### System Boot-Up

- [ ] **Power On Jetson**
  - [ ] Connect battery
  - [ ] Wait 60 seconds for boot
  - [ ] Verify LED status

- [ ] **SSH into Jetson**
  ```bash
  ssh nomad@<jetson-tailscale-ip>
  ```

- [ ] **Check System Status**
  ```bash
  # Services
  sudo systemctl status nomad
  sudo systemctl status mavlink-router
  tailscale status
  
  # Health
  jtop
  curl http://127.0.0.1:8000/health
  
  # Logs
  sudo journalctl -u nomad -f  # Leave running in separate terminal
  ```

- [ ] **Verify 4G Connection**
  ```bash
  ping -c 4 8.8.8.8
  tailscale ping <ground-station-ip>
  ```

### Ground Station Setup

- [ ] **Launch Mission Planner**
  - [ ] Connect to drone (verify MAVLink telemetry)
  - [ ] Load flight plan for task
  - [ ] Verify GPS lock
  - [ ] Check battery voltage

- [ ] **Open NOMAD Control Panel**
  - [ ] NOMAD → Open Control Panel
  - [ ] Verify "Connected" status
  - [ ] Check telemetry updates
  - [ ] Verify health monitor is green

- [ ] **Test Video Streams**
  - [ ] Click [VID] Primary → VLC opens
  - [ ] Click [TGT] Secondary → VLC opens
  - [ ] Verify low latency (<500ms)
  - [ ] Close VLC windows

### Pre-Flight Checklist

#### Hardware

- [ ] **Drone**
  - [ ] Propellers secure
  - [ ] Battery charged and secure
  - [ ] ZED camera lens clean
  - [ ] Gimbal moves freely
  - [ ] RC transmitter on and bound
  - [ ] Safety switch on Cube Orange

- [ ] **Jetson**
  - [ ] Power LED on
  - [ ] ZED USB connected
  - [ ] UART to Cube connected
  - [ ] 4G/LTE antenna secured
  - [ ] Cooling fan running

#### Software

- [ ] **Mission Planner**
  - [ ] MAVLink connected
  - [ ] Flight mode: STABILIZE
  - [ ] Arming checks passed
  - [ ] Geofence loaded (if required)
  - [ ] Failsafe: RTL

- [ ] **NOMAD System**
  - [ ] Control panel open
  - [ ] Connection: GREEN
  - [ ] Health monitor: ALL GREEN
  - [ ] Video streams working
  - [ ] Telemetry updating

---

## Task 1: Locate (Outdoor GPS Mode)

### Pre-Flight

- [ ] **Load Parameter File**
  ```bash
  # In Mission Planner
  Config → Full Parameter List → Load from File
  Select: config/profiles/task1_outdoor.params
  Write Params → Reboot
  ```

- [ ] **Load Waypoints**
  - [ ] Flight Plan tab
  - [ ] Load task 1 search pattern
  - [ ] Verify waypoints on map
  - [ ] Set home position

- [ ] **NOMAD Control Panel**
  - [ ] Task 1 section visible
  - [ ] [CAP] button enabled
  - [ ] WASD control: DISABLED

### During Flight

- [ ] **Takeoff**
  - [ ] Arm drone (Safety switch + RC)
  - [ ] Switch to GUIDED or AUTO mode
  - [ ] Initiate takeoff command
  - [ ] Monitor altitude and position

- [ ] **Search Pattern**
  - [ ] Execute waypoint mission
  - [ ] Monitor video feed for target
  - [ ] Adjust altitude if needed

- [ ] **Target Acquisition**
  - [ ] Visual confirmation of target in ZED feed
  - [ ] Click **[CAP] Capture Snapshot**
  - [ ] Wait for result (5-10 seconds)
  - [ ] Verify GPS coordinates and description
  - [ ] Record data on paper/tablet

- [ ] **Return**
  - [ ] Continue mission or RTL
  - [ ] Land safely
  - [ ] Disarm

### Post-Flight

- [ ] **Download Mission Log**
  ```bash
  scp nomad@<jetson-ip>:/home/nomad/NOMAD/data/mission_logs/task1_*.json ./
  ```

- [ ] **Submit Results**
  - [ ] GPS coordinates
  - [ ] Target description
  - [ ] Snapshot image (if stored)
  - [ ] Mission log JSON

---

## Task 2: Search & Extinguish (Indoor VIO Mode)

### Pre-Flight

- [ ] **Load Parameter File**
  ```bash
  # In Mission Planner
  Config → Full Parameter List → Load from File
  Select: config/profiles/task2_indoor.params
  Write Params → Reboot
  ```

- [ ] **Configure VIO**
  - [ ] EK3_SRC1_POSZ = 0 (No GPS)
  - [ ] EK3_SRC1_VELXY = 6 (ExternalNav)
  - [ ] Verify ZED T265 or ZED 2i VIO active

- [ ] **NOMAD Control Panel**
  - [ ] Task 2 section visible
  - [ ] Click **[CLR] Reset Exclusion Map**
  - [ ] Verify "Targets: 0"
  - [ ] WASD control: DISABLED (enable only if needed)

### During Flight

- [ ] **Takeoff**
  - [ ] Indoor position (away from walls)
  - [ ] Arm drone
  - [ ] Switch to GUIDED mode
  - [ ] Takeoff command (1-2m altitude)

- [ ] **Search Pattern**
  - [ ] Manual control or waypoint mission
  - [ ] Monitor video feed for fire detection
  - [ ] NOMAD automatically tracks detected fires
  - [ ] Check "Targets: X" count in control panel

- [ ] **WASD Nudge (Optional)**
  - [ ] If precise positioning needed:
    - [ ] Enable "WASD Indoor Control" checkbox
    - [ ] Accept warning dialog
    - [ ] Use W/A/S/D/Q/E to nudge drone
    - [ ] Speed: 0.5 m/s (adjust as needed)
    - [ ] **CRITICAL:** RC transmitter ready for override

- [ ] **Fire Extinguishing**
  - [ ] Gimbal auto-aims at fire (visual servoing)
  - [ ] Manual trigger extinguisher
  - [ ] Verify exclusion map updated
  - [ ] Continue to next fire

- [ ] **Complete Mission**
  - [ ] All fires extinguished
  - [ ] Return to landing zone
  - [ ] Land safely
  - [ ] Disarm

### Post-Flight

- [ ] **Download Mission Log**
  ```bash
  scp nomad@<jetson-ip>:/home/nomad/NOMAD/data/mission_logs/task2_*.json ./
  ```

- [ ] **Submit Results**
  - [ ] Number of targets hit
  - [ ] Mission duration
  - [ ] Exclusion map data
  - [ ] Mission log JSON

---

## Emergency Procedures

### Lost Telemetry (No MAVLink)

**Symptoms:** Mission Planner shows no telemetry

**Actions:**
1. **Check Tailscale:**
   ```bash
   tailscale status  # On laptop
   ```
2. **Check 4G Signal:** Verify signal strength
3. **Restart MAVLink Router:**
   ```bash
   ssh nomad@<jetson-ip>
   sudo systemctl restart mavlink-router
   ```
4. **Fallback:** Use RC transmitter for manual control
5. **RTL:** Switch to Return-to-Launch mode

### High Latency (>500ms)

**Symptoms:** Slow telemetry updates, laggy video

**Actions:**
1. Accept degraded performance
2. Rely on RC link for control
3. Disable WASD control (too slow)
4. Continue mission (telemetry is monitoring only)

### Vision Process Crash

**Symptoms:** No vision detections, no video stream

**Automatic Recovery:** Watchdog restarts after 3 failures (3 seconds)

**Manual Override:**
```bash
ssh nomad@<jetson-ip>
sudo systemctl restart nomad
```

### API Not Responding

**Symptoms:** [CAP] button fails, health monitor offline

**Check:**
```bash
ssh nomad@<jetson-ip>
sudo systemctl status nomad
sudo journalctl -u nomad -n 50
sudo systemctl restart nomad
```

### High Temperature (>80°C)

**Symptoms:** Jetson thermal throttling, slow performance

**Actions:**
1. Land drone immediately
2. Allow Jetson to cool (5 minutes)
3. Check cooling fan
4. Reduce YOLO inference rate if possible

### WASD Not Responding

**Check:**
- [ ] Is checkbox enabled?
- [ ] Is Mission Planner window focused?
- [ ] Is drone in GUIDED mode?
- [ ] Is MAVLink connected?

**Fix:** Disable and re-enable WASD control

### Lost GPS Lock (Task 1)

**Symptoms:** GPS status shows NO FIX

**Actions:**
1. Circle area for 30-60 seconds
2. Increase altitude (better satellite view)
3. If no fix: Abort task, land safely
4. Use VIO mode as fallback (Task 2 params)

---

## Post-Competition

### Data Collection

- [ ] **Download All Logs**
  ```bash
  scp -r nomad@<jetson-ip>:/home/nomad/NOMAD/data/mission_logs ./logs_$(date +%Y%m%d)/
  ```

- [ ] **Download Video Recordings (if enabled)**
  ```bash
  scp -r nomad@<jetson-ip>:/mnt/nvme/recordings ./videos_$(date +%Y%m%d)/
  ```

- [ ] **Export Mission Planner Logs**
  - [ ] Flight Data → Dataflash Logs → Download
  - [ ] Save to archive folder

### System Shutdown

- [ ] **Graceful Shutdown**
  ```bash
  ssh nomad@<jetson-ip>
  sudo shutdown -h now
  ```

- [ ] **Disconnect Power**
  - [ ] Wait for Jetson power LED to turn off
  - [ ] Disconnect battery
  - [ ] Secure all cables

### Review and Debrief

- [ ] **Analyze Mission Logs**
  ```bash
  cat logs/*.json | jq .
  ```

- [ ] **Review Video Recordings**
  - [ ] VLC playback of RTSP recordings
  - [ ] Identify any issues

- [ ] **Document Issues**
  - [ ] What worked well?
  - [ ] What needs improvement?
  - [ ] Any bugs or crashes?
  - [ ] Performance metrics

---

## Contact Information

**Team Members:**
- Name: _______________  Phone: _______________
- Name: _______________  Phone: _______________

**Emergency Contacts:**
- Competition Officials: _______________
- 4G/LTE Support: _______________
- Tailscale Support: support@tailscale.com

**System Information:**
- Jetson Tailscale IP: _______________
- Laptop Tailscale IP: _______________
- 4G/LTE APN: _______________
- ZED Serial Number: _______________

---

**Good luck! 🚁**  
**McGill Aerial Design - AEAC 2026**
