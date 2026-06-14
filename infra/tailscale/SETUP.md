# Tailscale Setup for NOMAD Remote Access

**The Link: Secure 4G/LTE tunnel between Jetson and Ground Station**

## Overview

Tailscale creates a secure WireGuard VPN mesh network that allows the Ground Station to access the Jetson Orin Nano over 4G/LTE, bypassing WiFi range limitations and NAT firewalls.

**Use Cases:**
- MAVLink telemetry streaming to Mission Planner over cellular
- HTTP access to Jetson API for monitoring and control
- SSH access for remote debugging and code deployment
- RTSP video streaming (single ZED stream)

See [README.md](README.md) for the architecture diagram and port table.

## Installation

### 1. Jetson Orin Nano Setup

**Automated (recommended):** `scripts/setup.sh` installs Tailscale,
authenticates, and installs the connection watchdog service in one step:

```bash
sudo infra/tailscale/scripts/setup.sh --authkey <YOUR_AUTH_KEY>
```

The manual steps below are the fallback if you need to do it by hand.

**Prerequisites:**
- Ubuntu 20.04+ (JetPack SDK)
- 4G/LTE USB modem connected and configured
- Internet access via cellular or WiFi

**Install Tailscale:**

```bash
# Download and install Tailscale
curl -fsSL https://tailscale.com/install.sh | sh

# Authenticate (generates login URL)
sudo tailscale up --authkey=<YOUR_AUTH_KEY> --hostname=nomad-jetson

# Alternative: Interactive authentication (requires browser)
sudo tailscale up --hostname=nomad-jetson

# Verify installation
tailscale status
tailscale ip -4  # Note this IP for Ground Station configuration
```

**Get Auth Key:**
1. Log in to https://login.tailscale.com/admin/settings/keys
2. Generate a reusable auth key
3. Use it in the `--authkey` parameter above

**Configure Auto-Start:**

```bash
# Enable Tailscale service
sudo systemctl enable tailscaled
sudo systemctl start tailscaled

# Verify service is running
sudo systemctl status tailscaled
```

**Configure Network Priority:**

Ensure Tailscale starts after network interfaces are up:

```bash
# Edit /etc/systemd/system/multi-user.target.wants/tailscaled.service
sudo systemctl edit tailscaled

# Add the following:
[Unit]
After=network-online.target
Wants=network-online.target
```

**Test Connectivity:**

```bash
# Check Tailscale IP
tailscale ip -4
# Expected output: 100.x.x.x

# Test internet connectivity through Tailscale
curl --interface tailscale0 https://www.google.com
```

---

### 2. Ground Station (Laptop) Setup

**Windows:**

1. Download Tailscale installer from https://tailscale.com/download/windows
2. Run installer
3. Login with the same Tailscale account used on Jetson
4. Verify connection:
   ```powershell
   tailscale status
   ping <jetson-tailscale-ip>
   ```

**Linux (including a Raspberry Pi ground station):**

```bash
# Install Tailscale
curl -fsSL https://tailscale.com/install.sh | sh

# Authenticate
sudo tailscale up --hostname=nomad-groundstation

# Verify
tailscale ip -4
ping <jetson-tailscale-ip>
```

**macOS:**

1. Download from https://tailscale.com/download/mac
2. Install and login
3. Verify connection

---

## Configuration

### MAVLink Router Integration

Update [mavlink-router main.conf](../transport/mavlink_router/main.conf):

```ini
[UdpEndpoint groundstation]
Mode=Normal
Address=100.y.y.y  # Ground Station Tailscale IP
Port=14560
```

**Find Ground Station IP:**

```bash
# On Ground Station
tailscale ip -4
```

Replace `100.y.y.y` in the config with this IP.

### Mission Planner Connection

**Connect through the NOMAD plugin router:**

1. Open Mission Planner
2. Select connection type: **UDP**
3. Port: **14600**
4. Click **Connect**
5. Mission Planner will listen to the plugin's merged stream

**Note:** The Jetson pushes LTE telemetry to the Ground Station on UDP 14560. The RadioMaster feed uses UDP 14550. The NOMAD plugin combines both and publishes the selected merged stream locally on UDP 14600.

### API Access

**Access Jetson Orchestrator API:**

```bash
# From Ground Station
curl http://<jetson-tailscale-ip>:8000/health

# Example:
curl http://100.100.10.5:8000/health
```

**Browser Access:**

Open Swagger UI:
```
http://<jetson-tailscale-ip>:8000/docs
```

### SSH Access

**Remote Terminal Access:**

```bash
# From Ground Station
ssh <jetson-user>@<jetson-tailscale-ip>

# Example:
ssh <jetson-user>@100.100.10.5
```

**Use Cases:**
- View logs: `journalctl -u nomad-edge-core -f`
- Deploy code: `git pull`
- Restart services: `nomad restart all` (or `sudo systemctl restart nomad.target`)
- Monitor resources: `jtop`

### RTSP Video Streaming

**Access ZED Camera Feed (single stream):**

```
rtsp://<jetson-tailscale-ip>:8554/stream
```

**VLC Player:**
```bash
# Open video stream
vlc rtsp://100.100.10.5:8554/stream
```

**Mission Planner Plugin:**

The NOMAD plugin automatically uses the configured Tailscale IP for video streams.

---

## Security Best Practices

### 1. Access Control Lists (ACLs)

Restrict access between devices:

https://login.tailscale.com/admin/acls

Example ACL:
```json
{
  "acls": [
    {
      "action": "accept",
      "src": ["tag:groundstation"],
      "dst": ["tag:jetson:*"]
    }
  ],
  "tagOwners": {
    "tag:jetson": ["your-email@example.com"],
    "tag:groundstation": ["your-email@example.com"]
  }
}
```

### 2. Key Expiry

Set auth key expiry to limit compromised key exposure:

```bash
# Generate 90-day expiry key
tailscale up --authkey=<key> --auth-key-expiry=90d
```

### 3. SSH Hardening

Disable password authentication:

```bash
# On Jetson
sudo nano /etc/ssh/sshd_config

# Set:
PasswordAuthentication no
PubkeyAuthentication yes

# Restart SSH
sudo systemctl restart sshd
```

### 4. Firewall Configuration

Allow only necessary ports:

```bash
# On Jetson
sudo ufw enable
sudo ufw allow from 100.0.0.0/8 to any port 22 proto tcp     # SSH
sudo ufw allow from 100.0.0.0/8 to any port 8000 proto tcp   # API
sudo ufw allow from 100.0.0.0/8 to any port 8554 proto tcp   # RTSP
sudo ufw allow from 100.0.0.0/8 to any port 14560 proto udp  # MAVLink LTE
```

---

## Troubleshooting

### Issue: Tailscale Not Connecting

**Check Service Status:**
```bash
sudo systemctl status tailscaled
sudo journalctl -u tailscaled -n 50
```

**Check Firewall:**
```bash
# Allow Tailscale ports (UDP 41641, ephemeral ports)
sudo ufw allow in on tailscale0
```

**Restart Tailscale:**
```bash
sudo systemctl restart tailscaled
sudo tailscale down
sudo tailscale up
```

### Issue: High Latency

**Check Network Path:**
```bash
# From Ground Station
ping <jetson-tailscale-ip>
traceroute <jetson-tailscale-ip>

# Check Tailscale status
tailscale status --json
```

**Enable DERP Relay:**

If direct connection fails, Tailscale falls back to DERP relay servers. Check if relay is in use:

```bash
tailscale status | grep "relay"
```

**Optimize Cellular Connection:**

- Use LTE Cat-6+ modem (>10 Mbps uplink)
- Ensure good signal strength (RSRP > -100 dBm)
- Consider external antenna

### Issue: Cannot Access Jetson API

**Verify Service is Running:**
```bash
# On Jetson
sudo systemctl status nomad-edge-core
curl http://127.0.0.1:8000/health
```

**Check Firewall:**
```bash
# On Jetson
sudo ufw status
sudo ufw allow from 100.0.0.0/8 to any port 8000 proto tcp
```

**Test from Jetson:**
```bash
# On Jetson
curl http://$(tailscale ip -4):8000/health
```

### Issue: MAVLink Not Received

**Check mavlink-router:**
```bash
# On Jetson
sudo systemctl status mavlink-router
sudo journalctl -u mavlink-router -f

# Verify Ground Station IP
sudo nano /etc/mavlink-router/main.conf
```

**Verify MAVLink Traffic:**
```bash
# On Ground Station
sudo tcpdump -i any port 14560 -vv
```

**Check Windows Firewall:**

```powershell
# On Ground Station (PowerShell as Admin)
New-NetFirewallRule -DisplayName "MAVLink LTE UDP" -Direction Inbound -Protocol UDP -LocalPort 14560 -Action Allow
```

---

## Performance Optimization

### Bandwidth Considerations

**MAVLink Telemetry:** ~5-10 KB/s (negligible)
**RTSP Video (480p):** ~1-2 Mbps
**RTSP Video (720p):** ~3-5 Mbps
**API Polling:** <1 KB/s

**Recommendation:** Use 480p video over cellular to conserve bandwidth and reduce latency.

### Latency Targets

- **MAVLink:** <100ms (acceptable for monitoring, not for real-time control)
- **Video:** <500ms (acceptable for situational awareness)
- **API:** <200ms (sufficient for health monitoring)

**Note:** Primary flight control should use ELRS RC link, not Tailscale/MAVLink.

---

## Network Resilience

### Auto-Reconnect

Tailscale automatically handles:
- Network interface changes (WiFi ↔ LTE)
- IP address changes (DHCP, cellular tower handoff)
- Temporary disconnections

### Connection Monitoring

The repo ships a connection watchdog — [scripts/watchdog.sh](scripts/watchdog.sh)
run by [config/tailscale-watchdog.service](config/tailscale-watchdog.service).
`scripts/setup.sh` installs and enables it; check it with:

```bash
sudo systemctl status tailscale-watchdog
sudo journalctl -u tailscale-watchdog -f
```

---

## Competition Day Checklist

### Pre-Flight

- [ ] Verify Tailscale is connected on Jetson: `tailscale status`
- [ ] Verify Tailscale is connected on Ground Station
- [ ] Test ping from Ground Station to Jetson
- [ ] Test MAVLink telemetry reception in Mission Planner
- [ ] Test API access: `curl http://<jetson-ip>:8000/health`
- [ ] Test video stream in VLC or Mission Planner plugin
- [ ] Verify 4G/LTE signal strength (RSRP > -100 dBm)

### During Flight

- [ ] Monitor Tailscale connection status
- [ ] Watch for high latency (ping >200ms may indicate issues)
- [ ] Use WASD nudge control only for minor corrections
- [ ] Rely on ELRS RC link for primary control

### Post-Flight

- [ ] Download mission logs via SSH
- [ ] Review telemetry and video recordings
- [ ] Check Tailscale connection statistics

---

## Summary

**Jetson Commands:**
```bash
# Install
curl -fsSL https://tailscale.com/install.sh | sh
sudo tailscale up --authkey=<key> --hostname=nomad-jetson

# Status
tailscale status
tailscale ip -4

# Restart
sudo systemctl restart tailscaled
```

**Ground Station Commands:**
```bash
# Install (Windows)
# Download from https://tailscale.com/download/windows

# Status
tailscale status

# Test
ping <jetson-tailscale-ip>
curl http://<jetson-tailscale-ip>:8000/health
```

**Mission Planner:**
- Main connection: UDP, Port 14600
- NOMAD plugin LTE input: UDP, Port 14560
- NOMAD plugin RadioMaster input: UDP, Port 14550
- Jetson API: `http://<jetson-tailscale-ip>:8000`
- Video: `rtsp://<jetson-tailscale-ip>:8554/stream`

**Configuration Files:**
- Jetson: `/etc/mavlink-router/main.conf` (update Ground Station IP)
- Ground Station: Mission Planner plugin settings (update Jetson IP)

---

**Next Steps:**

1. Install Tailscale on both devices
2. Note Tailscale IPs on both devices
3. Update mavlink-router config with Ground Station IP
4. Update Mission Planner plugin with Jetson IP
5. Test connectivity before first flight
