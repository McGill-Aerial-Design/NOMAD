# MAVLink Router Configuration

## Purpose
Split FC UART (921600) to local UDP endpoints and ground VPN.

## Outputs
- `127.0.0.1:14550` - Orchestrator (Edge Core)
- `127.0.0.1:14551` - Vision/VIO Process
- `<GCS_IP>:14560` - Ground Station NOMAD plugin LTE input (via Tailscale VPN / LTE)
- `14550` remains reserved for the RadioMaster/local MAVLink input on the ground station.
- `127.0.0.1:14600` on the ground station is the NOMAD plugin's merged output that Mission Planner connects to.

## Configuration
- **Primary config file:** `main.conf` - This is the single source of truth
- Copy to `/etc/mavlink-router/main.conf` on the Jetson

## Installation
```bash
sudo cp main.conf /etc/mavlink-router/main.conf
sudo systemctl enable mavlink-router
sudo systemctl start mavlink-router
```
