# Transport Layer

Networking and routing primitives.

## mavlink_router/

Splits the FC UART (921600 baud) to local UDP endpoints and the ground VPN.
`main.conf` is the single source of truth; `scripts/setup/setup_jetson.sh`
copies it to `/etc/mavlink-router/main.conf` on the Jetson.

Outputs:

- `127.0.0.1:14550` — Orchestrator (Edge Core)
- `127.0.0.1:14551` — Vision/VIO process
- `<GCS_IP>:14560` — Ground Station NOMAD plugin LTE input (via Tailscale VPN)
- `14550` stays reserved for the RadioMaster/local MAVLink input on the ground station
- `127.0.0.1:14600` on the ground station is the NOMAD plugin's merged output that Mission Planner connects to

Manual install:

```bash
sudo cp mavlink_router/main.conf /etc/mavlink-router/main.conf
sudo systemctl enable --now mavlink-router
```
