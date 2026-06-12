# NOMAD Tailscale Integration

Tailscale VPN configuration and code for secure 4G/LTE communication between
the Jetson (drone) and the Ground Station: MAVLink telemetry, the Edge Core
HTTP API, SSH, and RTSP video all ride the same WireGuard tunnel.

## Layout

```
tailscale/
├── README.md               # This file (architecture + ports)
├── SETUP.md                # Installation & configuration guide
├── scripts/
│   ├── setup.sh            # Automated Jetson setup (install + auth + watchdog)
│   └── watchdog.sh         # Connection watchdog (restarts tailscaled on loss)
├── config/
│   └── tailscale-watchdog.service  # Systemd unit for watchdog.sh
├── network_monitor.py      # LTE/connectivity monitoring (used by edge_core network module)
└── tailscale_manager.py    # Tailscale status polling (used by edge_core network module)
```

The Python modules are imported by `edge_core/services/network_module.py`
(`from infra.tailscale import NetworkMonitor, TailscaleManager`), are
mypy-checked, and are covered by `tests/test_network_module.py`.

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                       Tailscale Cloud                        │
│                    (Coordination Server)                     │
└─────────────────────────────────────────────────────────────┘
                           │
           ┌───────────────┴───────────────┐
           │                               │
    ┌──────▼──────┐               ┌───────▼──────┐
    │   Jetson    │               │    Ground    │
    │ Orin Nano   │◄─────────────►│   Station    │
    │ (on drone)  │  Encrypted    │   (Laptop)   │
    │             │  WireGuard    │              │
    │ 4G/LTE USB  │    Tunnel     │  WiFi/LTE    │
    └─────────────┘               └──────────────┘
    100.x.x.x                     100.y.y.y
```

## Ports & Endpoints

| Service | Port | Protocol | Description |
|---------|------|----------|-------------|
| MAVLink LTE/Tailscale | 14560 | UDP | Telemetry to Mission Planner |
| MAVLink RadioMaster | 14550 | UDP | Local radio link on Ground Station |
| MAVLink Plugin Router | 14600 | UDP | Local merged stream for Mission Planner |
| HTTP API | 8000 | TCP | Edge Core REST API |
| SSH | 22 | TCP | Remote terminal |
| RTSP Primary | 8554 | TCP | ZED left camera stream |
| RTSP Secondary | 8554 | TCP | ZED right camera stream |

## Quick Start

```bash
# On Jetson
sudo infra/tailscale/scripts/setup.sh --authkey <YOUR_KEY>

# Verify
tailscale status
tailscale ip -4
```

See [SETUP.md](SETUP.md) for ground-station setup, MAVLink router integration,
security hardening, and troubleshooting.
