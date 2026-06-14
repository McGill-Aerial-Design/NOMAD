# Infra

Device-side deployment: systemd units, host configs, and the Tailscale VPN
integration. (CI workflows live in `.github/workflows/`, not here.)

| Path | Purpose |
|------|---------|
| `systemd/` | Per-service units + `install.sh` (renders, installs, enables per `NOMAD_AUTOSTART_*` flags) |
| `transport/mavlink_router/` | mavlink-router config — FC UART fan-out to local UDP + ground VPN |
| `tailscale/` | Tailscale VPN: setup/watchdog scripts + Python monitors used by `edge_core` (see its README) |
| `mediamtx.yml` | MediaMTX RTSP server config (path set via `MEDIAMTX_CONFIG` in `config/nomad.env`) |
| `logrotate.conf` | Log rotation for `~/nomad_logs` (installed by `systemd/install.sh`) |

Install everything on the Jetson with:

```bash
sudo bash infra/systemd/install.sh
```
