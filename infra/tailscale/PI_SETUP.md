# Tailscale Setup — Ground Station (Raspberry Pi)

This document describes how to install and configure Tailscale on a Raspberry Pi ground station used with NOMAD.

## Supported OS
- Raspberry Pi OS (Bullseye / Bookworm) — 32-bit or 64-bit
- Ubuntu Server for Raspberry Pi (20.04 / 22.04)

## Quick Summary
1. Install Tailscale
2. Start and enable the tailscaled service
3. Authenticate (interactive or with an auth key)
4. Verify connectivity and test SSH, API, and MAVLink

## 1) Install Tailscale
Run the official install script (recommended):

```bash
# Update and install
sudo apt update && sudo apt -y upgrade
curl -fsSL https://tailscale.com/install.sh | sh
```

Alternative: for offline or locked environments, download the appropriate package from:
https://tailscale.com/download and install with `dpkg -i`.

## 2) Start and enable the service

```bash
sudo systemctl enable --now tailscaled
sudo systemctl status tailscaled
```

If you need Tailscale to wait for networking (useful on cellular links):

```bash
sudo systemctl edit tailscaled
# Add:
# [Unit]
# After=network-online.target
# Wants=network-online.target

sudo systemctl daemon-reload
sudo systemctl restart tailscaled
```

## 3) Authenticate (choose one)

Interactive (opens login link/QR in the terminal):

```bash
sudo tailscale up --hostname=nomad-groundstation-pi
```

Headless / automated (recommended for deployments):

1. Create an auth key in the admin console: `https://login.tailscale.com/admin/settings/keys`
   - Use a short expiry or reusable key depending on your policy.
2. Run on the Pi:

```bash
sudo tailscale up --authkey <AUTH_KEY> --hostname=nomad-groundstation-pi
```

For advanced setups (accept routes, enable exit node):

```bash
sudo tailscale up --authkey <AUTH_KEY> --hostname=nomad-groundstation-pi --accept-routes
```

## 4) Verify

```bash
tailscale status
tailscale ip -4
# Example: 100.x.y.z
ping <jetson-tailscale-ip>
```

## 5) SSH and services
- Ensure OpenSSH server is installed: `sudo apt install -y openssh-server`.
- SSH by Tailscale IP or MagicDNS name (if enabled):
  - `ssh pi@100.x.y.z` or `ssh pi@nomad-groundstation-pi.tailnet-yourname.ts.net`

## 6) Firewall and UFW
If using UFW:

```bash
sudo ufw allow from 100.0.0.0/8 to any port 22 proto tcp   # SSH
sudo ufw allow from 100.0.0.0/8 to any port 8000 proto tcp # API
sudo ufw allow from 100.0.0.0/8 to any port 14560 proto udp # MAVLink LTE
sudo ufw enable
```

## 7) Using MagicDNS and ACLs
- Enable MagicDNS and ACLs from the Tailscale admin console (login.tailscale.com).
- Use tags for Jetson / Groundstation and write ACLs to restrict access.

## 8) Testing with NOMAD
- From the Pi (or remote ground station), test API and MAVLink connectivity to the Jetson:

```bash
curl http://<jetson-tailscale-ip>:8000/health
nc -uzv <jetson-tailscale-ip> 14560
```

## 9) Troubleshooting
- `sudo systemctl status tailscaled`
- `sudo journalctl -u tailscaled -n 200 --no-pager`
- If high latency/relay: `tailscale status --json` and check for `derp` usage

## 10) Security notes
- Prefer ephemeral or short-lived auth keys for field units.
- Use ACLs to restrict SSH/API to only ground station hosts.
- Make sure system packages and kernel are up to date.

---

If you want, I can add a short script to `tailscale/scripts` that automates install + authkey usage for headless deployment (requires you to supply a key or store it securely).
