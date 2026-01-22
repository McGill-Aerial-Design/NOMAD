# MAVLink Dual Link Failover System

This document describes the MAVLink dual link failover system for NOMAD, which automatically switches between LTE (Tailscale) and RadioMaster connections for maximum reliability.

## Architecture Overview

```
┌─────────────────┐     ┌──────────────────┐     ┌─────────────────┐
│   Cube Orange   │────▶│  Jetson Orin     │────▶│  Mission Planner│
│  Flight Controller   │USB│  (mavlink-router)│ LTE │  (Primary Link) │
└─────────────────┘     └──────────────────┘     └─────────────────┘
        │                                               ▲
        │                                               │
        ▼                                               │
┌─────────────────┐                            ┌───────┴───────┐
│  RadioMaster    │─────────────────────────▶ │ Mission Planner│
│  Transmitter    │     2.4GHz ELRS           │ (Backup Link)  │
└─────────────────┘     UDP 14550             └───────────────┘
```

## Connection Paths

### Primary Link: LTE via Tailscale
- **Path**: Cube (USB) → Jetson (mavlink-router) → Tailscale VPN → Ground Station
- **Port**: UDP 14550
- **Typical Latency**: 50-200ms
- **Range**: Unlimited (cellular coverage)
- **Best For**: Long-range missions, beyond visual line of sight (BVLOS)

### Secondary Link: RadioMaster
- **Path**: Cube (ELRS receiver) → RadioMaster TX → USB → Ground Station
- **Port**: UDP 14550 (configurable)
- **Typical Latency**: 10-30ms
- **Range**: ~1-2km (ELRS dependent)
- **Best For**: Low-latency control, backup when LTE fails

## Setup Instructions

### 1. Jetson Configuration (mavlink-router)

The Jetson runs `mavlink-router` to forward MAVLink from the Cube to Mission Planner:

```bash
# Install mavlink-router
sudo apt install mavlink-router

# Copy config
sudo cp transport/mavlink_router/main.conf /etc/mavlink-router/main.conf

# Edit config to set your Ground Station Tailscale IP
sudo nano /etc/mavlink-router/main.conf

# Enable and start
sudo systemctl enable mavlink-router
sudo systemctl start mavlink-router
```

Key config settings in `/etc/mavlink-router/main.conf`:
```ini
[UartEndpoint usb_fc]
Device=/dev/ttyACM0    # Cube USB port
Baud=921600

[UdpEndpoint groundstation]
Mode=Normal
Address=100.x.x.x      # Your ground station Tailscale IP
Port=14550
```

### 2. RadioMaster Configuration

Configure your RadioMaster TX to output MAVLink via USB:

1. In ELRS Lua script: Enable MAVLink passthrough
2. Connect RadioMaster to ground station via USB
3. Mission Planner receives telemetry on UDP 14550

### 3. Mission Planner Configuration

#### Option A: Dual UDP Connections (Recommended)
1. **First Connection (LTE)**:
   - UDP, Local Port 14550
   - This receives data from Jetson via Tailscale

2. **Second Connection (RadioMaster)**:
   - UDP, Local Port 14551 (or connect RadioMaster serial directly)
   - Set RadioMaster to output on this port

#### Option B: Single Port with External Routing
- Use a local UDP router to merge both streams to port 14550
- NOMAD plugin detects source by packet characteristics

### 4. NOMAD Plugin Configuration

In Mission Planner, go to **NOMAD Menu → Settings**:

1. **Enable Dual Link Management**: ✓
2. **RadioMaster Port**: 14550 (or your configured port)
3. **Enable Automatic Failover**: ✓
4. **Preferred Link**: LTE (Tailscale) - primary for long range
5. **Auto-reconnect to preferred**: ✓ - returns to LTE when available

## Link Health Monitoring

Access via **NOMAD Menu → Link Status** or right-click map → **NOMAD Link Status**

### Health Indicators

| Status | Latency | Packet Loss | Action |
|--------|---------|-------------|--------|
| 🟢 Excellent | < 50ms | < 0.5% | Optimal |
| 🟢 Good | < 150ms | < 2% | Normal operation |
| 🟡 Fair | < 300ms | < 10% | Monitor closely |
| 🟠 Poor | < 500ms | < 25% | Consider switching |
| 🔴 Critical | > 500ms | > 25% | Auto-failover triggers |
| ⭕ Disconnected | - | - | Link unavailable |

### Automatic Failover

Failover triggers when:
1. **Primary link**: No heartbeat for 3+ seconds (3 missed heartbeats)
2. **Primary link**: Critical health status
3. **Better link available**: Significant health improvement on alternate link

Failover **back** to preferred link when:
1. Preferred link healthy for 10+ seconds (configurable)
2. Auto-reconnect enabled

## Testing Dual Link

### Test LTE Link
```bash
# On Jetson
ping -c 5 <ground-station-tailscale-ip>

# Check mavlink-router status
sudo systemctl status mavlink-router

# Monitor MAVLink traffic
mavproxy.py --master=/dev/ttyACM0 --baudrate=921600
```

### Test RadioMaster Link
1. Disconnect LTE/Tailscale temporarily
2. Verify Mission Planner still receives telemetry
3. Check Link Status panel shows RadioMaster as active

### Simulate Failover
1. Open Link Status panel
2. Disable LTE (disconnect Tailscale or stop mavlink-router)
3. Observe automatic switch to RadioMaster
4. Re-enable LTE
5. Observe automatic return to LTE (after delay)

## Troubleshooting

### No LTE Connection
- Check Tailscale: `tailscale status`
- Verify Jetson IP: `tailscale ip -4`
- Check mavlink-router: `sudo journalctl -u mavlink-router -f`
- Test connectivity: `ping <jetson-tailscale-ip>`

### No RadioMaster Connection
- Verify RadioMaster USB connection
- Check ELRS MAVLink passthrough is enabled
- Verify correct COM port / UDP port
- Check Mission Planner connection settings

### Frequent Failovers
- Check network stability (latency/jitter)
- Increase heartbeat timeout in settings
- Check for RF interference on ELRS link
- Review Link Status log for patterns

### Links Not Detected
- Ensure both connections are established in Mission Planner
- Verify NOMAD plugin is loaded (NOMAD menu visible)
- Check plugin debug log: Enable Debug Mode in settings

## API Reference

### MAVLinkConnectionManager

```csharp
// Get current active link
LinkType active = plugin.ConnectionManager.ActiveLink;

// Manually switch links
bool success = plugin.ConnectionManager.SwitchToLink(LinkType.RadioMaster);

// Get link statistics
LinkStatistics lteStats = plugin.ConnectionManager.LteStatistics;
LinkStatistics rmStats = plugin.ConnectionManager.RadioMasterStatistics;

// Subscribe to failover events
plugin.ConnectionManager.FailoverOccurred += (s, e) => {
    Console.WriteLine($"Failover: {e.FromLink} → {e.ToLink}");
};
```

### Link Status Panel Integration

The LinkHealthPanel can be embedded in custom forms:

```csharp
var panel = new LinkHealthPanel(connectionManager, config);
panel.Dock = DockStyle.Fill;
myForm.Controls.Add(panel);
```

## Version History

- **v2.1.0**: Initial dual link failover implementation
  - MAVLinkConnectionManager for link monitoring
  - LinkHealthPanel UI component
  - Automatic failover logic
  - Settings integration
