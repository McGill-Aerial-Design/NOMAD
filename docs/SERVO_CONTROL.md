# Servo Control Architecture

This document describes the nozzle servo control system for NOMAD, covering both manual control from Mission Planner and autonomous control from ROS.

## Hardware

| Component | Detail |
|-----------|--------|
| **Servo** | Standard hobby servo (50Hz PWM, 500-2500us pulse range) |
| **Signal Pin** | Jetson Orin Nano Pin 15 (gpiochip0, line 85 / SOC_GPIO39_PN1) |
| **Power** | Pin 2 or Pin 4 (5V rail) -- or external 5V BEC for high-torque servos |
| **Ground** | Pin 14 (any GND pin works: 6, 9, 14, 20, 25, 30, 34, 39) |

**Note:** Hardware PWM (sysfs pwmchip0) does not route to Pin 15 due to Jetson Orin Nano pinmux limitations. The system uses a compiled C helper that bit-bangs 50Hz PWM via the `/dev/gpiochip0` character device (GPIO line 85).

## Architecture Overview

```
  +-------------------+         +--------------------------+
  | Mission Planner   |         | Isaac ROS Docker         |
  | (Windows GCS)     |         | (Jetson)                 |
  |                   |         |                          |
  | Nozzle slider     |         | Fire detection node      |
  | (0-180 degrees)   |         | publishes Float32 angle  |
  +--------+----------+         +----------+---------------+
           |                               |
           | HTTP POST                     | ROS topic
           | /api/servo/camera/tilt        | /nomad/servo/nozzle_angle
           |                               |
           |         +--------------------+|
           |         | ros_http_bridge.py  ||
           |         | (inside Docker)     ||
           |         +--------+-----------+|
           |                  |             
           |                  | HTTP POST   
           |                  | /api/servo/camera/tilt
           v                  v             
    +------+------------------+--------+
    |     Edge Core (port 8000)        |
    |     api.py -> servo_controller   |
    +------+---------------------------+
           |
           | subprocess stdin
           v
    +------+---------------------------+
    | nomad_servo_pwm (C helper)       |
    | Bit-bang 50Hz PWM via            |
    | /dev/gpiochip0 line 85 (Pin 15)  |
    +----------------------------------+
           |
           | GPIO signal
           v
      [ Physical Servo ]
```

## Control Methods

### 1. Manual Control (Mission Planner)

The WASD control panel in Mission Planner includes a nozzle angle slider (0-180 degrees). When the slider moves, it sends:

```
POST http://{jetson_ip}:8000/api/servo/camera/tilt?angle={degrees}
```

**Source:** `mission_planner/src/EnhancedWASDControl.cs` -- `UpdateNozzleServo()` method.

### 2. Autonomous Control (ROS)

A ROS node inside the Isaac ROS Docker container publishes a `std_msgs/Float32` message to the `/nomad/servo/nozzle_angle` topic. The `ros_http_bridge.py` node subscribes and forwards the angle to Edge Core via the same HTTP API.

**Publishing from any ROS2 node:**

```python
from std_msgs.msg import Float32

# In your node's __init__:
self.servo_pub = self.create_publisher(Float32, '/nomad/servo/nozzle_angle', 10)

# To set the nozzle angle:
msg = Float32()
msg.data = 45.0  # degrees (0-180, 90 = center)
self.servo_pub.publish(msg)
```

**Quick test from command line (inside Docker):**

```bash
ros2 topic pub /nomad/servo/nozzle_angle std_msgs/msg/Float32 '{data: 90.0}' --once
```

**Bridge configuration:**

```bash
# Default (servo enabled on /nomad/servo/nozzle_angle):
python3 ros_http_bridge.py --host 172.17.0.1 --port 8000

# Custom servo topic:
python3 ros_http_bridge.py --servo-topic /my_custom/servo_angle

# Disable servo bridging:
python3 ros_http_bridge.py --disable-servo
```

### 3. Direct API (curl / any HTTP client)

```bash
# Set nozzle to 90 degrees (center)
curl -X POST 'http://100.75.218.89:8000/api/servo/camera/tilt?angle=90'

# Set nozzle to 0 degrees
curl -X POST 'http://100.75.218.89:8000/api/servo/camera/tilt?angle=0'

# Check servo status
curl http://100.75.218.89:8000/api/servo/status
```

## Edge Core API Endpoints

| Method | Endpoint | Description |
|--------|----------|-------------|
| `POST` | `/api/servo/camera/tilt?angle={0-180}` | Set nozzle servo angle |
| `POST` | `/api/servo/shooter/trigger?duration_ms={ms}` | Trigger water shooter GPIO |
| `GET` | `/api/servo/status` | Get servo + GPIO status |

## Servo Controller Details

**File:** `edge_core/servo_controller.py`

The `GPIOPWMServo` class manages the servo through a C helper process:

1. On first `set_angle()` call, it compiles `/tmp/nomad_servo_pwm.c` into `/tmp/nomad_servo_pwm`
2. Spawns the C helper as a subprocess with `stdin`/`stdout` pipes
3. The helper opens `/dev/gpiochip0`, acquires GPIO line 85, and bit-bangs 50Hz PWM
4. New angles are sent via stdin as pulse widths in microseconds
5. The helper runs until terminated (on Edge Core shutdown)

**Angle to pulse mapping:**
- 0 degrees = 500us pulse
- 90 degrees = 1500us pulse (center)
- 180 degrees = 2500us pulse

**Rate limiting:**
- The ROS bridge rate-limits servo commands to 10 Hz
- Angle changes smaller than 0.5 degrees are ignored to avoid flooding

## Wiring Diagram

```
Jetson 40-Pin Header (left column, looking at board with USB ports facing you):

  Pin 1  (3.3V)     Pin 2  (5V) ---- Servo VCC (red)
  Pin 3  (I2C SDA)  Pin 4  (5V)
  Pin 5  (I2C SCL)  Pin 6  (GND)
  Pin 7  (GPIO)     Pin 8  (UART TX)
  Pin 9  (GND)      Pin 10 (UART RX)
  Pin 11 (GPIO)     Pin 12 (I2S CLK)
  Pin 13 (GPIO)     Pin 14 (GND) --- Servo GND (brown/black)
  Pin 15 (PWM) ---- Servo Signal (orange/white)
  Pin 16 (GPIO)     Pin 17 (3.3V)
  ...
```

**Important:** If using an external 5V power supply for the servo, connect its ground to a Jetson GND pin (common ground reference required for the PWM signal).

## Troubleshooting

**Servo not moving via API:**
1. Check Edge Core is running: `curl http://100.75.218.89:8000/health`
2. Check servo status: `curl http://100.75.218.89:8000/api/servo/status`
3. Check the helper process: `ps aux | grep nomad_servo`
4. Check Edge Core logs: `tail -20 /tmp/edge_core.log`

**Servo moves via test script but not via API:**
- The Edge Core runs as user `mad` who must be in the `gpio` group
- Check: `id mad | grep gpio`
- Fix: `sudo usermod -aG gpio mad`

**API call hangs or times out:**
- The servo controller has a 5-second timeout on subprocess startup
- Check if `/tmp/nomad_servo_pwm` binary exists and is executable
- Check: `ls -la /tmp/nomad_servo_pwm`

**ROS bridge not forwarding servo commands:**
- Verify the bridge is running with servo enabled (check `--disable-servo` is not set)
- Check bridge stats: look for `servo_received` and `servo_sent` counts
- Test topic directly: `ros2 topic echo /nomad/servo/nozzle_angle`
