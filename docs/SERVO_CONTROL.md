# Servo Control Architecture

NOMAD servo and pump outputs are driven through the Cube Orange over MAVLink.
The Jetson does not drive local GPIO or PWM lines for servos.

## Hardware

| Component | Detail |
|-----------|--------|
| Payload servos | Cube Orange servo outputs, commanded with `MAV_CMD_DO_SET_SERVO` |
| ZED camera tilt | Cube Orange servo output configured by Mission Planner |
| Water pump | Cube Orange relay output, commanded with `MAV_CMD_DO_SET_RELAY` |
| Jetson role | Sends API/ROS/autonomy commands to the Cube through MAVLink only |

Mission Planner is the source of truth for channel mapping. On connect it pushes
the camera tilt configuration to Edge Core with `/api/servo/camera/config`.
Spray calibration pushes the pump relay number and spray duration through
`/api/spray/calibration`.

## Control Flow

```text
Mission Planner / ROS / autonomy
        |
        | HTTP API
        v
Edge Core servo_controller.py
        |
        | MAVLink through mavlink-router
        v
Cube Orange
        |
        +-- SERVOx output -> payload / tilt servo
        +-- RELAYx output -> water pump
```

## API Endpoints

| Method | Endpoint | Description |
|--------|----------|-------------|
| `GET` | `/api/servo/status` | Get Cube servo and relay status |
| `POST` | `/api/servo/channel/{channel}/pwm?pwm={us}` | Set Cube servo output PWM |
| `POST` | `/api/servo/camera/tilt?angle={0-180}` | Set camera/nozzle tilt angle |
| `GET` | `/api/servo/camera/tilt` | Get current camera/nozzle tilt feedback |
| `GET` | `/api/servo/camera/config` | Get tilt channel and calibration |
| `POST` | `/api/servo/camera/config` | Apply tilt channel and calibration from Mission Planner |
| `POST` | `/api/servo/enable` | Enable tracked Cube servo outputs |
| `POST` | `/api/servo/disable` | Disable tracked Cube servo outputs |
| `POST` | `/api/servo/shooter/trigger?duration_ms={ms}` | Trigger water pump through Cube relay |

## Manual Control

Mission Planner normally sends `DO_SET_SERVO`/`DO_SET_RELAY` directly to the
flight controller. If that direct link is unavailable, the plugin falls back to
the Edge Core API above, which sends the same MAVLink commands through the
Jetson's flight-controller link.

## ROS / Autonomy Control

The ROS HTTP bridge subscribes to `/nomad/servo/nozzle_angle` and forwards angle
commands to:

```text
POST /api/servo/camera/tilt?angle={degrees}
```

The Edge Core spray controller also uses the same camera tilt API and triggers
the pump through the configured Cube relay during the spray sequence.

## Requirements

For each Cube output used as a raw servo command, configure ArduPilot with:

```text
SERVOx_FUNCTION = 0
```

The water pump relay must be configured in ArduPilot according to the selected
Mission Planner `WaterPumpRelayNumber` value.
