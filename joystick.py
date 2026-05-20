"""
NOMAD Serial → Virtual Xbox 360 Bridge
======================================

Reads CSV telemetry from a microcontroller (RadioMaster passthrough,
gimbal stick MCU, etc.) over a serial port and emits a virtual Xbox 360
controller via vgamepad/ViGEmBus. Once running, Windows + Mission
Planner see it as a regular DirectInput device, so the NOMAD plugin's
NomadJoystickService can pick it up alongside any other gamepad.

Run directly:
    python jotystick.py --port COM10 --baud 115200

Or let the NOMADPlugin auto-launch it (see "Serial Bridge" section in
NOMAD Settings → Joystick).

Wire format (CSV, one frame per newline-terminated line):
    roll,pitch,yaw,throttle,gimbal_x,gimbal_y,sw1,sw2,sw3,mode[,kill]

Sticks and triggers are 0..1023 (10-bit ADC); switches are 0/1/2 (three-
position); mode is 0..3 (DPAD); kill is 0/1 (optional 11th column).
"""

from __future__ import annotations

import argparse
import signal
import sys
import time

try:
    import serial
except ImportError:
    print("ERROR: pyserial not installed. Run: python -m pip install pyserial",
          file=sys.stderr)
    sys.exit(2)

try:
    import vgamepad as vg
except ImportError:
    print("ERROR: vgamepad not installed. Run: python -m pip install vgamepad",
          file=sys.stderr)
    print("       (vgamepad also needs the ViGEmBus driver: https://github.com/ViGEm/ViGEmBus/releases)",
          file=sys.stderr)
    sys.exit(2)


# =========================
# SCALE FUNCTIONS
# =========================

def scale_stick(v) -> int:
    """10-bit (0..1023) → signed 16-bit (-32768..32767)."""
    try:
        v = max(0, min(1023, int(v)))
        return int((v / 1023) * 65535 - 32768)
    except (ValueError, TypeError):
        return 0


def scale_trigger(v) -> int:
    """10-bit (0..1023) → unsigned 8-bit (0..255)."""
    try:
        v = max(0, min(1023, int(v)))
        return int((v / 1023) * 255)
    except (ValueError, TypeError):
        return 0


# =========================
# MAIN LOOP
# =========================

def run(port: str, baud: int, verbose: bool) -> int:
    gamepad = vg.VX360Gamepad()

    # Initial neutral state so consumers don't see a stale frame from a
    # previous run (vgamepad keeps state until process exit, but a fresh
    # update() makes our intent explicit).
    gamepad.update()

    ser = None
    print(f"NOMAD bridge: opening {port} @ {baud}…", flush=True)

    # Keep retrying on missing/disconnected port so the plugin can auto-launch
    # us before the radio is plugged in.
    while ser is None:
        try:
            ser = serial.Serial(port, baud, timeout=0.05)
        except serial.SerialException as e:
            print(f"NOMAD bridge: serial open failed ({e}); retrying in 2s…", flush=True)
            time.sleep(2)

    time.sleep(2)
    try:
        ser.reset_input_buffer()
    except Exception:
        pass

    print("NOMAD bridge: bridge active. Ctrl+C to quit.", flush=True)

    last_print = 0.0
    PRINT_EVERY = 0.5  # seconds, when --verbose is off

    while True:
        try:
            raw = ser.readline()
            if not raw:
                continue
            line = raw.decode(errors="ignore").strip()
            if not line:
                continue

            if verbose:
                print(line, flush=True)
            else:
                now = time.monotonic()
                if now - last_print >= PRINT_EVERY:
                    print(line, flush=True)
                    last_print = now

            parts = line.split(",")
            if len(parts) < 10:
                continue
            parts = parts[:11]  # truncate any extras safely

            # -------- AXES --------
            roll     = scale_stick(parts[0])
            pitch    = scale_stick(parts[1])
            yaw_raw  = parts[2]  # mapped to right_trigger below so DirectInput sees yaw as an axis
            throttle = scale_trigger(parts[3])

            gimbal_x = scale_stick(parts[4])
            gimbal_y = scale_stick(parts[5])

            # -------- SWITCHES --------
            try:
                sw1  = int(parts[6])
                sw2  = int(parts[7])
                sw3  = int(parts[8])
                mode = int(parts[9])
            except ValueError:
                continue

            kill = 0
            if len(parts) > 10:
                try:
                    kill = int(parts[10])
                except ValueError:
                    kill = 0

            # -------- AXES OUTPUT --------
            # Left stick = vehicle roll/pitch (flight)
            # Right stick = gimbal pan/tilt (NomadJoystickService reads Rx/Ry)
            # Left trigger = throttle, Right trigger = unused
            gamepad.left_joystick(x_value=roll, y_value=pitch)
            gamepad.right_joystick(x_value=gimbal_x, y_value=gimbal_y)
            gamepad.left_trigger(value=throttle)
            # Yaw lives on a slider so it doesn't fight the gimbal axes.
            # vgamepad doesn't expose sliders directly; map yaw → right_trigger
            # so it's still reachable through DirectInput as an axis.
            gamepad.right_trigger(value=scale_trigger(yaw_raw))

            # -------- BUTTONS --------
            def set_btn(cond, btn):
                if cond:
                    gamepad.press_button(btn)
                else:
                    gamepad.release_button(btn)

            set_btn(sw1 == 1, vg.XUSB_BUTTON.XUSB_GAMEPAD_A)
            set_btn(sw1 == 2, vg.XUSB_BUTTON.XUSB_GAMEPAD_B)
            set_btn(sw2 == 1, vg.XUSB_BUTTON.XUSB_GAMEPAD_X)
            set_btn(sw2 == 2, vg.XUSB_BUTTON.XUSB_GAMEPAD_Y)
            set_btn(sw3 == 1, vg.XUSB_BUTTON.XUSB_GAMEPAD_LEFT_SHOULDER)
            set_btn(sw3 == 2, vg.XUSB_BUTTON.XUSB_GAMEPAD_RIGHT_SHOULDER)
            set_btn(kill == 1, vg.XUSB_BUTTON.XUSB_GAMEPAD_BACK)

            # -------- DPAD (mode) --------
            gamepad.release_button(vg.XUSB_BUTTON.XUSB_GAMEPAD_DPAD_UP)
            gamepad.release_button(vg.XUSB_BUTTON.XUSB_GAMEPAD_DPAD_DOWN)
            gamepad.release_button(vg.XUSB_BUTTON.XUSB_GAMEPAD_DPAD_LEFT)
            gamepad.release_button(vg.XUSB_BUTTON.XUSB_GAMEPAD_DPAD_RIGHT)
            if mode == 1:
                gamepad.press_button(vg.XUSB_BUTTON.XUSB_GAMEPAD_DPAD_DOWN)
            elif mode == 2:
                gamepad.press_button(vg.XUSB_BUTTON.XUSB_GAMEPAD_DPAD_LEFT)
            elif mode == 3:
                gamepad.press_button(vg.XUSB_BUTTON.XUSB_GAMEPAD_DPAD_RIGHT)
            # mode == 0 is the idle/neutral position — leave DPAD released.

            gamepad.update()
            time.sleep(0.005)

        except serial.SerialException as e:
            print(f"NOMAD bridge: serial error ({e}); reopening…", flush=True)
            try: ser.close()
            except Exception: pass
            ser = None
            while ser is None:
                try:
                    ser = serial.Serial(port, baud, timeout=0.05)
                    time.sleep(1)
                    ser.reset_input_buffer()
                    print("NOMAD bridge: serial reopened.", flush=True)
                except serial.SerialException:
                    time.sleep(2)

        except Exception as e:
            print(f"NOMAD bridge: error: {e}", flush=True)
            time.sleep(0.05)


def main() -> int:
    ap = argparse.ArgumentParser(description="NOMAD serial → virtual Xbox 360 bridge.")
    ap.add_argument("--port", default="COM7", help="Serial port (e.g. COM10, /dev/ttyUSB0)")
    ap.add_argument("--baud", type=int, default=115200, help="Baud rate (default 115200)")
    ap.add_argument("--verbose", action="store_true", help="Print every serial frame")
    args = ap.parse_args()

    # Make Ctrl+C clean (no Python traceback dump).
    def _bye(*_):
        print("NOMAD bridge: shutting down.", flush=True)
        sys.exit(0)
    signal.signal(signal.SIGINT, _bye)
    try:
        signal.signal(signal.SIGTERM, _bye)
    except (AttributeError, ValueError):
        pass  # SIGTERM not supported on Windows for non-console sessions

    return run(args.port, args.baud, args.verbose)


if __name__ == "__main__":
    sys.exit(main())
