import serial
import vgamepad as vg
import time

# =========================
# VIRTUAL CONTROLLER
# =========================

gamepad = vg.VX360Gamepad()

# =========================
# SERIAL CONFIG
# =========================

PORT = "COM10"   # CHANGE THIS
BAUD = 115200

# =========================
# SCALE FUNCTIONS
# =========================

def scale_stick(v):
    try:
        v = max(0, min(1023, int(v)))
        return int((v / 1023) * 65535 - 32768)
    except:
        return 0


def scale_trigger(v):
    try:
        v = max(0, min(1023, int(v)))
        return int((v / 1023) * 255)
    except:
        return 0

# =========================
# SERIAL INIT
# =========================

print("Starting Windows Flight Bridge...")

ser = serial.Serial(PORT, BAUD, timeout=0.05)
time.sleep(2)
ser.reset_input_buffer()

# =========================
# MAIN LOOP
# =========================

while True:
    try:
        line = ser.readline().decode(errors="ignore").strip()
        print(line)

        if not line:
            continue

        parts = line.split(",")

        # Accept 10 or 11 values safely
        if len(parts) < 10:
            continue

        parts = parts[:11]  # safe truncate if extra exists

        # =========================
        # AXES
        # =========================

        roll     = scale_stick(parts[0])
        pitch    = scale_stick(parts[1])
        yaw      = scale_stick(parts[2])
        throttle = scale_trigger(parts[3])

        gimbal_x = scale_stick(parts[4])
        gimbal_y = scale_stick(parts[5])

        # =========================
        # SWITCHES
        # =========================

        sw1  = int(parts[6])
        sw2  = int(parts[7])
        sw3  = int(parts[8])
        mode = int(parts[9])

        kill = int(parts[10]) if len(parts) > 10 else 0

        # =========================
        # AXES OUTPUT
        # =========================

        gamepad.left_joystick(x_value=roll, y_value=pitch)
        gamepad.right_joystick(x_value=yaw, y_value=gimbal_y)
        gamepad.left_trigger(value=throttle)

        # =========================
        # BUTTONS (FIXED STATE LOGIC)
        # =========================

        # SW1 → A / B
        if sw1 == 1:
            gamepad.press_button(vg.XUSB_BUTTON.XUSB_GAMEPAD_A)
        else:
            gamepad.release_button(vg.XUSB_BUTTON.XUSB_GAMEPAD_A)

        if sw1 == 2:
            gamepad.press_button(vg.XUSB_BUTTON.XUSB_GAMEPAD_B)
        else:
            gamepad.release_button(vg.XUSB_BUTTON.XUSB_GAMEPAD_B)

        # SW2 → X / Y
        if sw2 == 1:
            gamepad.press_button(vg.XUSB_BUTTON.XUSB_GAMEPAD_X)
        else:
            gamepad.release_button(vg.XUSB_BUTTON.XUSB_GAMEPAD_X)

        if sw2 == 2:
            gamepad.press_button(vg.XUSB_BUTTON.XUSB_GAMEPAD_Y)
        else:
            gamepad.release_button(vg.XUSB_BUTTON.XUSB_GAMEPAD_Y)

        # SW3 → shoulders
        if sw3 == 1:
            gamepad.press_button(vg.XUSB_BUTTON.XUSB_GAMEPAD_LEFT_SHOULDER)
        else:
            gamepad.release_button(vg.XUSB_BUTTON.XUSB_GAMEPAD_LEFT_SHOULDER)

        if sw3 == 2:
            gamepad.press_button(vg.XUSB_BUTTON.XUSB_GAMEPAD_RIGHT_SHOULDER)
        else:
            gamepad.release_button(vg.XUSB_BUTTON.XUSB_GAMEPAD_RIGHT_SHOULDER)

        # =========================
        # MODE (DPAD)
        # =========================

        # release all first
        gamepad.release_button(vg.XUSB_BUTTON.XUSB_GAMEPAD_DPAD_UP)
        gamepad.release_button(vg.XUSB_BUTTON.XUSB_GAMEPAD_DPAD_DOWN)
        gamepad.release_button(vg.XUSB_BUTTON.XUSB_GAMEPAD_DPAD_LEFT)
        gamepad.release_button(vg.XUSB_BUTTON.XUSB_GAMEPAD_DPAD_RIGHT)

        if mode == 0:
            gamepad.press_button(vg.XUSB_BUTTON.XUSB_GAMEPAD_DPAD_UP)
        elif mode == 1:
            gamepad.press_button(vg.XUSB_BUTTON.XUSB_GAMEPAD_DPAD_DOWN)
        elif mode == 2:
            gamepad.press_button(vg.XUSB_BUTTON.XUSB_GAMEPAD_DPAD_LEFT)
        elif mode == 3:
            gamepad.press_button(vg.XUSB_BUTTON.XUSB_GAMEPAD_DPAD_RIGHT)

        # =========================
        # KILL SWITCH (optional mapping)
        # =========================

        if kill == 1:
            gamepad.press_button(vg.XUSB_BUTTON.XUSB_GAMEPAD_BACK)
        else:
            gamepad.release_button(vg.XUSB_BUTTON.XUSB_GAMEPAD_BACK)

        # =========================
        # SEND UPDATE
        # =========================

        gamepad.update()

        time.sleep(0.005)

    except Exception as e:
        print("Error:", e)
        time.sleep(0.05)
