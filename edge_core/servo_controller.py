#!/usr/bin/env python3
"""
Servo Controller for NOMAD.

Controls water nozzle servo via GPIO bit-bang PWM and water shooter pump via GPIO
on Jetson Orin Nano GPIO pins.

Jetson Orin Nano 40-pin Header:
- Pin 15 - Water nozzle angle servo (GPIO bit-bang PWM via /dev/gpiochip0 line 85)
- Pin 18 (GPIO) - Water shooter pump trigger (simple GPIO HIGH/LOW)

NOTE: Hardware sysfs PWM (pwmchip0) does NOT route to Pin 15 on the Orin Nano
due to pinmux limitations. We use a compiled C helper for precise GPIO bit-bang
PWM instead. This has been tested and confirmed working.

Servo PWM specifications (typical):
- Frequency: 50 Hz (20ms period)
- Pulse width: 500us (0 deg) to 2500us (180 deg)
- Neutral: 1500us (90 deg)

Wiring (Pin 15 nozzle servo):
- Servo Signal (orange/white) -> Pin 15
- Servo Power (red)           -> Pin 2 or Pin 4 (5V) or external 5V supply
- Servo Ground (brown/black)  -> Pin 14 (GND) or any GND pin
"""

import logging
import os
import select
import subprocess
import threading
import time
from dataclasses import dataclass
from enum import Enum
from typing import Optional, Any

logger = logging.getLogger(__name__)

# GPIO pin for water shooter
WATER_SHOOTER_GPIO_PIN = 18  # Physical pin 18 on 40-pin header


class ServoFunction(Enum):
    """Servo functions in the system."""
    CAMERA_TILT = "camera_tilt"  # Pin 15 (PWM1) - Water nozzle angle
    WATER_SHOOTER = "water_shooter"  # Pin 18 (GPIO - not PWM)


@dataclass
class ServoConfig:
    """Configuration for a GPIO bit-bang PWM servo."""
    name: str
    gpio_chip: int  # /dev/gpiochipN
    gpio_line: int  # GPIO line number within the chip
    min_angle: float = 0.0
    max_angle: float = 180.0
    min_pulse_us: int = 500   # 0.5ms -> 0 degrees
    max_pulse_us: int = 2500  # 2.5ms -> 180 degrees
    neutral_angle: float = 90.0
    inverted: bool = False  # If True, 0 deg = max pulse


# Pin 15 on Jetson Orin Nano = gpiochip0, line 85 (SOC_GPIO39_PN1)
NOZZLE_SERVO_CONFIG = ServoConfig(
    name="nozzle",
    gpio_chip=0,
    gpio_line=85,
)


@dataclass 
class GPIOConfig:
    """Configuration for a GPIO output."""
    name: str
    pin: int  # Physical pin number (BOARD mode)
    active_high: bool = True  # True = HIGH to activate, False = LOW to activate


@dataclass
class ServoState:
    """Current state of a servo."""
    angle: float
    enabled: bool
    last_update: float


class PWMServo:
    """
    Controls a single servo via GPIO bit-bang PWM using a compiled C helper.
    
    Uses /dev/gpiochipN character device with ioctl for precise pulse timing.
    A background C process generates continuous 50Hz PWM pulses.
    
    This is used because hardware sysfs PWM (pwmchip0) does not route to
    Pin 15 on the Jetson Orin Nano despite correct pinmux overlay configuration.
    """
    
    # C source for the servo PWM helper - compiled once at init
    _C_SOURCE = r'''
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <signal.h>
#include <sys/ioctl.h>
#include <linux/gpio.h>
#include <time.h>

static volatile int running = 1;
void handle_sig(int s) { running = 0; }

void sleep_us(long us) {
    struct timespec ts;
    ts.tv_sec = us / 1000000;
    ts.tv_nsec = (us % 1000000) * 1000;
    nanosleep(&ts, NULL);
}

int main(int argc, char *argv[]) {
    if (argc < 4) {
        fprintf(stderr, "Usage: %s <chip> <line> <pulse_us>\n", argv[0]);
        return 1;
    }
    
    int chip_num = atoi(argv[1]);
    int line_num = atoi(argv[2]);
    int pulse_us = atoi(argv[3]);
    
    char chip_path[64];
    snprintf(chip_path, sizeof(chip_path), "/dev/gpiochip%d", chip_num);
    
    int fd = open(chip_path, O_RDONLY);
    if (fd < 0) { perror("open gpiochip"); return 1; }
    
    struct gpiohandle_request req;
    memset(&req, 0, sizeof(req));
    req.lineoffsets[0] = line_num;
    req.flags = GPIOHANDLE_REQUEST_OUTPUT;
    req.default_values[0] = 0;
    req.lines = 1;
    strcpy(req.consumer_label, "nomad_servo");
    
    if (ioctl(fd, GPIO_GET_LINEHANDLE_IOCTL, &req) < 0) {
        perror("ioctl get line");
        close(fd);
        return 1;
    }
    
    signal(SIGTERM, handle_sig);
    signal(SIGINT, handle_sig);
    
    /* Write PID to stdout so parent can manage us */
    printf("READY %d\n", getpid());
    fflush(stdout);
    
    /* Read new pulse widths from stdin (non-blocking) */
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
    
    struct gpiohandle_data data;
    int period_us = 20000; /* 50Hz */
    char buf[32];
    
    while (running) {
        /* Check for new pulse width command */
        int n = read(STDIN_FILENO, buf, sizeof(buf) - 1);
        if (n > 0) {
            buf[n] = '\0';
            int new_pulse = atoi(buf);
            if (new_pulse >= 500 && new_pulse <= 2500) {
                pulse_us = new_pulse;
            }
        }
        
        /* Generate one PWM cycle */
        data.values[0] = 1;
        ioctl(req.fd, GPIOHANDLE_SET_LINE_VALUES_IOCTL, &data);
        sleep_us(pulse_us);
        
        data.values[0] = 0;
        ioctl(req.fd, GPIOHANDLE_SET_LINE_VALUES_IOCTL, &data);
        sleep_us(period_us - pulse_us);
    }
    
    /* Cleanup - set low */
    data.values[0] = 0;
    ioctl(req.fd, GPIOHANDLE_SET_LINE_VALUES_IOCTL, &data);
    
    close(req.fd);
    close(fd);
    return 0;
}
'''
    
    _binary_path = os.path.expanduser("~/.nomad/bin/nomad_servo_pwm")
    _compiled = False
    _compile_lock = threading.Lock()
    
    def __init__(self, config: ServoConfig):
        self.config = config
        self._enabled = False
        self._current_angle = config.neutral_angle
        self._process: Optional[subprocess.Popen] = None
        self._lock = threading.Lock()
        
    @classmethod
    def _ensure_compiled(cls) -> bool:
        """Compile the C PWM helper if not already done."""
        with cls._compile_lock:
            if cls._compiled and os.path.exists(cls._binary_path):
                return True

            src_path = f"{cls._binary_path}.c"
            try:
                # Ensure the binary directory exists
                os.makedirs(os.path.dirname(cls._binary_path), exist_ok=True)

                with open(src_path, 'w') as f:
                    f.write(cls._C_SOURCE)

                result = subprocess.run(
                    ['gcc', '-O2', '-o', cls._binary_path, src_path],
                    capture_output=True, text=True, timeout=30
                )

                if result.returncode != 0:
                    logger.error(f"Failed to compile servo PWM helper: {result.stderr}")
                    return False

                cls._compiled = True
                logger.info(f"Servo PWM helper compiled: {cls._binary_path}")
                return True

            except Exception as e:
                logger.error(f"Failed to compile servo PWM helper: {e}")
                return False
    
    def initialize(self) -> bool:
        """
        Initialize the servo - compile helper and verify GPIO access.
        
        Returns True if successful, False otherwise.
        """
        try:
            if not self._ensure_compiled():
                return False
            
            # Verify GPIO chip exists
            chip_path = f"/dev/gpiochip{self.config.gpio_chip}"
            if not os.path.exists(chip_path):
                logger.error(f"GPIO chip not found: {chip_path}")
                return False
            
            logger.info(
                f"Servo {self.config.name} initialized: "
                f"gpiochip{self.config.gpio_chip}/line{self.config.gpio_line}"
            )
            return True
            
        except Exception as e:
            logger.error(f"Failed to initialize servo {self.config.name}: {e}")
            return False
    
    def enable(self) -> bool:
        """Enable servo - start the PWM helper process."""
        with self._lock:
            return self._enable_locked()
    
    def _enable_locked(self) -> bool:
        """Internal enable - caller must hold self._lock."""
        try:
            if self._process is not None:
                # Check if still alive
                if self._process.poll() is None:
                    self._enabled = True
                    return True
                # Process died, clean up
                self._process = None
            
            if not os.path.exists(self._binary_path):
                logger.error(f"Servo binary not found: {self._binary_path}")
                return False
            
            pulse_us = self._angle_to_pulse_us(self._current_angle)
            
            self._process = subprocess.Popen(
                [
                    self._binary_path,
                    str(self.config.gpio_chip),
                    str(self.config.gpio_line),
                    str(pulse_us),
                ],
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )
            
            # Wait for READY message with timeout to prevent hanging
            ready = select.select([self._process.stdout], [], [], 5.0)
            if not ready[0]:
                # Timeout - check if process crashed
                stderr_out = ""
                if self._process.poll() is not None:
                    stderr_out = self._process.stderr.read().decode(errors='replace')
                logger.error(
                    f"Servo helper timeout waiting for READY. "
                    f"Exit code: {self._process.poll()}, stderr: {stderr_out}"
                )
                self._process.terminate()
                try:
                    self._process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    self._process.kill()
                    try:
                        self._process.wait(timeout=1)
                    except Exception:
                        pass
                self._process = None
                return False
            
            ready_line = self._process.stdout.readline().decode().strip()
            if not ready_line.startswith("READY"):
                stderr_out = ""
                if self._process.poll() is not None:
                    stderr_out = self._process.stderr.read().decode(errors='replace')
                logger.error(
                    f"Servo helper did not start: '{ready_line}', stderr: {stderr_out}"
                )
                self._process.terminate()
                try:
                    self._process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    self._process.kill()
                    try:
                        self._process.wait(timeout=1)
                    except Exception:
                        pass
                self._process = None
                return False
            
            self._enabled = True
            logger.info(f"Servo {self.config.name} enabled ({ready_line})")
            return True
            
        except Exception as e:
            logger.error(f"Failed to enable servo {self.config.name}: {e}")
            if self._process is not None:
                try:
                    self._process.terminate()
                    try:
                        self._process.wait(timeout=2)
                    except subprocess.TimeoutExpired:
                        self._process.kill()
                        try:
                            self._process.wait(timeout=1)
                        except Exception:
                            pass
                except Exception:
                    pass
                self._process = None
            return False
    
    def disable(self) -> bool:
        """Disable servo - stop the PWM helper process."""
        with self._lock:
            try:
                if self._process is not None:
                    self._process.terminate()
                    try:
                        self._process.wait(timeout=2)
                    except subprocess.TimeoutExpired:
                        self._process.kill()
                    self._process = None
                
                self._enabled = False
                logger.info(f"Servo {self.config.name} disabled")
                return True
            except Exception as e:
                logger.error(f"Failed to disable servo {self.config.name}: {e}")
                return False
    
    def set_angle(self, angle: float) -> bool:
        """
        Set servo to specified angle.
        
        Args:
            angle: Target angle in degrees (0-180 typical)
            
        Returns:
            True if successful
        """
        # Clamp angle to valid range
        angle = max(self.config.min_angle, min(self.config.max_angle, angle))
        pulse_us = self._angle_to_pulse_us(angle)
        
        with self._lock:
            self._current_angle = angle
            
            if self._process is None or self._process.poll() is not None:
                # Auto-enable on first angle set (use _enable_locked to avoid deadlock)
                self._process = None
                if not self._enable_locked():
                    return False
            
            try:
                # Send new pulse width to the running helper
                self._process.stdin.write(f"{pulse_us}\n".encode())
                self._process.stdin.flush()
                return True
            except (BrokenPipeError, OSError) as e:
                logger.warning(f"Servo helper process died, restarting: {e}")
                self._process = None
                self._enabled = False
                # Try to restart (already holding lock)
                if self._enable_locked():
                    try:
                        self._process.stdin.write(f"{pulse_us}\n".encode())
                        self._process.stdin.flush()
                        return True
                    except:
                        pass
                return False
            except Exception as e:
                logger.error(f"Failed to set servo {self.config.name} angle: {e}")
                return False
    
    def get_state(self) -> ServoState:
        """Get current servo state."""
        return ServoState(
            angle=self._current_angle,
            enabled=self._enabled,
            last_update=time.time()
        )
    
    def _angle_to_pulse_us(self, angle: float) -> int:
        """Convert angle (degrees) to pulse width (microseconds)."""
        normalized = (angle - self.config.min_angle) / (self.config.max_angle - self.config.min_angle)
        
        if self.config.inverted:
            normalized = 1.0 - normalized
        
        pulse_range = self.config.max_pulse_us - self.config.min_pulse_us
        pulse_us = int(self.config.min_pulse_us + (normalized * pulse_range))
        
        return pulse_us


class MavlinkServo:
    """
    Controls a servo by sending MAVLink DO_SET_SERVO commands to the flight controller.

    This allows the nozzle / camera tilt servo to be driven from the Cube Orange
    PWM outputs instead of local Jetson GPIO PWM.
    """

    def __init__(self, config: ServoConfig, mavlink_service: Any | None = None, channel: Optional[int] = 8):
        self.config = config
        self._mav = mavlink_service
        self._channel = int(channel) if channel is not None else None
        self._enabled = False
        self._current_angle = config.neutral_angle
        self._lock = threading.Lock()

    def initialize(self) -> bool:
        """Initialize the MAVLink servo (no connection required at init).

        Returns True if configuration looks valid.
        """
        if self._channel is None or self._channel <= 0:
            logger.error(f"Invalid MAVLink servo channel: {self._channel}")
            return False

        logger.info(f"MAVLink servo {self.config.name} configured on channel {self._channel}")
        return True

    def enable(self) -> bool:
        self._enabled = True
        return True

    def disable(self) -> bool:
        self._enabled = False
        return True

    def set_angle(self, angle: float) -> bool:
        """Set servo angle by sending MAVLink DO_SET_SERVO (PWM microseconds)."""
        angle = max(self.config.min_angle, min(self.config.max_angle, angle))
        pulse_us = self._angle_to_pulse_us(angle)

        with self._lock:
            self._current_angle = angle
            if self._mav is None:
                logger.warning("MAVLink service not available for MAVLink servo")
                return False

            try:
                # MavlinkService.trigger_payload expects (pwm_value, servo_channel)
                success = self._mav.trigger_payload(pulse_us, self._channel)
                if not success:
                    logger.debug(f"MAVLink servo send failed (channel={self._channel}, pwm={pulse_us})")
                return success
            except Exception as e:
                logger.error(f"Failed to send MAVLink servo PWM: {e}")
                return False

    def get_state(self) -> ServoState:
        return ServoState(angle=self._current_angle, enabled=self._enabled, last_update=time.time())

    def _angle_to_pulse_us(self, angle: float) -> int:
        normalized = (angle - self.config.min_angle) / (self.config.max_angle - self.config.min_angle)
        if self.config.inverted:
            normalized = 1.0 - normalized
        pulse_range = self.config.max_pulse_us - self.config.min_pulse_us
        pulse_us = int(self.config.min_pulse_us + (normalized * pulse_range))
        return pulse_us


class GPIOOutput:
    """
    Controls a simple GPIO output pin for on/off control.
    
    Uses libgpiod (gpiod) library for GPIO control on Jetson Orin Nano.
    Jetson Orin Nano uses gpiochip interface instead of legacy sysfs.
    
    Physical pin to gpiochip/line mapping for Jetson Orin Nano:
    - Pin 18 (J40 header) = gpiochip0 line 50 (PQ.05)
    - Pin 12 = gpiochip0 line 79 (PR.04)
    """
    
    # Mapping of physical pins to (gpiochip, line) for Jetson Orin Nano
    PIN_MAP = {
        15: (0, 85),   # Physical pin 15 -> gpiochip0, line 85 (GPIO12)
        18: (0, 50),   # Physical pin 18 -> gpiochip0, line 50
        12: (0, 79),   # Physical pin 12 -> gpiochip0, line 79
        32: (0, 168),  # Physical pin 32 -> gpiochip0, line 168
        33: (0, 169),  # Physical pin 33 -> gpiochip0, line 169
    }
    
    def __init__(self, config: GPIOConfig):
        self.config = config
        self._enabled = False
        self._active = False
        self._gpio_available = False
        self._chip = None
        self._line = None
        
    def initialize(self) -> bool:
        """
        Initialize the GPIO pin using libgpiod.
        
        Returns True if successful, False otherwise.
        """
        try:
            import gpiod
            
            # Get chip and line number from pin map
            pin_info = self.PIN_MAP.get(self.config.pin)
            if pin_info is None:
                logger.error(f"Unknown physical pin {self.config.pin} - not in PIN_MAP")
                return False
            
            chip_num, line_num = pin_info
            
            # Open gpiochip
            self._chip = gpiod.Chip(f"/dev/gpiochip{chip_num}")
            
            # Get the line
            self._line = self._chip.get_line(line_num)
            
            # Request the line as output
            config_flags = gpiod.LINE_REQ_DIR_OUT
            initial_value = 0 if self.config.active_high else 1  # Start inactive
            
            self._line.request(
                consumer=f"nomad_{self.config.name}",
                type=config_flags,
                default_val=initial_value
            )
            
            self._gpio_available = True
            logger.info(f"GPIO {self.config.name} initialized: chip{chip_num}/line{line_num} (gpiod)")
            return True
            
        except ImportError:
            logger.warning(f"gpiod not available, trying subprocess fallback")
            return self._init_gpioset_fallback()
            
        except Exception as e:
            logger.error(f"Failed to initialize GPIO {self.config.name}: {e}")
            return self._init_gpioset_fallback()
    
    def _init_gpioset_fallback(self) -> bool:
        """
        Initialize GPIO using gpioset command (fallback when gpiod module not available).
        """
        try:
            import subprocess
            
            pin_info = self.PIN_MAP.get(self.config.pin)
            if pin_info is None:
                logger.error(f"Unknown physical pin {self.config.pin}")
                return False
            
            self._chip_num, self._line_num = pin_info
            
            # Test that gpioset is available
            result = subprocess.run(
                ["which", "gpioset"],
                capture_output=True,
                text=True
            )
            
            if result.returncode != 0:
                logger.error("gpioset command not found")
                return False
            
            self._use_subprocess = True
            self._gpio_available = True
            
            # Set initial state (inactive)
            initial_value = 0 if self.config.active_high else 1
            self._run_gpioset(initial_value)
            
            logger.info(f"GPIO {self.config.name} initialized: chip{self._chip_num}/line{self._line_num} (gpioset)")
            return True
            
        except Exception as e:
            logger.error(f"Failed to initialize GPIO via gpioset: {e}")
            return False
    
    def _run_gpioset(self, value: int) -> bool:
        """Run gpioset command to set GPIO value."""
        import subprocess
        try:
            # gpioset sets the line and holds it - we need to use -m time to not block
            # Actually, for a pump, we want to set and hold, so we'll use a different approach
            cmd = ["gpioset", f"gpiochip{self._chip_num}", f"{self._line_num}={value}"]
            
            # Run in background mode
            subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
            return True
        except Exception as e:
            logger.error(f"gpioset failed: {e}")
            return False
    
    def set_output(self, active: bool) -> bool:
        """
        Set GPIO output state.
        
        Args:
            active: True to activate (turn on), False to deactivate (turn off)
            
        Returns:
            True if successful
        """
        if not self._gpio_available:
            return False
        
        try:
            # Determine value based on active_high setting
            value = 1 if (active == self.config.active_high) else 0
            
            if hasattr(self, '_use_subprocess') and self._use_subprocess:
                # Kill any existing gpioset processes for this line
                import subprocess
                subprocess.run(
                    ["pkill", "-f", f"gpioset.*{self._line_num}="],
                    capture_output=True
                )
                time.sleep(0.05)
                self._run_gpioset(value)
            else:
                # Use gpiod library
                self._line.set_value(value)
            
            self._active = active
            logger.debug(f"GPIO {self.config.name} set to {'ON' if active else 'OFF'}")
            return True
            
        except Exception as e:
            logger.error(f"Failed to set GPIO {self.config.name}: {e}")
            return False
    
    def enable(self) -> bool:
        """Enable GPIO (mark as enabled, does not change output)."""
        self._enabled = True
        return True
    
    def disable(self) -> bool:
        """Disable GPIO (turn off output and mark as disabled)."""
        self.set_output(False)
        self._enabled = False
        return True
    
    def get_state(self) -> ServoState:
        """Get current GPIO state (reusing ServoState for compatibility)."""
        return ServoState(
            angle=100.0 if self._active else 0.0,  # 0 = off, 100 = on
            enabled=self._enabled,
            last_update=time.time()
        )
    
    def cleanup(self) -> None:
        """Cleanup GPIO resources."""
        try:
            self.set_output(False)
            if self._line is not None:
                self._line.release()
            if self._chip is not None:
                self._chip.close()
        except Exception as e:
            logger.warning(f"GPIO cleanup warning: {e}")


class ServoController:
    """
    Main servo controller managing all servos in the system.
    
    Nozzle angle uses PWM servo (Pin 15), water shooter pump uses GPIO (Pin 18).
    """
    
    def __init__(self):
        self._servos: dict[ServoFunction, PWMServo] = {}
        self._gpio_outputs: dict[ServoFunction, GPIOOutput] = {}
        self._initialized = False
        # Optional MAVLink integration for camera tilt servo
        self._mavlink_service: Optional[Any] = None
        self._camera_tilt_channel: Optional[int] = None
        
        # PWM servo configuration (nozzle on Pin 15 via GPIO bit-bang)
        self._servo_configs = {
            ServoFunction.CAMERA_TILT: NOZZLE_SERVO_CONFIG,
        }
        
        # GPIO configuration (water shooter)
        self._gpio_configs = {
            ServoFunction.WATER_SHOOTER: GPIOConfig(
                name="water_shooter",
                pin=WATER_SHOOTER_GPIO_PIN,  # Pin 18
                active_high=True,  # HIGH = pump on
            ),
        }
    
    def initialize(self) -> bool:
        """
        Initialize all servos and GPIO outputs.
        
        Returns True if at least one device initialized successfully.
        """
        success_count = 0
        
        # Initialize servos (PWM on Jetson or MAVLink-driven depending on config)
        for function, config in self._servo_configs.items():
            # If camera tilt is configured to use MAVLink channel, create MavlinkServo
            channel = getattr(self, '_camera_tilt_channel', None)
            if function == ServoFunction.CAMERA_TILT and channel is not None and channel > 0:
                servo = MavlinkServo(config, mavlink_service=self._mavlink_service, channel=channel)
            else:
                servo = PWMServo(config)

            if servo.initialize():
                self._servos[function] = servo
                success_count += 1
            else:
                logger.warning(f"Servo {function.value} failed to initialize")
        
        # Initialize GPIO outputs
        for function, config in self._gpio_configs.items():
            gpio = GPIOOutput(config)
            if gpio.initialize():
                self._gpio_outputs[function] = gpio
                success_count += 1
            else:
                logger.warning(f"GPIO {function.value} failed to initialize")
        
        self._initialized = success_count > 0
        
        if self._initialized:
            total = len(self._servo_configs) + len(self._gpio_configs)
            logger.info(f"Servo controller initialized: {success_count}/{total} devices")
        else:
            logger.error("Servo controller failed to initialize - no devices available")
        
        return self._initialized
    
    def is_available(self) -> bool:
        """Check if servo controller is available."""
        return self._initialized and (len(self._servos) > 0 or len(self._gpio_outputs) > 0)
    
    def get_servo(self, function: ServoFunction) -> Optional[Any]:
        """Get a specific servo (PWM or MAVLink) by function."""
        return self._servos.get(function)
    
    def get_gpio(self, function: ServoFunction) -> Optional[GPIOOutput]:
        """Get a specific GPIO output by function."""
        return self._gpio_outputs.get(function)
    
    def set_camera_tilt(self, angle: float) -> bool:
        """
        Set nozzle angle (API name kept as camera_tilt for backward compatibility).
        
        Args:
            angle: Nozzle angle in degrees (0=down, 90=level, 180=up)
            
        Returns:
            True if successful
        """
        servo = self.get_servo(ServoFunction.CAMERA_TILT)
        if servo:
            return servo.set_angle(angle)
        logger.warning("Camera tilt servo not available")
        return False
    
    def get_camera_tilt(self) -> Optional[float]:
        """Get current nozzle angle."""
        servo = self.get_servo(ServoFunction.CAMERA_TILT)
        if servo:
            return servo.get_state().angle
        return None
    
    def trigger_water_shooter(self, duration_ms: int = 200) -> bool:
        """
        Trigger water shooter for specified duration.
        
        Uses GPIO HIGH/LOW to turn pump on/off.
        
        Args:
            duration_ms: How long to activate shooter (milliseconds)
            
        Returns:
            True if successful
        """
        gpio = self.get_gpio(ServoFunction.WATER_SHOOTER)
        if not gpio:
            logger.warning("Water shooter GPIO not available")
            return False
        
        try:
            # Turn on pump
            gpio.set_output(True)
            gpio.enable()
            
            # Wait for duration
            time.sleep(duration_ms / 1000.0)
            
            # Turn off pump
            gpio.set_output(False)
            
            logger.info(f"Water shooter triggered for {duration_ms}ms")
            return True
            
        except Exception as e:
            logger.error(f"Water shooter trigger failed: {e}")
            # Ensure pump is off on error
            try:
                gpio.set_output(False)
            except:
                pass
            return False
    
    def enable_all(self) -> None:
        """Enable all servos and GPIO outputs."""
        for servo in self._servos.values():
            servo.enable()
        for gpio in self._gpio_outputs.values():
            gpio.enable()
    
    def disable_all(self) -> None:
        """Disable all servos and GPIO outputs (for safety)."""
        for servo in self._servos.values():
            servo.disable()
        for gpio in self._gpio_outputs.values():
            gpio.disable()
    
    def get_status(self) -> dict:
        """Get status of all servos and GPIO outputs."""
        status = {
            "initialized": self._initialized,
            "servo_count": len(self._servos),
            "gpio_count": len(self._gpio_outputs),
            "servos": {},
            "gpio_outputs": {}
        }
        
        for function, servo in self._servos.items():
            state = servo.get_state()
            # Determine servo type
            servo_type = "gpio_pwm"
            if isinstance(servo, MavlinkServo):
                servo_type = "mavlink"

            status["servos"][function.value] = {
                "angle": state.angle,
                "enabled": state.enabled,
                "last_update": state.last_update,
                "type": servo_type,
            }
        
        for function, gpio in self._gpio_outputs.items():
            state = gpio.get_state()
            status["gpio_outputs"][function.value] = {
                "active": state.angle > 50,  # angle > 50 means active (on)
                "enabled": state.enabled,
                "last_update": state.last_update,
                "type": "gpio"
            }
        
        return status
    
    def shutdown(self) -> None:
        """Safely shutdown all servos and GPIO outputs."""
        logger.info("Shutting down servo controller")
        self.disable_all()
        
        # Cleanup GPIO resources
        for gpio in self._gpio_outputs.values():
            gpio.cleanup()
        
        self._initialized = False


# Singleton instance
_controller: Optional[ServoController] = None


def init_servo_controller(mavlink_service: Any | None = None, camera_tilt_channel: Optional[int] = None) -> bool:
    """Initialize the global servo controller.

    Args:
        mavlink_service: Optional MavlinkService instance to drive FC PWM outputs.
        camera_tilt_channel: If set (>0) use this FC servo output channel for camera tilt (default None).
    """
    global _controller
    if _controller is None:
        _controller = ServoController()

    # Configure optional MAVLink-driven camera tilt
    _controller._mavlink_service = mavlink_service
    _controller._camera_tilt_channel = int(camera_tilt_channel) if camera_tilt_channel is not None else None

    return _controller.initialize()


def get_servo_controller() -> Optional[ServoController]:
    """Get the global servo controller instance."""
    return _controller


def shutdown_servo_controller() -> None:
    """Shutdown the global servo controller."""
    global _controller
    if _controller:
        _controller.shutdown()
        _controller = None
