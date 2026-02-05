#!/usr/bin/env python3
"""
Servo Controller for NOMAD.

Controls water nozzle servo via PWM and water shooter pump via GPIO on Jetson GPIO pins.

Jetson Orin Nano 40-pin Header:
- Pin 15 (PWM1) = pwmchip0 - Water nozzle angle servo
- Pin 18 (GPIO) - Water shooter pump trigger (simple GPIO HIGH/LOW)

Pin 15 PWM configuration:
- Must be configured via jetson-io: sudo python3 /opt/nvidia/jetson-io/config-by-function.py pwm1
- PWM sysfs path: /sys/devices/3280000.pwm -> /sys/class/pwm/pwmchip0
- PWM channel: 0

Servo PWM specifications (typical):
- Frequency: 50 Hz (20ms period)
- Pulse width: 500us (0 deg) to 2500us (180 deg)
- Neutral: 1500us (90 deg)

Wiring (Pin 15 nozzle servo):
- Servo Signal (orange/white) -> Pin 15 (PWM1)
- Servo Power (red)           -> Pin 2 or Pin 4 (5V) or external 5V supply
- Servo Ground (brown/black)  -> Pin 14 (GND) or any GND pin
"""

import logging
import time
from dataclasses import dataclass
from enum import Enum
from typing import Optional

logger = logging.getLogger(__name__)

# PWM chip and channel mapping for Jetson Orin Nano
# PWM1 (Pin 15) = pwmchip0 at 0x3280000
PWM_CHIPS = {
    "pwm1": {"chip": 0, "channel": 0},  # Pin 15 - nozzle servo
}

# GPIO pin for water shooter
WATER_SHOOTER_GPIO_PIN = 18  # Physical pin 18 on 40-pin header

# Servo configuration
SERVO_FREQ_HZ = 50  # Standard servo frequency
SERVO_PERIOD_NS = int(1e9 / SERVO_FREQ_HZ)  # 20,000,000 ns = 20ms

# Pulse width range (microseconds -> nanoseconds)
SERVO_MIN_PULSE_NS = 500_000   # 0.5ms -> 0 degrees
SERVO_MAX_PULSE_NS = 2500_000  # 2.5ms -> 180 degrees
SERVO_NEUTRAL_PULSE_NS = 1500_000  # 1.5ms -> 90 degrees


class ServoFunction(Enum):
    """Servo functions in the system."""
    CAMERA_TILT = "camera_tilt"  # Pin 15 (PWM1) - Water nozzle angle
    WATER_SHOOTER = "water_shooter"  # Pin 18 (GPIO - not PWM)


@dataclass
class ServoConfig:
    """Configuration for a PWM servo."""
    name: str
    pwm_chip: int
    pwm_channel: int
    min_angle: float = 0.0
    max_angle: float = 180.0
    min_pulse_ns: int = SERVO_MIN_PULSE_NS
    max_pulse_ns: int = SERVO_MAX_PULSE_NS
    neutral_angle: float = 90.0
    inverted: bool = False  # If True, 0 deg = max pulse


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
    Controls a single PWM servo via sysfs interface.
    
    Uses /sys/class/pwm/pwmchipX/pwmY for hardware PWM control.
    """
    
    def __init__(self, config: ServoConfig):
        self.config = config
        self._enabled = False
        self._current_angle = config.neutral_angle
        self._pwm_path = f"/sys/class/pwm/pwmchip{config.pwm_chip}/pwm{config.pwm_channel}"
        self._chip_path = f"/sys/class/pwm/pwmchip{config.pwm_chip}"
        
    def initialize(self) -> bool:
        """
        Initialize the PWM channel.
        
        Returns True if successful, False otherwise.
        """
        try:
            # Export the PWM channel if not already exported
            if not self._path_exists(self._pwm_path):
                export_path = f"{self._chip_path}/export"
                if self._path_exists(export_path):
                    self._write_sysfs(export_path, str(self.config.pwm_channel))
                    time.sleep(0.1)  # Wait for export
                else:
                    logger.error(f"PWM chip {self.config.pwm_chip} not found")
                    return False
            
            # Set period (frequency)
            self._write_sysfs(f"{self._pwm_path}/period", str(SERVO_PERIOD_NS))
            
            # Set initial duty cycle (neutral)
            neutral_pulse = self._angle_to_pulse_ns(self.config.neutral_angle)
            self._write_sysfs(f"{self._pwm_path}/duty_cycle", str(neutral_pulse))
            
            logger.info(f"Servo {self.config.name} initialized: chip={self.config.pwm_chip}, channel={self.config.pwm_channel}")
            return True
            
        except Exception as e:
            logger.error(f"Failed to initialize servo {self.config.name}: {e}")
            return False
    
    def enable(self) -> bool:
        """Enable PWM output."""
        try:
            self._write_sysfs(f"{self._pwm_path}/enable", "1")
            self._enabled = True
            logger.info(f"Servo {self.config.name} enabled")
            return True
        except Exception as e:
            logger.error(f"Failed to enable servo {self.config.name}: {e}")
            return False
    
    def disable(self) -> bool:
        """Disable PWM output."""
        try:
            self._write_sysfs(f"{self._pwm_path}/enable", "0")
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
        
        try:
            pulse_ns = self._angle_to_pulse_ns(angle)
            self._write_sysfs(f"{self._pwm_path}/duty_cycle", str(pulse_ns))
            self._current_angle = angle
            return True
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
    
    def _angle_to_pulse_ns(self, angle: float) -> int:
        """Convert angle (degrees) to pulse width (nanoseconds)."""
        # Normalize angle to 0-1 range
        normalized = (angle - self.config.min_angle) / (self.config.max_angle - self.config.min_angle)
        
        if self.config.inverted:
            normalized = 1.0 - normalized
        
        # Map to pulse width
        pulse_range = self.config.max_pulse_ns - self.config.min_pulse_ns
        pulse_ns = int(self.config.min_pulse_ns + (normalized * pulse_range))
        
        return pulse_ns
    
    def _write_sysfs(self, path: str, value: str) -> None:
        """Write value to sysfs file."""
        with open(path, 'w') as f:
            f.write(value)
    
    def _path_exists(self, path: str) -> bool:
        """Check if sysfs path exists."""
        import os
        return os.path.exists(path)


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
        
        # PWM servo configuration (nozzle on Pin 15)
        self._servo_configs = {
            ServoFunction.CAMERA_TILT: ServoConfig(
                name="nozzle",
                pwm_chip=0,  # PWM1 on Pin 15
                pwm_channel=0,
                min_angle=0.0,
                max_angle=180.0,
                neutral_angle=90.0,  # Nozzle starts level
            ),
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
        
        # Initialize PWM servos
        for function, config in self._servo_configs.items():
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
    
    def get_servo(self, function: ServoFunction) -> Optional[PWMServo]:
        """Get a specific PWM servo by function."""
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
            status["servos"][function.value] = {
                "angle": state.angle,
                "enabled": state.enabled,
                "last_update": state.last_update,
                "type": "pwm"
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


def init_servo_controller() -> bool:
    """Initialize the global servo controller."""
    global _controller
    if _controller is None:
        _controller = ServoController()
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
