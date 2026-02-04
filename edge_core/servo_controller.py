#!/usr/bin/env python3
"""
Servo Controller for NOMAD.

Controls camera tilt servo and water shooter via PWM on Jetson GPIO pins.

Jetson Orin Nano 40-pin Header PWM pins:
- Pin 13 (PWM8) - Camera tilt servo
- Pin 15 (PWM1) - Reserved/future use
- Pin 18 (PWM5) - Water shooter trigger servo

Before using, configure pins via jetson-io:
    sudo /opt/nvidia/jetson-io/jetson-io.py
    Configure Jetson 40pin Header -> Configure header pins manually
    -> [*] pwm1 (15), [*] pwm5 (18), [*] pwm8 (13)
    -> Back -> Save pin changes -> Save and reboot

Servo PWM specifications (typical):
- Frequency: 50 Hz (20ms period)
- Pulse width: 500us (0 deg) to 2500us (180 deg)
- Neutral: 1500us (90 deg)
"""

import logging
import time
from dataclasses import dataclass
from enum import Enum
from typing import Optional

logger = logging.getLogger(__name__)

# PWM chip and channel mapping for Jetson Orin Nano
# These may need adjustment based on actual jetson-io configuration
PWM_CHIPS = {
    "pwm1": {"chip": 0, "channel": 0},  # Pin 15
    "pwm5": {"chip": 1, "channel": 0},  # Pin 18  
    "pwm8": {"chip": 2, "channel": 0},  # Pin 13
}

# Servo configuration
SERVO_FREQ_HZ = 50  # Standard servo frequency
SERVO_PERIOD_NS = int(1e9 / SERVO_FREQ_HZ)  # 20,000,000 ns = 20ms

# Pulse width range (microseconds -> nanoseconds)
SERVO_MIN_PULSE_NS = 500_000   # 0.5ms -> 0 degrees
SERVO_MAX_PULSE_NS = 2500_000  # 2.5ms -> 180 degrees
SERVO_NEUTRAL_PULSE_NS = 1500_000  # 1.5ms -> 90 degrees


class ServoFunction(Enum):
    """Servo functions in the system."""
    CAMERA_TILT = "camera_tilt"  # Pin 13 (PWM8)
    WATER_SHOOTER = "water_shooter"  # Pin 18 (PWM5)


@dataclass
class ServoConfig:
    """Configuration for a servo."""
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


class ServoController:
    """
    Main servo controller managing all servos in the system.
    """
    
    def __init__(self):
        self._servos: dict[ServoFunction, PWMServo] = {}
        self._initialized = False
        
        # Default servo configurations
        self._configs = {
            ServoFunction.CAMERA_TILT: ServoConfig(
                name="camera_tilt",
                pwm_chip=2,  # PWM8 on pin 13
                pwm_channel=0,
                min_angle=0.0,
                max_angle=180.0,
                neutral_angle=90.0,  # Camera starts level
            ),
            ServoFunction.WATER_SHOOTER: ServoConfig(
                name="water_shooter",
                pwm_chip=1,  # PWM5 on pin 18
                pwm_channel=0,
                min_angle=0.0,
                max_angle=180.0,
                neutral_angle=0.0,  # Shooter starts closed/off
            ),
        }
    
    def initialize(self) -> bool:
        """
        Initialize all servos.
        
        Returns True if at least one servo initialized successfully.
        """
        success_count = 0
        
        for function, config in self._configs.items():
            servo = PWMServo(config)
            if servo.initialize():
                self._servos[function] = servo
                success_count += 1
            else:
                logger.warning(f"Servo {function.value} failed to initialize")
        
        self._initialized = success_count > 0
        
        if self._initialized:
            logger.info(f"Servo controller initialized: {success_count}/{len(self._configs)} servos")
        else:
            logger.error("Servo controller failed to initialize - no servos available")
        
        return self._initialized
    
    def is_available(self) -> bool:
        """Check if servo controller is available."""
        return self._initialized and len(self._servos) > 0
    
    def get_servo(self, function: ServoFunction) -> Optional[PWMServo]:
        """Get a specific servo by function."""
        return self._servos.get(function)
    
    def set_camera_tilt(self, angle: float) -> bool:
        """
        Set camera tilt angle.
        
        Args:
            angle: Tilt angle in degrees (0=down, 90=level, 180=up)
            
        Returns:
            True if successful
        """
        servo = self.get_servo(ServoFunction.CAMERA_TILT)
        if servo:
            return servo.set_angle(angle)
        logger.warning("Camera tilt servo not available")
        return False
    
    def get_camera_tilt(self) -> Optional[float]:
        """Get current camera tilt angle."""
        servo = self.get_servo(ServoFunction.CAMERA_TILT)
        if servo:
            return servo.get_state().angle
        return None
    
    def trigger_water_shooter(self, duration_ms: int = 200) -> bool:
        """
        Trigger water shooter for specified duration.
        
        Args:
            duration_ms: How long to activate shooter (milliseconds)
            
        Returns:
            True if successful
        """
        servo = self.get_servo(ServoFunction.WATER_SHOOTER)
        if not servo:
            logger.warning("Water shooter servo not available")
            return False
        
        try:
            # Activate shooter (move to max position)
            servo.set_angle(180.0)
            servo.enable()
            
            # Wait for duration
            time.sleep(duration_ms / 1000.0)
            
            # Return to neutral (closed)
            servo.set_angle(0.0)
            
            logger.info(f"Water shooter triggered for {duration_ms}ms")
            return True
            
        except Exception as e:
            logger.error(f"Water shooter trigger failed: {e}")
            return False
    
    def enable_all(self) -> None:
        """Enable all servos."""
        for servo in self._servos.values():
            servo.enable()
    
    def disable_all(self) -> None:
        """Disable all servos (for safety)."""
        for servo in self._servos.values():
            servo.disable()
    
    def get_status(self) -> dict:
        """Get status of all servos."""
        status = {
            "initialized": self._initialized,
            "servo_count": len(self._servos),
            "servos": {}
        }
        
        for function, servo in self._servos.items():
            state = servo.get_state()
            status["servos"][function.value] = {
                "angle": state.angle,
                "enabled": state.enabled,
                "last_update": state.last_update,
            }
        
        return status
    
    def shutdown(self) -> None:
        """Safely shutdown all servos."""
        logger.info("Shutting down servo controller")
        self.disable_all()
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
