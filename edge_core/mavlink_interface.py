from __future__ import annotations

import threading
import time
from typing import Any

from pymavlink import mavutil

from .state import StateManager


class MavlinkService:
    def __init__(self, state_manager: StateManager, endpoint: str = "127.0.0.1:14550") -> None:
        self.state_manager = state_manager
        self.endpoint = endpoint if endpoint.startswith("udp:") else f"udp:{endpoint}"
        self._conn: Any = None  # mavutil.mavlink_connection return type varies
        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._last_heartbeat = 0.0
        self.disconnect_timeout = 3.0
        
        # Time sync service reference (set externally)
        self._time_sync_service: Any = None
        
        # RC servo bridge reference (set externally)
        self._rc_servo_bridge: Any = None

    def set_time_sync_service(self, service: Any) -> None:
        """Set the TimeSyncService to receive GPS time updates."""
        self._time_sync_service = service

    def set_rc_servo_bridge(self, bridge: Any) -> None:
        """Set the RCServoBridge to receive RC channel updates."""
        self._rc_servo_bridge = bridge

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        self.stop_health_broadcast()  # Stop health broadcast if running
        if self._thread:
            self._thread.join(timeout=2.0)
        if self._conn:
            try:
                self._conn.close()
            except Exception:
                pass

    def arm_disarm(self, should_arm: bool) -> None:
        if not self._conn:
            return
        try:
            self._conn.mav.command_long_send(
                self._conn.target_system,
                self._conn.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                1 if should_arm else 0,
                0,
                0,
                0,
                0,
                0,
                0,
            )
        except Exception:
            pass

    def _connect(self) -> None:
        try:
            self._conn = mavutil.mavlink_connection(self.endpoint, autoreconnect=True)
        except Exception:
            self._conn = None

    def _run(self) -> None:
        self._connect()
        while not self._stop_event.is_set():
            if self._conn is None:
                time.sleep(0.5)
                self._connect()
                continue

            try:
                # Include RC_CHANNELS when bridge is active
                msg_types = ["HEARTBEAT", "SYS_STATUS", "GLOBAL_POSITION_INT", "ATTITUDE", "SYSTEM_TIME"]
                if self._rc_servo_bridge is not None:
                    msg_types.append("RC_CHANNELS")
                msg = self._conn.recv_match(
                    type=msg_types,
                    blocking=True,
                    timeout=0.2,
                )
            except Exception:
                msg = None

            now = time.time()
            if msg is None:
                self._update_connection_status(now)
                continue

            msg_type = msg.get_type()
            if msg_type == "HEARTBEAT":
                self._last_heartbeat = now
                mode = self._resolve_mode(msg)
                # Check armed status from base_mode (bit 7 = MAV_MODE_FLAG_SAFETY_ARMED)
                base_mode = getattr(msg, "base_mode", 0)
                is_armed = bool(base_mode & 128)  # MAV_MODE_FLAG_SAFETY_ARMED = 128
                # Critical state change - use force_state_update for immediate visibility
                self.state_manager.force_state_update(flight_mode=mode, connected=True, armed=is_armed)
            elif msg_type == "SYS_STATUS":
                voltage = getattr(msg, "voltage_battery", 0) or 0
                if voltage:
                    self.state_manager.update_state(battery_voltage=voltage / 1000.0)
            elif msg_type == "GLOBAL_POSITION_INT":
                lat_raw = getattr(msg, "lat", 0)
                lon_raw = getattr(msg, "lon", 0)
                alt_raw = getattr(msg, "alt", 0)  # mm above MSL
                gps_fix = bool(lat_raw or lon_raw)
                # GLOBAL_POSITION_INT encodes lat/lon in 1e7 degrees, alt in mm
                gps_lat = lat_raw / 1e7 if lat_raw else None
                gps_lon = lon_raw / 1e7 if lon_raw else None
                gps_alt = alt_raw / 1000.0 if alt_raw else None  # mm -> m
                self.state_manager.update_state(
                    gps_fix=gps_fix,
                    gps_lat=gps_lat,
                    gps_lon=gps_lon,
                    gps_alt=gps_alt,
                )
            elif msg_type == "ATTITUDE":
                import math
                # ATTITUDE message provides roll/pitch/yaw in radians
                roll_rad = getattr(msg, "roll", 0.0)
                pitch_rad = getattr(msg, "pitch", 0.0)
                yaw_rad = getattr(msg, "yaw", 0.0)
                heading_deg = math.degrees(yaw_rad) % 360.0  # 0-360
                pitch_deg = math.degrees(pitch_rad)
                roll_deg = math.degrees(roll_rad)
                self.state_manager.update_state(
                    heading_deg=heading_deg,
                    pitch_deg=pitch_deg,
                    roll_deg=roll_deg,
                )
            elif msg_type == "SYSTEM_TIME":
                # Update time sync service with GPS time
                if self._time_sync_service is not None:
                    time_unix_usec = getattr(msg, "time_unix_usec", 0)
                    time_boot_ms = getattr(msg, "time_boot_ms", 0)
                    if time_unix_usec > 0:
                        self._time_sync_service.update_gps_time(time_unix_usec, time_boot_ms)
            elif msg_type == "RC_CHANNELS":
                # Forward RC channel data to servo bridge
                if self._rc_servo_bridge is not None:
                    try:
                        self._rc_servo_bridge.on_rc_channels(msg)
                    except Exception:
                        pass

    def _update_connection_status(self, now: float) -> None:
        if self._last_heartbeat and (now - self._last_heartbeat) > self.disconnect_timeout:
            if self.state_manager.get_state().connected:
                # Critical state change (disconnection) - use force_state_update
                self.state_manager.force_state_update(connected=False, flight_mode="LOST")

    @staticmethod
    def _resolve_mode(msg) -> str:
        try:
            mode = mavutil.mode_string_v10(msg)
            return mode or "UNKNOWN"
        except Exception:
            return "UNKNOWN"

    def send_vision_position_estimate(
        self,
        timestamp_us: int,
        x: float,
        y: float,
        z: float,
        roll: float,
        pitch: float,
        yaw: float,
        covariance: list[float] | None = None,
        reset_counter: int = 0,
    ) -> bool:
        """
        Send VISION_POSITION_ESTIMATE message to the flight controller.
        
        This is used to inject VIO (Visual Inertial Odometry) data from the
        ZED camera into ArduPilot for indoor autonomous flight.
        
        Args:
            timestamp_us: Timestamp in microseconds (UNIX epoch)
            x: North position in meters (NED frame)
            y: East position in meters (NED frame)
            z: Down position in meters (NED frame, positive down)
            roll: Roll angle in radians
            pitch: Pitch angle in radians
            yaw: Yaw angle in radians
            covariance: Optional 21-element upper-triangle covariance matrix
            reset_counter: Estimate reset counter (increment on tracking reset)
        
        Returns:
            True if message sent successfully
        
        Note:
            ArduPilot parameter EK3_SRC1_POSXY=6 (ExternalNav) must be set
            to use this data for position estimation.
        """
        if self._conn is None:
            return False
        
        try:
            # Default covariance: diagonal matrix with reasonable variances
            if covariance is None:
                # 21 elements for upper triangle of 6x6 covariance matrix
                # [xx, xy, xz, xroll, xpitch, xyaw,
                #      yy, yz, yroll, ypitch, yyaw,
                #          zz, zroll, zpitch, zyaw,
                #              rollroll, rollpitch, rollyaw,
                #                        pitchpitch, pitchyaw,
                #                                    yawyaw]
                covariance = [
                    0.01, 0, 0, 0, 0, 0,     # x variance 0.01m^2
                          0.01, 0, 0, 0, 0,  # y variance 0.01m^2
                               0.01, 0, 0, 0,  # z variance 0.01m^2
                                    0.01, 0, 0,  # roll variance
                                          0.01, 0,  # pitch variance
                                                0.01   # yaw variance
                ]
            
            # Send VISION_POSITION_ESTIMATE message
            self._conn.mav.vision_position_estimate_send(
                timestamp_us,  # usec: Timestamp (microseconds since UNIX epoch)
                x,            # x: North (NED frame)
                y,            # y: East (NED frame)
                z,            # z: Down (NED frame)
                roll,         # roll: Roll angle
                pitch,        # pitch: Pitch angle
                yaw,          # yaw: Yaw angle
                covariance,   # covariance: Upper triangle of covariance matrix
                reset_counter  # reset_counter: Estimate reset counter
            )
            return True
            
        except Exception as e:
            # Log at debug level to avoid spam
            import logging
            logging.getLogger(__name__).debug(f"VIO send error: {e}")
            return False

    def send_vision_speed_estimate(
        self,
        timestamp_us: int,
        vx: float,
        vy: float,
        vz: float,
        covariance: list[float] | None = None,
        reset_counter: int = 0,
    ) -> bool:
        """
        Send VISION_SPEED_ESTIMATE message to the flight controller.
        
        Args:
            timestamp_us: Timestamp in microseconds
            vx: North velocity in m/s
            vy: East velocity in m/s
            vz: Down velocity in m/s
            covariance: Optional 9-element covariance matrix
            reset_counter: Estimate reset counter
        
        Returns:
            True if message sent successfully
        """
        if self._conn is None:
            return False
        
        try:
            if covariance is None:
                covariance = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]
            
            self._conn.mav.vision_speed_estimate_send(
                timestamp_us,
                vx,
                vy,
                vz,
                covariance,
                reset_counter,
            )
            return True
            
        except Exception as e:
            import logging
            logging.getLogger(__name__).debug(f"Vision speed send error: {e}")
            return False

    def send_velocity_command(
        self,
        vx: float,
        vy: float,
        vz: float,
        yaw_rate: float = 0.0,
        coordinate_frame: int | None = None,
    ) -> bool:
        """
        Send velocity command via SET_POSITION_TARGET_LOCAL_NED.
        
        Used for visual servoing to control drone velocity based on
        target tracking errors.
        
        Args:
            vx: X velocity in m/s (body-forward or NED-North)
            vy: Y velocity in m/s (body-right or NED-East)
            vz: Z velocity in m/s (body-down or NED-Down, positive = descend)
            yaw_rate: Yaw rate in rad/s (positive = clockwise from above)
            coordinate_frame: MAV_FRAME (default: BODY_OFFSET_NED for FRD)
        
        Returns:
            True if message sent successfully
        
        Note:
            type_mask specifies which fields to IGNORE:
            - Bit 0: x position
            - Bit 1: y position
            - Bit 2: z position
            - Bit 3: vx velocity (we want this, so bit = 0)
            - Bit 4: vy velocity (we want this, so bit = 0)
            - Bit 5: vz velocity (we want this, so bit = 0)
            - Bit 6: ax acceleration
            - Bit 7: ay acceleration
            - Bit 8: az acceleration
            - Bit 9: is force
            - Bit 10: yaw
            - Bit 11: yaw_rate (we want this, so bit = 0)
            
            For velocity + yaw_rate only: 0b0000_0111_11000_111 = 0x0FC7
            Position bits (0-2) = 1 (ignore), velocity bits (3-5) = 0 (use),
            accel bits (6-8) = 1 (ignore), force bit (9) = 1 (ignore),
            yaw bit (10) = 1 (ignore), yaw_rate bit (11) = 0 (use)
        """
        if self._conn is None:
            return False
        
        try:
            # Default to BODY_OFFSET_NED (body-relative, FRD convention)
            if coordinate_frame is None:
                coordinate_frame = mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED
            
            # type_mask: ignore position, acceleration, force, yaw angle
            # Use only: vx, vy, vz, yaw_rate
            type_mask = (
                0b0000_0111_1100_0111  # = 0x07C7
                # Bits 0-2 (pos): ignored
                # Bits 3-5 (vel): used
                # Bits 6-8 (acc): ignored
                # Bit 9 (force): ignored
                # Bit 10 (yaw): ignored
                # Bit 11 (yaw_rate): used
            )
            
            self._conn.mav.set_position_target_local_ned_send(
                0,                          # time_boot_ms (0 = use system time)
                self._conn.target_system,
                self._conn.target_component,
                coordinate_frame,
                type_mask,
                0, 0, 0,                    # x, y, z (ignored)
                vx, vy, vz,                 # vx, vy, vz
                0, 0, 0,                    # afx, afy, afz (ignored)
                0,                          # yaw (ignored)
                yaw_rate,                   # yaw_rate
            )
            return True
            
        except Exception as e:
            import logging
            logging.getLogger(__name__).debug(f"Velocity command error: {e}")
            return False

    def send_gimbal_command(
        self,
        pitch: float,
        yaw: float,
        roll: float = 0.0,
    ) -> bool:
        """
        Send gimbal control command via COMMAND_LONG (MAV_CMD_DO_MOUNT_CONTROL).
        
        Controls gimbal orientation for target tracking during Task 2.
        
        Args:
            pitch: Gimbal pitch angle in degrees (-90 = down, 0 = horizon)
            yaw: Gimbal yaw angle in degrees relative to vehicle heading
            roll: Gimbal roll angle in degrees (usually 0)
        
        Returns:
            True if message sent successfully
        
        Note:
            MAV_CMD_DO_MOUNT_CONTROL (205) parameters:
            - param1: pitch (degrees * 100 for some systems, degrees for others)
            - param2: roll (degrees)
            - param3: yaw (degrees)
            - param7: MAV_MOUNT_MODE (2 = MAVLINK_TARGETING)
            
            For ArduPilot, mount must be configured with MNT1_TYPE parameter.
        """
        if self._conn is None:
            return False
        
        try:
            # Send MAV_CMD_DO_MOUNT_CONTROL
            self._conn.mav.command_long_send(
                self._conn.target_system,
                self._conn.target_component,
                mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL,
                0,                                      # confirmation
                pitch,                                  # param1: pitch (deg)
                roll,                                   # param2: roll (deg)
                yaw,                                    # param3: yaw (deg)
                0,                                      # param4: altitude (unused)
                0,                                      # param5: latitude (unused)
                0,                                      # param6: longitude (unused)
                mavutil.mavlink.MAV_MOUNT_MODE_MAVLINK_TARGETING,  # param7: mode
            )
            return True
            
        except Exception as e:
            import logging
            logging.getLogger(__name__).debug(f"Gimbal command error: {e}")
            return False

    def send_gimbal_rate_command(
        self,
        pitch_rate: float,
        yaw_rate: float,
        roll_rate: float = 0.0,
    ) -> bool:
        """
        Send gimbal angular rate command.
        
        Alternative gimbal control using rates instead of angles.
        Useful for smooth tracking with PID output.
        
        Args:
            pitch_rate: Pitch rate in deg/s (positive = tilt up)
            yaw_rate: Yaw rate in deg/s (positive = pan right)
            roll_rate: Roll rate in deg/s (usually 0)
        
        Returns:
            True if message sent successfully
        """
        if self._conn is None:
            return False
        
        try:
            # Use GIMBAL_MANAGER_SET_PITCHYAW for rate control
            # Note: This requires gimbal manager protocol support
            self._conn.mav.command_long_send(
                self._conn.target_system,
                self._conn.target_component,
                mavutil.mavlink.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW,
                0,                  # confirmation
                pitch_rate,         # param1: pitch angle or NaN
                yaw_rate,           # param2: yaw angle or NaN  
                pitch_rate,         # param3: pitch rate (deg/s)
                yaw_rate,           # param4: yaw rate (deg/s)
                0,                  # param5: flags
                0,                  # param6: reserved
                0,                  # param7: gimbal device ID (0 = primary)
            )
            return True
            
        except Exception as e:
            import logging
            logging.getLogger(__name__).debug(f"Gimbal rate command error: {e}")
            return False

    def trigger_payload(
        self,
        pwm_value: int,
        servo_channel: int = 9,
    ) -> bool:
        """
        Trigger payload (water pump) via MAV_CMD_DO_SET_SERVO.
        
        Activates the servo controlling the water pump for fire extinguishing.
        
        Args:
            pwm_value: PWM value in microseconds (typically 1000-2000)
                       - 1000: servo min position (pump off)
                       - 1500: servo center
                       - 2000: servo max position (pump on/full)
            servo_channel: Servo output channel (default 9, AUX1 on most FCs)
        
        Returns:
            True if message sent successfully
        
        Note:
            Ensure the servo channel is configured in ArduPilot:
            - SERVOx_FUNCTION = 0 (disabled/passthrough)
            - The PWM range should be calibrated for your pump/servo
        
        Safety:
            Call with pwm_value=1000 to turn off the pump after dispensing.
        """
        if self._conn is None:
            return False
        
        try:
            self._conn.mav.command_long_send(
                self._conn.target_system,
                self._conn.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                0,                  # confirmation
                servo_channel,      # param1: servo instance (1-indexed)
                pwm_value,          # param2: PWM value (us)
                0, 0, 0, 0, 0,      # param3-7: unused
            )
            return True
            
        except Exception as e:
            import logging
            logging.getLogger(__name__).debug(f"Payload trigger error: {e}")
            return False

    def stop_velocity(self) -> bool:
        """
        Send zero velocity command to stop movement.
        
        Convenience method for emergency stop or end of servoing.
        
        Returns:
            True if message sent successfully
        """
        return self.send_velocity_command(0.0, 0.0, 0.0, 0.0)

    def send_statustext(
        self,
        text: str,
        severity: int | None = None,
    ) -> bool:
        """
        Send STATUSTEXT message to GCS.
        
        Used to broadcast status information visible in Mission Planner's
        HUD or Messages tab.
        
        Args:
            text: Status text message (max 50 chars)
            severity: MAV_SEVERITY level (default: INFO)
        
        Returns:
            True if message sent successfully
        """
        if self._conn is None:
            return False
        
        try:
            if severity is None:
                severity = mavutil.mavlink.MAV_SEVERITY_INFO
            
            # STATUSTEXT has max 50 characters
            text = text[:50]
            
            self._conn.mav.statustext_send(
                severity,
                text.encode('utf-8'),
            )
            return True
            
        except Exception as e:
            import logging
            logging.getLogger(__name__).debug(f"Statustext error: {e}")
            return False

    def start_health_broadcast(self, interval: float = 2.0) -> None:
        """
        Start background task to broadcast hardware health via STATUSTEXT.
        
        Sends "NOMAD: CPU 55C | GPU 80% | NVMe 10GB" every interval.
        
        Args:
            interval: Broadcast interval in seconds (default: 2.0)
        """
        if hasattr(self, '_health_thread') and self._health_thread is not None:
            return  # Already running
        
        self._health_interval = interval
        self._health_stop_event = threading.Event()
        self._health_thread = threading.Thread(
            target=self._broadcast_health_loop,
            daemon=True,
            name="nomad-health-broadcast",
        )
        self._health_thread.start()

    def stop_health_broadcast(self) -> None:
        """Stop the health broadcast background task."""
        if hasattr(self, '_health_stop_event'):
            self._health_stop_event.set()
        if hasattr(self, '_health_thread') and self._health_thread:
            self._health_thread.join(timeout=2.0)
            self._health_thread = None

    def _broadcast_health_loop(self) -> None:
        """Background loop to broadcast health status."""
        import logging
        logger = logging.getLogger(__name__)
        
        while not self._health_stop_event.is_set():
            try:
                # Get current system state
                state = self.state_manager.get_state()
                
                # Format health status message
                # "NOMAD: CPU 55C | GPU 80% | NVMe 10GB"
                cpu_temp = state.cpu_temp_c if state.cpu_temp_c else 0
                gpu_load = state.gpu_load_pct if state.gpu_load_pct else 0
                disk_free = state.disk_free_gb if state.disk_free_gb else 0
                
                status_msg = (
                    f"NOMAD: CPU {cpu_temp:.0f}C | "
                    f"GPU {gpu_load:.0f}% | "
                    f"NVMe {disk_free:.0f}GB"
                )
                
                # Send as STATUSTEXT
                if self.send_statustext(status_msg):
                    logger.debug(f"Health broadcast: {status_msg}")
                
            except Exception as e:
                logger.debug(f"Health broadcast error: {e}")
            
            # Wait for next interval
            self._health_stop_event.wait(self._health_interval)

    def set_mode(self, mode_id: int) -> bool:
        """
        Set the flight mode on the autopilot.
        
        Used by NavController to request GUIDED mode for Jetson-centric navigation.
        
        Args:
            mode_id: ArduPilot mode ID
                     - 0: STABILIZE
                     - 2: ALT_HOLD
                     - 4: GUIDED
                     - 5: LOITER
                     - 6: RTL
                     - 9: LAND
        
        Returns:
            True if mode change command was sent successfully
        """
        if self._conn is None:
            return False
        
        try:
            # Custom mode for Copter
            self._conn.mav.command_long_send(
                self._conn.target_system,
                self._conn.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                0,              # confirmation
                1,              # base mode (MAV_MODE_FLAG_CUSTOM_MODE_ENABLED)
                mode_id,        # custom mode
                0, 0, 0, 0, 0,  # unused params
            )
            return True
            
        except Exception as e:
            import logging
            logging.getLogger(__name__).debug(f"Set mode error: {e}")
            return False

    def send_position_target(
        self,
        x: float,
        y: float,
        z: float,
        yaw: float,
    ) -> bool:
        """
        Send position target via SET_POSITION_TARGET_LOCAL_NED.
        
        Used by NavController for position-based navigation commands.
        
        Args:
            x: North position in meters (NED frame)
            y: East position in meters (NED frame)
            z: Down position in meters (NED frame, positive = down)
            yaw: Heading in radians
        
        Returns:
            True if message sent successfully
        """
        if self._conn is None:
            return False
        
        try:
            # type_mask: ignore velocity, acceleration, force, yaw_rate
            # Use only: x, y, z, yaw
            type_mask = (
                0b0000_1111_1111_1000  # = 0x0FF8
                # Bits 0-2 (pos): used
                # Bits 3-5 (vel): ignored
                # Bits 6-8 (acc): ignored
                # Bit 9 (force): ignored
                # Bit 10 (yaw): used
                # Bit 11 (yaw_rate): ignored
            )
            
            self._conn.mav.set_position_target_local_ned_send(
                0,                          # time_boot_ms (0 = use system time)
                self._conn.target_system,
                self._conn.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                type_mask,
                x, y, z,                    # x, y, z position
                0, 0, 0,                    # vx, vy, vz (ignored)
                0, 0, 0,                    # afx, afy, afz (ignored)
                yaw,                        # yaw
                0,                          # yaw_rate (ignored)
            )
            return True
            
        except Exception as e:
            import logging
            logging.getLogger(__name__).debug(f"Position target error: {e}")
            return False

    def send_obstacle_distance(
        self,
        distances: list[int],
        increment: int = 5,
        min_distance: int = 20,
        max_distance: int = 2000,
        angle_offset: int = 0,
        frame: int = 0,
    ) -> bool:
        """
        Send OBSTACLE_DISTANCE message to ArduPilot (NV-008).

        Provides proximity sensor data for obstacle avoidance. ArduPilot uses
        this to avoid obstacles in the horizontal plane during autonomous flight.

        Args:
            distances: List of distances in centimeters for each angular sector.
                       Length must be 72 for 5-degree increments.
                       Use max_distance (or 0) for no obstacle detected.
            increment: Angular width of each sector in degrees (default: 5).
            min_distance: Minimum valid distance in cm (default: 20).
            max_distance: Maximum valid distance in cm (default: 2000 = 20m).
            angle_offset: Starting angle in degrees (default: 0 = forward).
            frame: MAV_FRAME (0 = BODY_FRD, default).

        Returns:
            True if message sent successfully.
        """
        if self._conn is None:
            return False

        try:
            # Pad or truncate to exactly 72 values
            dist_array = list(distances[:72])
            while len(dist_array) < 72:
                dist_array.append(max_distance)

            self._conn.mav.obstacle_distance_send(
                int(time.time() * 1e3) & 0xFFFFFFFF,  # time_usec (ms, truncated)
                0,                    # sensor_type: MAV_DISTANCE_SENSOR_UNKNOWN
                dist_array,           # distances[72] in cm
                increment,            # angular width per sector
                min_distance,         # min distance cm
                max_distance,         # max distance cm
                float(increment),     # increment_f (float version)
                float(angle_offset),  # angle_offset (float)
                frame,                # MAV_FRAME
            )
            return True

        except Exception as e:
            import logging
            logging.getLogger(__name__).debug(f"Obstacle distance error: {e}")
            return False

