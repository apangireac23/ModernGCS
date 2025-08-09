
"""
Flight control interface for ArduPilot vehicles
Provides high-level flight control commands
"""

from enum import Enum
from typing import Optional, Dict, Any, Tuple
from PyQt6.QtCore import QObject, pyqtSignal, QTimer
from pymavlink.dialects.v20 import ardupilotmega as mavlink

class FlightMode(Enum):
    """ArduPilot flight modes"""
    # Copter modes
    STABILIZE = "STABILIZE"
    ACRO = "ACRO"
    ALT_HOLD = "ALT_HOLD"
    AUTO = "AUTO"
    GUIDED = "GUIDED"
    LOITER = "LOITER"
    RTL = "RTL"
    CIRCLE = "CIRCLE"
    LAND = "LAND"
    DRIFT = "DRIFT"
    SPORT = "SPORT"
    FLIP = "FLIP"
    AUTOTUNE = "AUTOTUNE"
    POSHOLD = "POSHOLD"
    BRAKE = "BRAKE"
    THROW = "THROW"
    AVOID_ADSB = "AVOID_ADSB"
    GUIDED_NOGPS = "GUIDED_NOGPS"
    SMART_RTL = "SMART_RTL"
    FLOWHOLD = "FLOWHOLD"
    FOLLOW = "FOLLOW"
    ZIGZAG = "ZIGZAG"
    SYSTEMID = "SYSTEMID"
    AUTOROTATE = "AUTOROTATE"

class VehicleState(Enum):
    """Vehicle states"""
    UNKNOWN = "unknown"
    DISARMED = "disarmed"
    ARMED = "armed"
    FLYING = "flying"
    LANDED = "landed"
    ERROR = "error"

class FlightController(QObject):
    """
    High-level flight controller interface for ArduPilot vehicles
    """

    # Signals
    armed_changed = pyqtSignal(bool)  # Armed state
    mode_changed = pyqtSignal(str)    # Flight mode
    state_changed = pyqtSignal(object)  # Vehicle state (VehicleState)
    altitude_changed = pyqtSignal(float)      # Altitude in meters
    position_changed = pyqtSignal(float, float, float)  # lat, lon, alt
    attitude_changed = pyqtSignal(float, float, float)  # roll, pitch, yaw (radians)
    velocity_changed = pyqtSignal(float, float, float)  # vx, vy, vz (m/s)
    battery_changed = pyqtSignal(float, float, int)     # voltage, current, remaining%
    gps_changed = pyqtSignal(int, int, float)           # fix_type, satellites, hdop
    command_ack = pyqtSignal(int, int)                  # command, result

    def __init__(self, connection):
        super().__init__()

        self.connection = connection

        # Vehicle state
        self.armed = False
        self.current_mode = "UNKNOWN"
        self.vehicle_state = VehicleState.UNKNOWN

        # Position and attitude
        self.latitude = 0.0
        self.longitude = 0.0
        self.altitude_msl = 0.0
        self.altitude_rel = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        # Velocity
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.groundspeed = 0.0
        self.airspeed = 0.0

        # System status
        self.battery_voltage = 0.0
        self.battery_current = 0.0
        self.battery_remaining = 0
        self.gps_fix_type = 0
        self.gps_satellites = 0
        self.gps_hdop = 0.0

        # Mission state
        self.mission_current = 0
        self.mission_count = 0
        self.mission_item_reached = False

        # Connect to MAVLink connection
        self.connection.message_received.connect(self._handle_message)

        # Message handlers
        self.message_handlers = {
            'HEARTBEAT': self._handle_heartbeat,
            'ATTITUDE': self._handle_attitude,
            'GLOBAL_POSITION_INT': self._handle_global_position,
            'LOCAL_POSITION_NED': self._handle_local_position,
            'VFR_HUD': self._handle_vfr_hud,
            'SYS_STATUS': self._handle_sys_status,
            'GPS_RAW_INT': self._handle_gps_raw,
            'MISSION_CURRENT': self._handle_mission_current,
            'MISSION_ITEM_REACHED': self._handle_mission_item_reached,
            'COMMAND_ACK': self._handle_command_ack,
            'STATUSTEXT': self._handle_statustext
        }

        # Request data streams
        QTimer.singleShot(1000, self._request_data_streams)  # Delay to ensure connection is ready

    def _handle_message(self, msg):
        """Handle incoming MAVLink messages"""
        msg_type = msg.get_type()
        if msg_type in self.message_handlers:
            self.message_handlers[msg_type](msg)

    def _handle_heartbeat(self, msg):
        """Handle HEARTBEAT messages"""
        try:
            # Check armed state
            new_armed = bool(msg.base_mode & mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            if new_armed != self.armed:
                self.armed = new_armed
                self.armed_changed.emit(self.armed)

            # Check flight mode (custom mode for ArduPilot)
            mode_mapping = {
                0: "STABILIZE", 1: "ACRO", 2: "ALT_HOLD", 3: "AUTO", 4: "GUIDED",
                5: "LOITER", 6: "RTL", 7: "CIRCLE", 9: "LAND", 11: "DRIFT",
                13: "SPORT", 14: "FLIP", 15: "AUTOTUNE", 16: "POSHOLD", 17: "BRAKE",
                18: "THROW", 19: "AVOID_ADSB", 20: "GUIDED_NOGPS", 21: "SMART_RTL",
                22: "FLOWHOLD", 23: "FOLLOW", 24: "ZIGZAG", 25: "SYSTEMID", 26: "AUTOROTATE"
            }

            if msg.base_mode & mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED:
                new_mode = mode_mapping.get(msg.custom_mode, f"CUSTOM_{msg.custom_mode}")
            else:
                new_mode = "MANUAL" if msg.base_mode & mavlink.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED else "UNKNOWN"

            if new_mode != self.current_mode:
                self.current_mode = new_mode
                self.mode_changed.emit(self.current_mode)

            # Update vehicle state
            new_state = VehicleState.UNKNOWN
            if not self.armed:
                new_state = VehicleState.DISARMED
            elif msg.system_status == mavlink.MAV_STATE_ACTIVE:
                new_state = VehicleState.FLYING if self.altitude_rel > 0.5 else VehicleState.ARMED
            elif msg.system_status in [mavlink.MAV_STATE_EMERGENCY, mavlink.MAV_STATE_CRITICAL]:
                new_state = VehicleState.ERROR
            else:
                new_state = VehicleState.ARMED

            if new_state != self.vehicle_state:
                self.vehicle_state = new_state
                self.state_changed.emit(self.vehicle_state)

        except Exception as e:
            print(f"Error handling heartbeat: {e}")

    def _handle_attitude(self, msg):
        """Handle ATTITUDE messages"""
        try:
            self.roll = msg.roll
            self.pitch = msg.pitch  
            self.yaw = msg.yaw
            self.attitude_changed.emit(self.roll, self.pitch, self.yaw)
        except Exception as e:
            print(f"Error handling attitude: {e}")

    def _handle_global_position(self, msg):
        """Handle GLOBAL_POSITION_INT messages"""
        try:
            self.latitude = msg.lat / 1e7
            self.longitude = msg.lon / 1e7
            self.altitude_msl = msg.alt / 1000.0  # mm to m
            self.altitude_rel = msg.relative_alt / 1000.0  # mm to m

            self.vx = msg.vx / 100.0  # cm/s to m/s
            self.vy = msg.vy / 100.0
            self.vz = msg.vz / 100.0

            self.position_changed.emit(self.latitude, self.longitude, self.altitude_rel)
            self.altitude_changed.emit(self.altitude_rel)
            self.velocity_changed.emit(self.vx, self.vy, self.vz)
        except Exception as e:
            print(f"Error handling global position: {e}")

    def _handle_local_position(self, msg):
        """Handle LOCAL_POSITION_NED messages"""
        try:
            # Update velocity if not available from global position
            if self.vx == 0 and self.vy == 0 and self.vz == 0:
                self.vx = msg.vx
                self.vy = msg.vy
                self.vz = msg.vz
                self.velocity_changed.emit(self.vx, self.vy, self.vz)
        except Exception as e:
            print(f"Error handling local position: {e}")

    def _handle_vfr_hud(self, msg):
        """Handle VFR_HUD messages"""
        try:
            self.groundspeed = msg.groundspeed
            self.airspeed = msg.airspeed
        except Exception as e:
            print(f"Error handling VFR HUD: {e}")

    def _handle_sys_status(self, msg):
        """Handle SYS_STATUS messages"""
        try:
            self.battery_voltage = msg.voltage_battery / 1000.0  # mV to V
            self.battery_current = msg.current_battery / 100.0   # cA to A
            self.battery_remaining = msg.battery_remaining
            self.battery_changed.emit(self.battery_voltage, self.battery_current, self.battery_remaining)
        except Exception as e:
            print(f"Error handling system status: {e}")

    def _handle_gps_raw(self, msg):
        """Handle GPS_RAW_INT messages"""
        try:
            self.gps_fix_type = msg.fix_type
            self.gps_satellites = msg.satellites_visible
            self.gps_hdop = msg.eph / 100.0 if msg.eph != 65535 else 99.99
            self.gps_changed.emit(self.gps_fix_type, self.gps_satellites, self.gps_hdop)
        except Exception as e:
            print(f"Error handling GPS raw: {e}")

    def _handle_mission_current(self, msg):
        """Handle MISSION_CURRENT messages"""
        try:
            self.mission_current = msg.seq
        except Exception as e:
            print(f"Error handling mission current: {e}")

    def _handle_mission_item_reached(self, msg):
        """Handle MISSION_ITEM_REACHED messages"""
        try:
            self.mission_item_reached = True
        except Exception as e:
            print(f"Error handling mission item reached: {e}")

    def _handle_command_ack(self, msg):
        """Handle COMMAND_ACK messages"""
        try:
            self.command_ack.emit(msg.command, msg.result)
        except Exception as e:
            print(f"Error handling command ack: {e}")

    def _handle_statustext(self, msg):
        """Handle STATUSTEXT messages"""
        try:
            severity = msg.severity
            text = msg.text.decode('utf-8').rstrip('\x00')
            print(f"Vehicle Status [{severity}]: {text}")
        except Exception as e:
            print(f"Error handling status text: {e}")

    def _request_data_streams(self):
        """Request data streams from vehicle"""
        if not self.connection.is_connected():
            return

        # Request various data streams at appropriate rates
        streams = [
            (mavlink.MAV_DATA_STREAM_EXTENDED_STATUS, 2),   # 2 Hz
            (mavlink.MAV_DATA_STREAM_POSITION, 10),         # 10 Hz
            (mavlink.MAV_DATA_STREAM_EXTRA1, 10),           # 10 Hz (attitude)
            (mavlink.MAV_DATA_STREAM_EXTRA2, 10),           # 10 Hz (VFR_HUD)
            (mavlink.MAV_DATA_STREAM_RC_CHANNELS, 5),       # 5 Hz
        ]

        for stream_id, rate in streams:
            try:
                msg = self.connection.mavlink_connection.mav.request_data_stream_encode(
                    self.connection.system_id,
                    self.connection.component_id,
                    stream_id,
                    rate,
                    1  # start/stop (1=start)
                )
                self.connection.send_message(msg)
            except Exception as e:
                print(f"Failed to request data stream {stream_id}: {e}")

    # Flight Control Commands

    def arm(self) -> bool:
        """Arm the vehicle"""
        return self.connection.arm_disarm(True)

    def disarm(self) -> bool:
        """Disarm the vehicle"""
        return self.connection.arm_disarm(False)

    def takeoff(self, altitude: float) -> bool:
        """Takeoff to specified altitude (meters)"""
        return self.connection.takeoff(altitude)

    def land(self) -> bool:
        """Land at current position"""
        return self.connection.land()

    def return_to_launch(self) -> bool:
        """Return to launch point"""
        return self.connection.return_to_launch()

    def set_mode(self, mode: FlightMode) -> bool:
        """Set flight mode"""
        return self.connection.set_mode(mode.value)

    def goto_position(self, lat: float, lon: float, alt: float) -> bool:
        """Go to specified position in guided mode"""
        if not self.connection.is_connected():
            return False

        try:
            # Send SET_POSITION_TARGET_GLOBAL_INT message
            msg = self.connection.mavlink_connection.mav.set_position_target_global_int_encode(
                0,  # timestamp
                self.connection.system_id,
                self.connection.component_id,
                mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                0b0000111111111000,  # type_mask (only position)
                int(lat * 1e7),   # lat_int
                int(lon * 1e7),   # lon_int
                alt,              # alt
                0, 0, 0,          # velocity (not used)
                0, 0, 0,          # acceleration (not used)
                0, 0              # yaw, yaw_rate (not used)
            )
            return self.connection.send_message(msg)
        except Exception as e:
            print(f"Failed to go to position: {e}")
            return False

    def set_velocity(self, vx: float, vy: float, vz: float, yaw_rate: float = 0.0) -> bool:
        """Set velocity in body frame (m/s)"""
        if not self.connection.is_connected():
            return False

        try:
            msg = self.connection.mavlink_connection.mav.set_position_target_local_ned_encode(
                0,  # timestamp
                self.connection.system_id,
                self.connection.component_id,
                mavlink.MAV_FRAME_BODY_OFFSET_NED,
                0b0000111111000111,  # type_mask (only velocity and yaw rate)
                0, 0, 0,             # position (not used)
                vx, vy, vz,          # velocity
                0, 0, 0,             # acceleration (not used)
                0,                   # yaw (not used)
                yaw_rate             # yaw_rate
            )
            return self.connection.send_message(msg)
        except Exception as e:
            print(f"Failed to set velocity: {e}")
            return False

    def set_yaw(self, yaw_degrees: float, relative: bool = False) -> bool:
        """Set yaw angle"""
        if not self.connection.is_connected():
            return False

        try:
            msg = self.connection.mavlink_connection.mav.command_long_encode(
                self.connection.system_id,
                self.connection.component_id,
                mavlink.MAV_CMD_CONDITION_YAW,
                0,  # confirmation
                yaw_degrees,  # param1: target angle
                0,            # param2: yaw speed (0 = default)
                1 if yaw_degrees >= 0 else -1,  # param3: direction
                1 if relative else 0,  # param4: relative (1) or absolute (0)
                0, 0, 0       # param5-7: unused
            )
            return self.connection.send_message(msg)
        except Exception as e:
            print(f"Failed to set yaw: {e}")
            return False

    # Utility methods

    def get_vehicle_info(self) -> Dict[str, Any]:
        """Get comprehensive vehicle information"""
        return {
            'armed': self.armed,
            'mode': self.current_mode,
            'state': self.vehicle_state.value,
            'position': {
                'lat': self.latitude,
                'lon': self.longitude,
                'alt_msl': self.altitude_msl,
                'alt_rel': self.altitude_rel
            },
            'attitude': {
                'roll': self.roll,
                'pitch': self.pitch,
                'yaw': self.yaw
            },
            'velocity': {
                'vx': self.vx,
                'vy': self.vy,
                'vz': self.vz,
                'groundspeed': self.groundspeed,
                'airspeed': self.airspeed
            },
            'battery': {
                'voltage': self.battery_voltage,
                'current': self.battery_current,
                'remaining': self.battery_remaining
            },
            'gps': {
                'fix_type': self.gps_fix_type,
                'satellites': self.gps_satellites,
                'hdop': self.gps_hdop
            },
            'mission': {
                'current': self.mission_current,
                'count': self.mission_count
            }
        }

    def is_armable(self) -> Tuple[bool, str]:
        """Check if vehicle is ready to arm"""
        if self.armed:
            return False, "Vehicle is already armed"

        if self.gps_fix_type < 3:
            return False, "GPS fix required (3D fix)"

        if self.battery_voltage < 10.0:  # Adjust based on your setup
            return False, "Battery voltage too low"

        if self.current_mode not in ["STABILIZE", "ALT_HOLD", "LOITER", "GUIDED", "POSHOLD"]:
            return False, f"Cannot arm in {self.current_mode} mode"

        return True, "Ready to arm"

    def is_ready_for_takeoff(self) -> Tuple[bool, str]:
        """Check if vehicle is ready for takeoff"""
        if not self.armed:
            return False, "Vehicle must be armed first"

        if self.current_mode not in ["GUIDED", "AUTO", "LOITER", "ALT_HOLD"]:
            return False, f"Cannot takeoff in {self.current_mode} mode"

        if self.altitude_rel > 0.5:
            return False, "Vehicle appears to already be airborne"

        return True, "Ready for takeoff"
