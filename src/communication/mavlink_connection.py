
"""
MAVLink connection management for ArduPilot GCS
Supports serial, TCP, and UDP connections
"""

import threading
import time
from enum import Enum
from typing import Optional, Dict, Any, Callable
from PyQt6.QtCore import QObject, pyqtSignal, QTimer
import socket
import serial
import struct

try:
    from pymavlink import mavutil, mavwp
    from pymavlink.mavutil import mavlink
except ImportError:
    print("pymavlink not installed. Install with: pip install pymavlink")
    raise


class ConnectionType(Enum):
    """Connection types"""
    SERIAL = "serial"
    UDP = "udp"
    TCP = "tcp"


class ConnectionState(Enum):
    """Connection states"""
    DISCONNECTED = "disconnected"
    CONNECTING = "connecting"
    CONNECTED = "connected"
    ERROR = "error"


class MAVLinkConnection(QObject):
    """MAVLink connection manager"""

    # Signals
    connection_state_changed = pyqtSignal(ConnectionState)
    status_message = pyqtSignal(str)
    message_received = pyqtSignal(object)
    connection_lost = pyqtSignal()

    def __init__(self):
        super().__init__()

        # Connection parameters
        self.connection = None
        self.connection_type = None
        self.connection_params = {}
        self.state = ConnectionState.DISCONNECTED

        # Message handling
        self.message_callbacks = {}
        self.messages_received = 0
        self.last_heartbeat = 0

        # Threading
        self.receive_thread = None
        self.heartbeat_thread = None
        self.running = False

        # Vehicle info
        self.vehicle_type = None
        self.autopilot_type = None
        self.system_id = 1
        self.component_id = 1

        # Heartbeat timer
        self.heartbeat_timer = QTimer()
        self.heartbeat_timer.timeout.connect(self.check_heartbeat)
        self.heartbeat_timer.start(5000)  # Check every 5 seconds

    def connect_serial(self, port: str, baudrate: int = 115200) -> bool:
        """Connect via serial port"""
        try:
            connection_string = f"{port},{baudrate}"
            self.connection = mavutil.mavlink_connection(connection_string)
            self.connection_type = ConnectionType.SERIAL
            self.connection_params = {'port': port, 'baudrate': baudrate}

            return self._initialize_connection()

        except Exception as e:
            self.status_message.emit(f"Serial connection failed: {str(e)}")
            return False

    def connect_udp(self, host: str = "127.0.0.1", port: int = 14550, local_port: int = 14551) -> bool:
        """Connect via UDP"""
        try:
            connection_string = f"udp:{host}:{port}"
            if local_port != 14551:
                connection_string = f"udpin:{local_port}"

            self.connection = mavutil.mavlink_connection(connection_string)
            self.connection_type = ConnectionType.UDP
            self.connection_params = {'host': host, 'port': port, 'local_port': local_port}

            return self._initialize_connection()

        except Exception as e:
            self.status_message.emit(f"UDP connection failed: {str(e)}")
            return False

    def connect_tcp(self, host: str, port: int = 5760) -> bool:
        """Connect via TCP"""
        try:
            connection_string = f"tcp:{host}:{port}"
            self.connection = mavutil.mavlink_connection(connection_string)
            self.connection_type = ConnectionType.TCP
            self.connection_params = {'host': host, 'port': port}

            return self._initialize_connection()

        except Exception as e:
            self.status_message.emit(f"TCP connection failed: {str(e)}")
            return False

    def _initialize_connection(self) -> bool:
        """Initialize connection after creation"""
        if not self.connection:
            return False

        try:
            self.state = ConnectionState.CONNECTING
            self.connection_state_changed.emit(self.state)

            # Wait for first heartbeat
            self.status_message.emit("Waiting for heartbeat...")

            start_time = time.time()
            timeout = 10.0  # 10 second timeout

            while time.time() - start_time < timeout:
                msg = self.connection.recv_match(type='HEARTBEAT', blocking=False, timeout=1)
                if msg:
                    self.vehicle_type = msg.type
                    self.autopilot_type = msg.autopilot
                    self.system_id = msg.get_srcSystem()
                    self.component_id = msg.get_srcComponent()

                    self.last_heartbeat = time.time()
                    self.messages_received += 1

                    self.state = ConnectionState.CONNECTED
                    self.connection_state_changed.emit(self.state)

                    # Start receive thread
                    self.running = True
                    self.receive_thread = threading.Thread(target=self._receive_loop, daemon=True)
                    self.receive_thread.start()

                    self.status_message.emit("Connected to vehicle")
                    return True

            # Timeout
            self.connection.close()
            self.connection = None
            self.state = ConnectionState.ERROR
            self.connection_state_changed.emit(self.state)
            self.status_message.emit("Connection timeout - no heartbeat received")
            return False

        except Exception as e:
            self.status_message.emit(f"Connection initialization failed: {str(e)}")
            self.state = ConnectionState.ERROR
            self.connection_state_changed.emit(self.state)
            return False

    def disconnect(self):
        """Disconnect from vehicle"""
        self.running = False

        if self.receive_thread and self.receive_thread.is_alive():
            self.receive_thread.join(timeout=2.0)

        if self.connection:
            try:
                self.connection.close()
            except:
                pass
            self.connection = None

        self.state = ConnectionState.DISCONNECTED
        self.connection_state_changed.emit(self.state)
        self.status_message.emit("Disconnected")

    def is_connected(self) -> bool:
        """Check if connected"""
        return self.state == ConnectionState.CONNECTED and self.connection is not None

    def _receive_loop(self):
        """Message receive loop (runs in separate thread)"""
        while self.running and self.connection:
            try:
                msg = self.connection.recv_match(blocking=False, timeout=0.1)
                if msg:
                    self.messages_received += 1

                    # Update heartbeat time
                    if msg.get_type() == 'HEARTBEAT':
                        self.last_heartbeat = time.time()

                    # Emit signal for message
                    self.message_received.emit(msg)

                    # Call registered callbacks
                    msg_type = msg.get_type()
                    if msg_type in self.message_callbacks:
                        for callback in self.message_callbacks[msg_type]:
                            try:
                                callback(msg)
                            except Exception as e:
                                print(f"Error in message callback: {e}")

                time.sleep(0.01)  # Small delay to prevent excessive CPU usage

            except Exception as e:
                if self.running:
                    print(f"Receive loop error: {e}")
                    self.connection_lost.emit()
                break

    def check_heartbeat(self):
        """Check if heartbeat is still being received"""
        if self.state == ConnectionState.CONNECTED:
            if time.time() - self.last_heartbeat > 10:  # 10 second timeout
                self.connection_lost.emit()
                self.disconnect()

    def send_message(self, msg) -> bool:
        """Send MAVLink message"""
        if not self.is_connected():
            return False

        try:
            # Set target system/component if not set
            if hasattr(msg, 'target_system') and msg.target_system == 0:
                msg.target_system = self.system_id
            if hasattr(msg, 'target_component') and msg.target_component == 0:
                msg.target_component = self.component_id

            self.connection.mav.send(msg)
            return True
        except Exception as e:
            self.status_message.emit(f"Failed to send message: {str(e)}")
            return False

    def add_message_callback(self, msg_type: str, callback: Callable):
        """Add callback for specific message type"""
        if msg_type not in self.message_callbacks:
            self.message_callbacks[msg_type] = []
        self.message_callbacks[msg_type].append(callback)

    def remove_message_callback(self, msg_type: str, callback: Callable):
        """Remove message callback"""
        if msg_type in self.message_callbacks:
            if callback in self.message_callbacks[msg_type]:
                self.message_callbacks[msg_type].remove(callback)

    def get_connection_info(self) -> Dict[str, Any]:
        """Get connection information"""
        return {
            'state': self.state,
            'type': self.connection_type,
            'params': self.connection_params,
            'vehicle_type': self.vehicle_type,
            'autopilot_type': self.autopilot_type,
            'system_id': self.system_id,
            'component_id': self.component_id,
            'messages_received': self.messages_received,
            'last_heartbeat': self.last_heartbeat
        }

    # Flight control commands

    def arm_disarm(self, arm: bool) -> bool:
        """Arm or disarm the vehicle"""
        if not self.is_connected():
            return False

        try:
            self.connection.mav.command_long_send(
                self.system_id,
                self.component_id,
                mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,  # confirmation
                1 if arm else 0,  # param1: arm/disarm
                0,  # param2: force
                0, 0, 0, 0, 0  # param3-7: unused
            )
            return True
        except Exception as e:
            self.status_message.emit(f"Failed to {'arm' if arm else 'disarm'}: {str(e)}")
            return False

    def takeoff(self, altitude: float) -> bool:
        """Takeoff to specified altitude"""
        if not self.is_connected():
            return False

        try:
            self.connection.mav.command_long_send(
                self.system_id,
                self.component_id,
                mavlink.MAV_CMD_NAV_TAKEOFF,
                0,  # confirmation
                0,  # param1: pitch angle
                0,  # param2: empty
                0,  # param3: empty
                0,  # param4: yaw angle
                0,  # param5: latitude
                0,  # param6: longitude
                altitude  # param7: altitude
            )
            return True
        except Exception as e:
            self.status_message.emit(f"Failed to takeoff: {str(e)}")
            return False

    def land(self) -> bool:
        """Land at current position"""
        if not self.is_connected():
            return False

        try:
            self.connection.mav.command_long_send(
                self.system_id,
                self.component_id,
                mavlink.MAV_CMD_NAV_LAND,
                0,  # confirmation
                0,  # param1: abort altitude
                0,  # param2: precision land mode
                0, 0,  # param3-4: empty
                0,  # param5: latitude (0 = current)
                0,  # param6: longitude (0 = current)
                0   # param7: altitude (0 = current)
            )
            return True
        except Exception as e:
            self.status_message.emit(f"Failed to land: {str(e)}")
            return False

    def return_to_launch(self) -> bool:
        """Return to launch point"""
        if not self.is_connected():
            return False

        try:
            self.connection.mav.command_long_send(
                self.system_id,
                self.component_id,
                mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
                0,  # confirmation
                0, 0, 0, 0, 0, 0, 0  # param1-7: unused
            )
            return True
        except Exception as e:
            self.status_message.emit(f"Failed to return to launch: {str(e)}")
            return False

    def set_mode(self, mode: str) -> bool:
        """Set flight mode"""
        if not self.is_connected():
            return False

        try:
            # ArduPilot mode mapping
            mode_mapping = {
                'STABILIZE': 0, 'ACRO': 1, 'ALT_HOLD': 2, 'AUTO': 3, 'GUIDED': 4,
                'LOITER': 5, 'RTL': 6, 'CIRCLE': 7, 'LAND': 9, 'DRIFT': 11,
                'SPORT': 13, 'FLIP': 14, 'AUTOTUNE': 15, 'POSHOLD': 16,
                'BRAKE': 17, 'THROW': 18, 'AVOID_ADSB': 19, 'GUIDED_NOGPS': 20,
                'SMART_RTL': 21, 'FLOWHOLD': 22, 'FOLLOW': 23, 'ZIGZAG': 24,
                'SYSTEMID': 25, 'AUTOROTATE': 26
            }

            if mode not in mode_mapping:
                self.status_message.emit(f"Unknown mode: {mode}")
                return False

            mode_id = mode_mapping[mode]

            self.connection.mav.set_mode_send(
                self.system_id,
                mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_id
            )
            return True
        except Exception as e:
            self.status_message.emit(f"Failed to set mode: {str(e)}")
            return False

    def set_home(self, lat: float = 0, lon: float = 0, alt: float = 0) -> bool:
        """Set home position"""
        if not self.is_connected():
            return False

        try:
            self.connection.mav.command_long_send(
                self.system_id,
                self.component_id,
                mavlink.MAV_CMD_DO_SET_HOME,
                0,  # confirmation
                1 if (lat == 0 and lon == 0) else 0,  # param1: use current position
                0,  # param2: unused
                0, 0,  # param3-4: unused
                lat,  # param5: latitude
                lon,  # param6: longitude
                alt   # param7: altitude
            )
            return True
        except Exception as e:
            self.status_message.emit(f"Failed to set home: {str(e)}")
            return False

    def goto_location(self, lat: float, lon: float, alt: float) -> bool:
        """Go to specified location"""
        if not self.is_connected():
            return False

        try:
            self.connection.mav.mission_item_send(
                self.system_id,
                self.component_id,
                0,  # seq
                mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                mavlink.MAV_CMD_NAV_WAYPOINT,
                2,  # current: 2 = guided mode waypoint
                1,  # autocontinue
                0,  # param1: hold time
                0,  # param2: acceptance radius
                0,  # param3: pass radius
                0,  # param4: yaw
                lat,  # x: latitude
                lon,  # y: longitude
                alt   # z: altitude
            )
            return True
        except Exception as e:
            self.status_message.emit(f"Failed to go to location: {str(e)}")
            return False

    def set_servo(self, channel: int, pwm: int) -> bool:
        """Set servo output"""
        if not self.is_connected():
            return False

        try:
            self.connection.mav.command_long_send(
                self.system_id,
                self.component_id,
                mavlink.MAV_CMD_DO_SET_SERVO,
                0,  # confirmation
                channel,  # param1: servo number
                pwm,  # param2: PWM value
                0, 0, 0, 0, 0  # param3-7: unused
            )
            return True
        except Exception as e:
            self.status_message.emit(f"Failed to set servo: {str(e)}")
            return False

    def request_data_stream(self, stream_id: int, rate: int) -> bool:
        """Request data stream at specified rate"""
        if not self.is_connected():
            return False

        try:
            self.connection.mav.request_data_stream_send(
                self.system_id,
                self.component_id,
                stream_id,
                rate,  # Hz
                1 if rate > 0 else 0  # start/stop
            )
            return True
        except Exception as e:
            self.status_message.emit(f"Failed to request data stream: {str(e)}")
            return False

    def setup_default_data_streams(self):
        """Setup default data streams"""
        streams = [
            (mavlink.MAV_DATA_STREAM_ALL, 2),
            (mavlink.MAV_DATA_STREAM_RAW_SENSORS, 10),
            (mavlink.MAV_DATA_STREAM_EXTENDED_STATUS, 2),
            (mavlink.MAV_DATA_STREAM_RC_CHANNELS, 5),
            (mavlink.MAV_DATA_STREAM_POSITION, 10),
            (mavlink.MAV_DATA_STREAM_EXTRA1, 10),  # Attitude
            (mavlink.MAV_DATA_STREAM_EXTRA2, 10),
            (mavlink.MAV_DATA_STREAM_EXTRA3, 2),
        ]

        for stream_id, rate in streams:
            self.request_data_stream(stream_id, rate)
