"""
Facade wrapper expected by main.py

Provides a higher-level handler that uses MAVLinkConnection and exposes
signals and methods used by the UI code in main.py.
"""

from PyQt6.QtCore import QObject, pyqtSignal

from .mavlink_connection import MAVLinkConnection, ConnectionType
from .flight_controller import FlightController, FlightMode


class MAVLinkHandler(QObject):
    """Thin wrapper that owns MAVLinkConnection and FlightController."""

    # Signals mirrored to UI
    message_received = pyqtSignal(object)
    connection_lost = pyqtSignal()

    def __init__(self, config: dict):
        super().__init__()
        self._config = config or {}
        self.connection = MAVLinkConnection()
        self.flight = FlightController(self.connection)

        # Bridge signals
        self.connection.message_received.connect(self.message_received)
        self.connection.connection_lost.connect(self.connection_lost)

    # Lifecycle
    def start(self):
        ctype = (self._config.get('type') or '').lower()
        if ctype == 'serial':
            port = self._config.get('port') or ''
            baud = int(self._config.get('baudrate') or 115200)
            self.connection.connect_serial(port, baud)
        elif ctype == 'udp':
            host = self._config.get('host') or '127.0.0.1'
            port = int(self._config.get('port') or 14550)
            local_port = int(self._config.get('local_port') or 14551)
            self.connection.connect_udp(host, port, local_port)
        elif ctype == 'tcp':
            host = self._config.get('host') or '127.0.0.1'
            port = int(self._config.get('port') or 5760)
            self.connection.connect_tcp(host, port)
        else:
            raise ValueError(f"Unknown connection type: {ctype}")

    def stop(self):
        self.connection.disconnect()

    # Status helpers
    def is_connected(self) -> bool:
        return self.connection.is_connected()

    def get_last_message_time(self) -> float:
        import time
        if not self.connection.last_heartbeat:
            return float('inf')
        return max(0.0, time.time() - self.connection.last_heartbeat)

    # High-level commands used by UI
    def arm_vehicle(self) -> bool:
        return self.flight.arm()

    def disarm_vehicle(self) -> bool:
        return self.flight.disarm()

    def takeoff(self, altitude: float) -> bool:
        return self.flight.takeoff(float(altitude))

    def land(self) -> bool:
        return self.flight.land()

    def return_to_launch(self) -> bool:
        return self.flight.return_to_launch()

    def set_mode(self, mode: str) -> bool:
        try:
            fm = FlightMode(mode)
        except Exception:
            # Fallback: pass plain string to connection
            return self.connection.set_mode(mode)
        return self.flight.set_mode(fm)

    # Mission helpers to match main.py uses
    def upload_mission(self, waypoints: list) -> bool:
        # Expect list of dicts with lat/lon/alt
        # Provide a minimal implementation that sends first point as guided goto
        if not waypoints:
            return False
        wp = waypoints[0]
        return self.goto_waypoint(wp.get('lat', 0), wp.get('lon', 0), wp.get('alt', 10))

    def download_mission(self) -> list:
        # Placeholder: real mission manager not wired here. Return empty list.
        return []

    def goto_waypoint(self, lat: float, lon: float, alt: float) -> bool:
        return self.flight.goto_position(float(lat), float(lon), float(alt))


