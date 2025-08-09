
"""
Settings and configuration management for the GCS
"""

from PyQt6.QtCore import QSettings
from typing import Any, Dict, Optional
import json
import os

class GCSSettings:
    """
    Centralized settings management for the GCS application
    """

    def __init__(self):
        self.settings = QSettings("ArduPilot", "GCS")
        self.default_values = {
            # Connection settings
            "connection/last_type": "udp",
            "connection/serial_port": "/dev/ttyUSB0",
            "connection/serial_baudrate": 57600,
            "connection/tcp_host": "127.0.0.1",
            "connection/tcp_port": 5760,
            "connection/udp_host": "127.0.0.1",
            "connection/udp_port": 14550,

            # Map settings
            "map/provider": "esri",
            "map/tile_server": "https://services.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
            "map/default_zoom": 15,
            "map/default_lat": 37.7749,
            "map/default_lon": -122.4194,

            # UI settings
            "ui/theme": "dark",
            "ui/window_width": 1200,
            "ui/window_height": 800,
            "ui/window_maximized": False,

            # Flight settings
            "flight/default_takeoff_altitude": 10.0,
            "flight/default_rtl_altitude": 50.0,
            "flight/auto_arm_timeout": 10,

            # Logging settings
            "logging/enabled": True,
            "logging/level": "INFO",
            "logging/log_directory": "./logs",
        }

    def get(self, key: str, default: Any = None) -> Any:
        """Get a setting value"""
        if default is None and key in self.default_values:
            default = self.default_values[key]

        return self.settings.value(key, default)

    def set(self, key: str, value: Any):
        """Set a setting value"""
        self.settings.setValue(key, value)
        self.settings.sync()

    def get_connection_settings(self) -> Dict[str, Any]:
        """Get all connection-related settings"""
        return {
            "last_type": self.get("connection/last_type"),
            "serial_port": self.get("connection/serial_port"),
            "serial_baudrate": int(self.get("connection/serial_baudrate")),
            "tcp_host": self.get("connection/tcp_host"),
            "tcp_port": int(self.get("connection/tcp_port")),
            "udp_host": self.get("connection/udp_host"),
            "udp_port": int(self.get("connection/udp_port")),
        }

    def set_connection_settings(self, settings: Dict[str, Any]):
        """Set connection-related settings"""
        for key, value in settings.items():
            self.set(f"connection/{key}", value)

    def get_map_settings(self) -> Dict[str, Any]:
        """Get all map-related settings"""
        return {
            "provider": self.get("map/provider"),
            "tile_server": self.get("map/tile_server"),
            "default_zoom": int(self.get("map/default_zoom")),
            "default_lat": float(self.get("map/default_lat")),
            "default_lon": float(self.get("map/default_lon")),
        }

    def set_map_settings(self, settings: Dict[str, Any]):
        """Set map-related settings"""
        for key, value in settings.items():
            self.set(f"map/{key}", value)

    def get_ui_settings(self) -> Dict[str, Any]:
        """Get all UI-related settings"""
        return {
            "theme": self.get("ui/theme"),
            "window_width": int(self.get("ui/window_width")),
            "window_height": int(self.get("ui/window_height")),
            "window_maximized": bool(self.get("ui/window_maximized")),
        }

    def set_ui_settings(self, settings: Dict[str, Any]):
        """Set UI-related settings"""
        for key, value in settings.items():
            self.set(f"ui/{key}", value)

    def get_flight_settings(self) -> Dict[str, Any]:
        """Get all flight-related settings"""
        return {
            "default_takeoff_altitude": float(self.get("flight/default_takeoff_altitude")),
            "default_rtl_altitude": float(self.get("flight/default_rtl_altitude")),
            "auto_arm_timeout": int(self.get("flight/auto_arm_timeout")),
        }

    def set_flight_settings(self, settings: Dict[str, Any]):
        """Set flight-related settings"""
        for key, value in settings.items():
            self.set(f"flight/{key}", value)

    def reset_to_defaults(self):
        """Reset all settings to default values"""
        self.settings.clear()
        for key, value in self.default_values.items():
            self.set(key, value)

    def export_settings(self, file_path: str) -> bool:
        """Export settings to JSON file"""
        try:
            settings_dict = {}
            for key in self.default_values.keys():
                settings_dict[key] = self.get(key)

            with open(file_path, 'w') as f:
                json.dump(settings_dict, f, indent=2)

            return True
        except Exception as e:
            print(f"Failed to export settings: {e}")
            return False

    def import_settings(self, file_path: str) -> bool:
        """Import settings from JSON file"""
        try:
            if not os.path.exists(file_path):
                return False

            with open(file_path, 'r') as f:
                settings_dict = json.load(f)

            for key, value in settings_dict.items():
                self.set(key, value)

            return True
        except Exception as e:
            print(f"Failed to import settings: {e}")
            return False

# Global settings instance
settings = GCSSettings()
