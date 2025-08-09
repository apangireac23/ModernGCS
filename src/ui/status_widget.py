
"""
Status widget showing vehicle information
"""

from PyQt6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
                            QLabel, QProgressBar, QGroupBox)
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QFont
import time

try:
    from pymavlink.mavutil import mavlink
except ImportError:
    mavlink = None


class StatusWidget(QWidget):
    """Widget showing vehicle status information"""

    def __init__(self, parent=None):
        super().__init__(parent)

        # Data storage
        self.last_heartbeat = 0
        self.armed = False
        self.mode = "Unknown"
        self.battery_voltage = 0.0
        self.battery_current = 0.0
        self.battery_remaining = -1
        self.gps_fix = 0
        self.gps_satellites = 0
        self.altitude = 0.0
        self.ground_speed = 0.0

        self.init_ui()

    def init_ui(self):
        """Initialize UI"""
        layout = QVBoxLayout(self)

        # Title
        title = QLabel("Vehicle Status")
        title_font = QFont()
        title_font.setBold(True)
        title_font.setPointSize(12)
        title.setFont(title_font)
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(title)

        # Status indicators
        status_group = QGroupBox("Status")
        status_layout = QGridLayout(status_group)

        # Armed status
        status_layout.addWidget(QLabel("Armed:"), 0, 0)
        self.armed_label = QLabel("Unknown")
        self.armed_label.setStyleSheet("font-weight: bold;")
        status_layout.addWidget(self.armed_label, 0, 1)

        # Flight mode
        status_layout.addWidget(QLabel("Mode:"), 1, 0)
        self.mode_label = QLabel("Unknown")
        self.mode_label.setStyleSheet("font-weight: bold;")
        status_layout.addWidget(self.mode_label, 1, 1)

        layout.addWidget(status_group)

        # Battery
        battery_group = QGroupBox("Battery")
        battery_layout = QVBoxLayout(battery_group)

        # Battery percentage
        battery_info_layout = QHBoxLayout()
        battery_info_layout.addWidget(QLabel("Level:"))
        self.battery_progress = QProgressBar()
        self.battery_progress.setRange(0, 100)
        self.battery_progress.setValue(0)
        battery_info_layout.addWidget(self.battery_progress)
        battery_layout.addLayout(battery_info_layout)

        # Battery voltage/current
        voltage_layout = QHBoxLayout()
        voltage_layout.addWidget(QLabel("Voltage:"))
        self.voltage_label = QLabel("0.0 V")
        voltage_layout.addWidget(self.voltage_label)
        voltage_layout.addStretch()
        battery_layout.addLayout(voltage_layout)

        current_layout = QHBoxLayout()
        current_layout.addWidget(QLabel("Current:"))
        self.current_label = QLabel("0.0 A")
        current_layout.addWidget(self.current_label)
        current_layout.addStretch()
        battery_layout.addLayout(current_layout)

        layout.addWidget(battery_group)

        # GPS
        gps_group = QGroupBox("GPS")
        gps_layout = QGridLayout(gps_group)

        gps_layout.addWidget(QLabel("Fix:"), 0, 0)
        self.gps_fix_label = QLabel("No Fix")
        gps_layout.addWidget(self.gps_fix_label, 0, 1)

        gps_layout.addWidget(QLabel("Satellites:"), 1, 0)
        self.gps_sat_label = QLabel("0")
        gps_layout.addWidget(self.gps_sat_label, 1, 1)

        layout.addWidget(gps_group)

        # Flight data
        flight_group = QGroupBox("Flight Data")
        flight_layout = QGridLayout(flight_group)

        flight_layout.addWidget(QLabel("Altitude:"), 0, 0)
        self.altitude_label = QLabel("0.0 m")
        flight_layout.addWidget(self.altitude_label, 0, 1)

        flight_layout.addWidget(QLabel("Ground Speed:"), 1, 0)
        self.speed_label = QLabel("0.0 m/s")
        flight_layout.addWidget(self.speed_label, 1, 1)

        layout.addWidget(flight_group)

        layout.addStretch()

    def update_from_message(self, message):
        """Update status from MAVLink message"""
        if not message:
            return

        msg_type = message.get_type()

        if msg_type == 'HEARTBEAT':
            self.last_heartbeat = time.time()
            self.armed = bool(message.base_mode & mavlink.MAV_MODE_FLAG_SAFETY_ARMED if mavlink else False)
            self.mode = self.get_mode_name(message.custom_mode) if hasattr(message, 'custom_mode') else "Unknown"
            self.update_status_display()

        elif msg_type == 'SYS_STATUS':
            if hasattr(message, 'voltage_battery'):
                self.battery_voltage = message.voltage_battery / 1000.0  # mV to V
            if hasattr(message, 'current_battery'):
                self.battery_current = message.current_battery / 100.0  # cA to A
            if hasattr(message, 'battery_remaining'):
                self.battery_remaining = message.battery_remaining
            self.update_battery_display()

        elif msg_type == 'GPS_RAW_INT':
            if hasattr(message, 'fix_type'):
                self.gps_fix = message.fix_type
            if hasattr(message, 'satellites_visible'):
                self.gps_satellites = message.satellites_visible
            if hasattr(message, 'alt'):
                self.altitude = message.alt / 1000.0  # mm to m
            self.update_gps_display()
            self.update_flight_display()

        elif msg_type == 'VFR_HUD':
            if hasattr(message, 'groundspeed'):
                self.ground_speed = message.groundspeed
            if hasattr(message, 'alt'):
                self.altitude = message.alt
            self.update_flight_display()

    def get_mode_name(self, custom_mode):
        """Get mode name from custom mode number"""
        # ArduCopter mode mapping
        modes = {
            0: 'STABILIZE', 1: 'ACRO', 2: 'ALT_HOLD', 3: 'AUTO', 4: 'GUIDED',
            5: 'LOITER', 6: 'RTL', 7: 'CIRCLE', 9: 'LAND', 11: 'DRIFT',
            13: 'SPORT', 14: 'FLIP', 15: 'AUTOTUNE', 16: 'POSHOLD',
            17: 'BRAKE', 18: 'THROW', 19: 'AVOID_ADSB', 20: 'GUIDED_NOGPS',
            21: 'SMART_RTL', 22: 'FLOWHOLD', 23: 'FOLLOW', 24: 'ZIGZAG'
        }
        return modes.get(custom_mode, f"UNKNOWN({custom_mode})")

    def update_status_display(self):
        """Update status display"""
        # Armed status
        if self.armed:
            self.armed_label.setText("ARMED")
            self.armed_label.setStyleSheet("color: red; font-weight: bold;")
        else:
            self.armed_label.setText("DISARMED")
            self.armed_label.setStyleSheet("color: green; font-weight: bold;")

        # Flight mode
        self.mode_label.setText(self.mode)

    def update_battery_display(self):
        """Update battery display"""
        self.voltage_label.setText(f"{self.battery_voltage:.1f} V")
        self.current_label.setText(f"{abs(self.battery_current):.1f} A")

        if self.battery_remaining >= 0:
            self.battery_progress.setValue(self.battery_remaining)

            # Color coding
            if self.battery_remaining > 50:
                color = "green"
            elif self.battery_remaining > 20:
                color = "orange"
            else:
                color = "red"

            self.battery_progress.setStyleSheet(f"""
                QProgressBar::chunk {{ background-color: {color}; }}
            """)

    def update_gps_display(self):
        """Update GPS display"""
        # GPS fix type
        fix_types = {
            0: "No GPS", 1: "No Fix", 2: "2D Fix", 3: "3D Fix",
            4: "DGPS", 5: "RTK Float", 6: "RTK Fixed"
        }

        fix_text = fix_types.get(self.gps_fix, "Unknown")
        self.gps_fix_label.setText(fix_text)

        # Color coding for GPS fix
        if self.gps_fix >= 3:
            self.gps_fix_label.setStyleSheet("color: green; font-weight: bold;")
        elif self.gps_fix >= 1:
            self.gps_fix_label.setStyleSheet("color: orange; font-weight: bold;")
        else:
            self.gps_fix_label.setStyleSheet("color: red; font-weight: bold;")

        # Satellites
        self.gps_sat_label.setText(str(self.gps_satellites))

    def update_flight_display(self):
        """Update flight data display"""
        self.altitude_label.setText(f"{self.altitude:.1f} m")
        self.speed_label.setText(f"{self.ground_speed:.1f} m/s")
