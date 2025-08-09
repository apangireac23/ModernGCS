
"""
Vehicle control widget with arm/disarm, takeoff, land, RTL buttons
"""

from PyQt6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
                            QPushButton, QLabel, QComboBox, QSpinBox, QDoubleSpinBox,
                            QGroupBox, QButtonGroup)
from PyQt6.QtCore import Qt, pyqtSignal
from PyQt6.QtGui import QFont


class VehicleControlWidget(QWidget):
    """Widget for controlling vehicle operations"""

    command_signal = pyqtSignal(str, dict)  # command, kwargs

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setEnabled(False)  # Disabled until connected
        self.init_ui()

    def init_ui(self):
        """Initialize UI"""
        layout = QVBoxLayout(self)

        # Title
        title = QLabel("Vehicle Control")
        title_font = QFont()
        title_font.setBold(True)
        title_font.setPointSize(12)
        title.setFont(title_font)
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(title)

        # Mode selection
        mode_group = QGroupBox("Flight Mode")
        mode_layout = QVBoxLayout(mode_group)

        self.mode_combo = QComboBox()
        modes = ['STABILIZE', 'ALT_HOLD', 'LOITER', 'AUTO', 'GUIDED', 'RTL', 'LAND']
        self.mode_combo.addItems(modes)
        mode_layout.addWidget(self.mode_combo)

        self.set_mode_button = QPushButton("Set Mode")
        self.set_mode_button.clicked.connect(self.set_mode)
        mode_layout.addWidget(self.set_mode_button)

        layout.addWidget(mode_group)

        # Arm/Disarm
        arm_group = QGroupBox("Arming")
        arm_layout = QHBoxLayout(arm_group)

        self.arm_button = QPushButton("ARM")
        self.arm_button.setStyleSheet("QPushButton { background-color: #4CAF50; color: white; font-weight: bold; }")
        self.arm_button.clicked.connect(self.arm)

        self.disarm_button = QPushButton("DISARM")
        self.disarm_button.setStyleSheet("QPushButton { background-color: #f44336; color: white; font-weight: bold; }")
        self.disarm_button.clicked.connect(self.disarm)

        arm_layout.addWidget(self.arm_button)
        arm_layout.addWidget(self.disarm_button)

        layout.addWidget(arm_group)

        # Flight operations
        flight_group = QGroupBox("Flight Operations")
        flight_layout = QGridLayout(flight_group)

        # Takeoff
        takeoff_label = QLabel("Takeoff Alt (m):")
        self.takeoff_altitude = QDoubleSpinBox()
        self.takeoff_altitude.setRange(1.0, 100.0)
        self.takeoff_altitude.setValue(10.0)
        self.takeoff_altitude.setSuffix(" m")

        self.takeoff_button = QPushButton("TAKEOFF")
        self.takeoff_button.setStyleSheet("QPushButton { background-color: #2196F3; color: white; font-weight: bold; }")
        self.takeoff_button.clicked.connect(self.takeoff)

        flight_layout.addWidget(takeoff_label, 0, 0)
        flight_layout.addWidget(self.takeoff_altitude, 0, 1)
        flight_layout.addWidget(self.takeoff_button, 0, 2)

        # Land
        self.land_button = QPushButton("LAND")
        self.land_button.setStyleSheet("QPushButton { background-color: #FF9800; color: white; font-weight: bold; }")
        self.land_button.clicked.connect(self.land)
        flight_layout.addWidget(self.land_button, 1, 0, 1, 3)

        # RTL
        self.rtl_button = QPushButton("RETURN TO LAUNCH")
        self.rtl_button.setStyleSheet("QPushButton { background-color: #9C27B0; color: white; font-weight: bold; }")
        self.rtl_button.clicked.connect(self.rtl)
        flight_layout.addWidget(self.rtl_button, 2, 0, 1, 3)

        layout.addWidget(flight_group)

        # Emergency stop
        emergency_group = QGroupBox("Emergency")
        emergency_layout = QVBoxLayout(emergency_group)

        self.emergency_stop_button = QPushButton("EMERGENCY STOP")
        self.emergency_stop_button.setStyleSheet("""
            QPushButton { 
                background-color: #d32f2f; 
                color: white; 
                font-weight: bold; 
                font-size: 14px;
                padding: 10px;
            }
            QPushButton:hover {
                background-color: #b71c1c;
            }
        """)
        self.emergency_stop_button.clicked.connect(self.emergency_stop)
        emergency_layout.addWidget(self.emergency_stop_button)

        layout.addWidget(emergency_group)

        layout.addStretch()

    def set_mode(self):
        """Set flight mode"""
        mode = self.mode_combo.currentText()
        self.command_signal.emit('set_mode', {'mode': mode})

    def arm(self):
        """Arm the vehicle"""
        self.command_signal.emit('arm', {})

    def disarm(self):
        """Disarm the vehicle"""
        self.command_signal.emit('disarm', {})

    def takeoff(self):
        """Takeoff to specified altitude"""
        altitude = self.takeoff_altitude.value()
        self.command_signal.emit('takeoff', {'altitude': altitude})

    def land(self):
        """Land at current position"""
        self.command_signal.emit('land', {})

    def rtl(self):
        """Return to launch"""
        self.command_signal.emit('rtl', {})

    def emergency_stop(self):
        """Emergency stop - disarm immediately"""
        # Force disarm regardless of current state
        self.command_signal.emit('disarm', {})
