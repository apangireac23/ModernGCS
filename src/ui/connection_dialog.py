
"""
Connection dialog for MAVLink connections
"""

from PyQt6.QtWidgets import (QDialog, QVBoxLayout, QHBoxLayout, QFormLayout,
                            QComboBox, QLineEdit, QSpinBox, QPushButton, 
                            QLabel, QGroupBox, QTabWidget, QWidget)
from PyQt6.QtCore import Qt
import serial.tools.list_ports


class ConnectionDialog(QDialog):
    """Dialog for configuring MAVLink connections"""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Connect to Vehicle")
        self.setModal(True)
        self.resize(400, 300)

        self.init_ui()

    def init_ui(self):
        """Initialize UI"""
        layout = QVBoxLayout(self)

        # Tab widget for different connection types
        self.tab_widget = QTabWidget()
        layout.addWidget(self.tab_widget)

        # Serial tab
        self.serial_tab = self.create_serial_tab()
        self.tab_widget.addTab(self.serial_tab, "Serial")

        # UDP tab
        self.udp_tab = self.create_udp_tab()
        self.tab_widget.addTab(self.udp_tab, "UDP")

        # TCP tab
        self.tcp_tab = self.create_tcp_tab()
        self.tab_widget.addTab(self.tcp_tab, "TCP")

        # Buttons
        button_layout = QHBoxLayout()

        self.connect_button = QPushButton("Connect")
        self.connect_button.clicked.connect(self.accept)
        self.connect_button.setDefault(True)

        self.cancel_button = QPushButton("Cancel")
        self.cancel_button.clicked.connect(self.reject)

        button_layout.addStretch()
        button_layout.addWidget(self.connect_button)
        button_layout.addWidget(self.cancel_button)

        layout.addLayout(button_layout)

    def create_serial_tab(self):
        """Create serial connection tab"""
        widget = QWidget()
        layout = QFormLayout(widget)

        # Port selection
        self.serial_port_combo = QComboBox()
        self.refresh_serial_ports()
        layout.addRow("Port:", self.serial_port_combo)

        # Refresh button
        refresh_button = QPushButton("Refresh")
        refresh_button.clicked.connect(self.refresh_serial_ports)
        port_layout = QHBoxLayout()
        port_layout.addWidget(self.serial_port_combo)
        port_layout.addWidget(refresh_button)
        layout.addRow("Port:", port_layout)

        # Baud rate
        self.serial_baud_combo = QComboBox()
        baud_rates = [9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600]
        for rate in baud_rates:
            self.serial_baud_combo.addItem(str(rate), rate)
        self.serial_baud_combo.setCurrentText("115200")
        layout.addRow("Baud Rate:", self.serial_baud_combo)

        return widget

    def create_udp_tab(self):
        """Create UDP connection tab"""
        widget = QWidget()
        layout = QFormLayout(widget)

        # Host
        self.udp_host_edit = QLineEdit("127.0.0.1")
        layout.addRow("Host:", self.udp_host_edit)

        # Port
        self.udp_port_spin = QSpinBox()
        self.udp_port_spin.setRange(1, 65535)
        self.udp_port_spin.setValue(14550)
        layout.addRow("Port:", self.udp_port_spin)

        # Local port
        self.udp_local_port_spin = QSpinBox()
        self.udp_local_port_spin.setRange(1, 65535)
        self.udp_local_port_spin.setValue(14551)
        layout.addRow("Local Port:", self.udp_local_port_spin)

        return widget

    def create_tcp_tab(self):
        """Create TCP connection tab"""
        widget = QWidget()
        layout = QFormLayout(widget)

        # Host
        self.tcp_host_edit = QLineEdit("127.0.0.1")
        layout.addRow("Host:", self.tcp_host_edit)

        # Port
        self.tcp_port_spin = QSpinBox()
        self.tcp_port_spin.setRange(1, 65535)
        self.tcp_port_spin.setValue(5760)
        layout.addRow("Port:", self.tcp_port_spin)

        return widget

    def refresh_serial_ports(self):
        """Refresh available serial ports"""
        self.serial_port_combo.clear()

        ports = serial.tools.list_ports.comports()
        for port in sorted(ports):
            self.serial_port_combo.addItem(f"{port.device} - {port.description}", port.device)

        if self.serial_port_combo.count() == 0:
            self.serial_port_combo.addItem("No ports available", None)

    def get_connection_info(self):
        """Get connection information based on selected tab"""
        current_tab = self.tab_widget.currentIndex()

        if current_tab == 0:  # Serial
            return {
                'type': 'serial',
                'port': self.serial_port_combo.currentData(),
                'baudrate': self.serial_baud_combo.currentData()
            }
        elif current_tab == 1:  # UDP
            return {
                'type': 'udp',
                'host': self.udp_host_edit.text(),
                'port': self.udp_port_spin.value(),
                'local_port': self.udp_local_port_spin.value()
            }
        elif current_tab == 2:  # TCP
            return {
                'type': 'tcp',
                'host': self.tcp_host_edit.text(),
                'port': self.tcp_port_spin.value()
            }

        return {}
