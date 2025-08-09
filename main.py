
"""
ModernGCS - A modern Ground Control Station for ArduPilot
"""

import sys
import os
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                            QHBoxLayout, QSplitter, QMenuBar, QStatusBar, 
                            QMessageBox, QTabWidget)
from PyQt6.QtCore import Qt, QTimer, pyqtSignal
from PyQt6.QtGui import QAction, QIcon

# Add src to path for imports
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from communication.mavlink_handler import MAVLinkHandler
from ui.connection_dialog import ConnectionDialog
from ui.map_widget import MapWidget
from ui.vehicle_control import VehicleControlWidget
from ui.status_widget import StatusWidget
from ui.flight_instruments import FlightInstrumentsWidget
from utils.settings import Settings


class ModernGCS(QMainWindow):
    """Main application window"""

    def __init__(self):
        super().__init__()

        # Initialize components
        self.mavlink_handler = None
        self.settings = Settings()

        # Initialize UI
        self.init_ui()

        # Setup connections
        self.setup_connections()

        # Status update timer
        self.status_timer = QTimer()
        self.status_timer.timeout.connect(self.update_status)
        self.status_timer.start(1000)  # Update every second

    def init_ui(self):
        """Initialize user interface"""
        self.setWindowTitle("ModernGCS - Ground Control Station")
        self.setGeometry(100, 100, 1400, 900)

        # Create menu bar
        self.create_menu_bar()

        # Create status bar
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        self.status_bar.showMessage("Disconnected", 0)

        # Create central widget
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        # Main layout
        main_layout = QHBoxLayout(central_widget)

        # Create splitter for resizable panels
        main_splitter = QSplitter(Qt.Orientation.Horizontal)
        main_layout.addWidget(main_splitter)

        # Left panel - controls and status
        left_widget = QWidget()
        left_layout = QVBoxLayout(left_widget)
        left_widget.setMaximumWidth(350)
        left_widget.setMinimumWidth(300)

        # Vehicle controls
        self.vehicle_control = VehicleControlWidget()
        left_layout.addWidget(self.vehicle_control)

        # Status widget
        self.status_widget = StatusWidget()
        left_layout.addWidget(self.status_widget)

        left_layout.addStretch()
        main_splitter.addWidget(left_widget)

        # Center panel - map
        self.map_widget = MapWidget()
        main_splitter.addWidget(self.map_widget)

        # Right panel - instruments
        right_widget = QWidget()
        right_layout = QVBoxLayout(right_widget)
        right_widget.setMaximumWidth(350)
        right_widget.setMinimumWidth(300)

        # Flight instruments
        self.flight_instruments = FlightInstrumentsWidget()
        right_layout.addWidget(self.flight_instruments)

        right_layout.addStretch()
        main_splitter.addWidget(right_widget)

        # Set splitter proportions
        main_splitter.setStretchFactor(0, 0)  # Left panel - fixed
        main_splitter.setStretchFactor(1, 1)  # Center panel - expandable
        main_splitter.setStretchFactor(2, 0)  # Right panel - fixed

    def create_menu_bar(self):
        """Create menu bar"""
        menubar = self.menuBar()

        # File menu
        file_menu = menubar.addMenu('&File')

        # Connect action
        connect_action = QAction('&Connect...', self)
        connect_action.setShortcut('Ctrl+C')
        connect_action.triggered.connect(self.show_connection_dialog)
        file_menu.addAction(connect_action)

        # Disconnect action
        self.disconnect_action = QAction('&Disconnect', self)
        self.disconnect_action.setShortcut('Ctrl+D')
        self.disconnect_action.triggered.connect(self.disconnect)
        self.disconnect_action.setEnabled(False)
        file_menu.addAction(self.disconnect_action)

        file_menu.addSeparator()

        # Exit action
        exit_action = QAction('E&xit', self)
        exit_action.setShortcut('Ctrl+Q')
        exit_action.triggered.connect(self.close)
        file_menu.addAction(exit_action)

        # View menu
        view_menu = menubar.addMenu('&View')

        # Refresh map action
        refresh_map_action = QAction('&Refresh Map', self)
        refresh_map_action.setShortcut('F5')
        refresh_map_action.triggered.connect(self.map_widget.refresh)
        view_menu.addAction(refresh_map_action)

        # Help menu
        help_menu = menubar.addMenu('&Help')

        # About action
        about_action = QAction('&About', self)
        about_action.triggered.connect(self.show_about)
        help_menu.addAction(about_action)

    def setup_connections(self):
        """Setup signal connections"""
        # Vehicle control signals
        self.vehicle_control.command_signal.connect(self.handle_vehicle_command)

        # Map waypoint signals
        self.map_widget.waypoint_signal.connect(self.handle_waypoint_command)

    def show_connection_dialog(self):
        """Show connection dialog"""
        dialog = ConnectionDialog(self)

        if dialog.exec() == ConnectionDialog.DialogCode.Accepted:
            connection_config = dialog.get_connection_config()
            self.connect_to_vehicle(connection_config)

    def connect_to_vehicle(self, config):
        """Connect to vehicle with given configuration"""
        try:
            # Create MAVLink handler
            self.mavlink_handler = MAVLinkHandler(config)

            # Connect signals
            self.mavlink_handler.message_received.connect(self.handle_mavlink_message)
            self.mavlink_handler.connection_lost.connect(self.handle_connection_lost)

            # Start connection
            self.mavlink_handler.start()

            # Update UI
            self.status_bar.showMessage(f"Connecting to {config['type']}...", 3000)
            self.disconnect_action.setEnabled(True)
            self.vehicle_control.setEnabled(True)

        except Exception as e:
            QMessageBox.critical(self, "Connection Error", f"Failed to connect: {str(e)}")

    def disconnect(self):
        """Disconnect from vehicle"""
        if self.mavlink_handler:
            self.mavlink_handler.stop()
            self.mavlink_handler = None

        # Update UI
        self.status_bar.showMessage("Disconnected", 0)
        self.disconnect_action.setEnabled(False)
        self.vehicle_control.setEnabled(False)

    def handle_mavlink_message(self, message):
        """Handle received MAVLink message"""
        # Update status widget
        self.status_widget.update_from_message(message)

        # Update flight instruments
        self.flight_instruments.update_from_message(message)

        # Update map widget
        self.map_widget.update_from_message(message)

    def handle_vehicle_command(self, command, kwargs):
        """Handle vehicle command from control widget"""
        if not self.mavlink_handler:
            QMessageBox.warning(self, "Not Connected", "Please connect to a vehicle first.")
            return

        try:
            if command == 'arm':
                self.mavlink_handler.arm_vehicle()
            elif command == 'disarm':
                self.mavlink_handler.disarm_vehicle()
            elif command == 'takeoff':
                altitude = kwargs.get('altitude', 10.0)
                self.mavlink_handler.takeoff(altitude)
            elif command == 'land':
                self.mavlink_handler.land()
            elif command == 'rtl':
                self.mavlink_handler.return_to_launch()
            elif command == 'set_mode':
                mode = kwargs.get('mode', 'STABILIZE')
                self.mavlink_handler.set_mode(mode)

            self.status_bar.showMessage(f"Command sent: {command}", 2000)

        except Exception as e:
            QMessageBox.critical(self, "Command Error", f"Failed to send command: {str(e)}")

    def handle_waypoint_command(self, command, waypoints):
        """Handle waypoint command from map"""
        if not self.mavlink_handler:
            QMessageBox.warning(self, "Not Connected", "Please connect to a vehicle first.")
            return

        try:
            if command == 'upload':
                self.mavlink_handler.upload_mission(waypoints)
                self.status_bar.showMessage("Mission uploaded", 2000)
            elif command == 'download':
                mission = self.mavlink_handler.download_mission()
                self.map_widget.load_mission(mission)
                self.status_bar.showMessage("Mission downloaded", 2000)
            elif command == 'goto':
                if waypoints:
                    waypoint = waypoints[0]
                    self.mavlink_handler.goto_waypoint(waypoint['lat'], waypoint['lon'], waypoint.get('alt', 10))
                    self.status_bar.showMessage("Going to waypoint", 2000)

        except Exception as e:
            QMessageBox.critical(self, "Waypoint Error", f"Failed to handle waypoints: {str(e)}")

    def handle_connection_lost(self):
        """Handle connection lost"""
        QMessageBox.warning(self, "Connection Lost", "Connection to vehicle has been lost.")
        self.disconnect()

    def update_status(self):
        """Update status bar"""
        if self.mavlink_handler and self.mavlink_handler.is_connected():
            self.status_bar.showMessage(f"Connected - Last message: {self.mavlink_handler.get_last_message_time():.1f}s ago")

    def show_about(self):
        """Show about dialog"""
        QMessageBox.about(self, "About ModernGCS",
                         """ModernGCS v1.0

A modern Ground Control Station for ArduPilot vehicles.

Features:
- Multiple connection types (Serial, TCP, UDP)
- Real-time flight data display
- Interactive mapping with waypoint management
- Flight instruments
- Vehicle control

Built with Python and PyQt6.""")

    def closeEvent(self, event):
        """Handle close event"""
        if self.mavlink_handler:
            self.disconnect()
        event.accept()


def main():
    """Main entry point"""
    app = QApplication(sys.argv)
    app.setApplicationName("ModernGCS")
    app.setApplicationVersion("1.0")

    # Create and show main window
    window = ModernGCS()
    window.show()

    # Run application
    sys.exit(app.exec())


if __name__ == '__main__':
    main()
