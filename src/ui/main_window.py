
"""
Main window for ArduPilot Ground Control Station
"""

import sys
import os
from typing import Optional
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
    QSplitter, QTabWidget, QLabel, QPushButton, QComboBox, QSpinBox,
    QLineEdit, QGroupBox, QFormLayout, QMessageBox, QStatusBar,
    QProgressBar, QMenuBar, QMenu, QFileDialog, QCheckBox
)
from PyQt6.QtCore import Qt, QTimer, pyqtSignal, QSettings
from PyQt6.QtGui import QAction, QIcon, QFont

# Import our modules
try:
    from communications.mavlink_connection import MAVLinkConnection, ConnectionType, ConnectionState
    from flight.waypoint_manager import WaypointManager
    from gui.map_widget import MapWidget
    from gui.connection_dialog import ConnectionDialog
    from gui.flight_instruments import FlightInstrumentsWidget
    from gui.vehicle_status import VehicleStatusWidget
    from gui.waypoint_editor import WaypointEditorWidget
except ImportError as e:
    print(f"Import error: {e}")


class MainWindow(QMainWindow):
    """Main application window"""

    def __init__(self):
        super().__init__()

        # Application settings
        self.settings = QSettings("ArduPilotGCS", "ArduPilotGCS")

        # Core components
        self.mavlink_connection = MAVLinkConnection()
        self.waypoint_manager = WaypointManager(self.mavlink_connection)

        # Vehicle state
        self.vehicle_armed = False
        self.vehicle_mode = "UNKNOWN"
        self.connection_status = ConnectionState.DISCONNECTED

        # GUI components
        self.central_widget = None
        self.map_widget = None
        self.flight_instruments = None
        self.vehicle_status = None
        self.waypoint_editor = None
        self.status_bar = None
        self.connection_dialog = None

        # Timers
        self.status_timer = QTimer()
        self.status_timer.timeout.connect(self.update_status)
        self.status_timer.start(1000)  # Update every second

        self.init_ui()
        self.connect_signals()
        self.restore_settings()

    def init_ui(self):
        """Initialize the user interface"""
        self.setWindowTitle("ArduPilot Ground Control Station")
        self.setMinimumSize(1200, 800)

        # Create menus
        self.create_menus()

        # Create central widget with splitter layout
        self.central_widget = QSplitter(Qt.Orientation.Horizontal)
        self.setCentralWidget(self.central_widget)

        # Left panel - Flight instruments and vehicle status
        left_panel = self.create_left_panel()
        self.central_widget.addWidget(left_panel)

        # Center panel - Map
        try:
            self.map_widget = MapWidget()
            self.central_widget.addWidget(self.map_widget)
        except Exception as e:
            print(f"Failed to create map widget: {e}")
            # Create placeholder
            placeholder = QLabel("Map not available")
            placeholder.setAlignment(Qt.AlignmentFlag.AlignCenter)
            self.central_widget.addWidget(placeholder)

        # Right panel - Waypoint editor
        try:
            self.waypoint_editor = WaypointEditorWidget(self.waypoint_manager)
            self.central_widget.addWidget(self.waypoint_editor)
        except Exception as e:
            print(f"Failed to create waypoint editor: {e}")
            # Create placeholder  
            placeholder = QLabel("Waypoint editor not available")
            placeholder.setAlignment(Qt.AlignmentFlag.AlignCenter)
            self.central_widget.addWidget(placeholder)

        # Set splitter proportions (20%, 60%, 20%)
        self.central_widget.setSizes([300, 900, 300])

        # Status bar
        self.create_status_bar()

        # Connection dialog
        self.connection_dialog = ConnectionDialog()

    def create_left_panel(self) -> QWidget:
        """Create left panel with flight instruments and status"""
        panel = QWidget()
        layout = QVBoxLayout(panel)

        # Flight instruments
        try:
            self.flight_instruments = FlightInstrumentsWidget()
            layout.addWidget(self.flight_instruments)
        except Exception as e:
            print(f"Failed to create flight instruments: {e}")
            placeholder = QLabel("Flight Instruments\nNot Available")
            placeholder.setAlignment(Qt.AlignmentFlag.AlignCenter)
            placeholder.setMinimumHeight(300)
            layout.addWidget(placeholder)

        # Vehicle status
        try:
            self.vehicle_status = VehicleStatusWidget()
            layout.addWidget(self.vehicle_status)
        except Exception as e:
            print(f"Failed to create vehicle status: {e}")
            placeholder = QLabel("Vehicle Status\nNot Available")  
            placeholder.setAlignment(Qt.AlignmentFlag.AlignCenter)
            placeholder.setMinimumHeight(200)
            layout.addWidget(placeholder)

        # Flight controls
        flight_controls = self.create_flight_controls()
        layout.addWidget(flight_controls)

        return panel

    def create_flight_controls(self) -> QWidget:
        """Create flight control buttons"""
        group = QGroupBox("Flight Controls")
        layout = QVBoxLayout(group)

        # Connection controls
        connection_layout = QHBoxLayout()

        self.connect_btn = QPushButton("Connect")
        self.connect_btn.clicked.connect(self.show_connection_dialog)
        connection_layout.addWidget(self.connect_btn)

        self.disconnect_btn = QPushButton("Disconnect")
        self.disconnect_btn.clicked.connect(self.disconnect)
        self.disconnect_btn.setEnabled(False)
        connection_layout.addWidget(self.disconnect_btn)

        layout.addLayout(connection_layout)

        # Mode selection
        mode_layout = QHBoxLayout()
        mode_layout.addWidget(QLabel("Mode:"))

        self.mode_combo = QComboBox()
        self.mode_combo.addItems([
            "STABILIZE", "ACRO", "ALT_HOLD", "AUTO", "GUIDED",
            "LOITER", "RTL", "CIRCLE", "LAND", "DRIFT", "SPORT",
            "FLIP", "AUTOTUNE", "POSHOLD", "BRAKE", "THROW",
            "AVOID_ADSB", "GUIDED_NOGPS", "SMART_RTL", "FLOWHOLD",
            "FOLLOW", "ZIGZAG", "SYSTEMID", "AUTOROTATE"
        ])
        self.mode_combo.currentTextChanged.connect(self.set_flight_mode)
        self.mode_combo.setEnabled(False)
        mode_layout.addWidget(self.mode_combo)

        layout.addLayout(mode_layout)

        # Arm/Disarm
        arm_layout = QHBoxLayout()

        self.arm_btn = QPushButton("ARM")
        self.arm_btn.clicked.connect(self.arm_vehicle)
        self.arm_btn.setEnabled(False)
        self.arm_btn.setStyleSheet("QPushButton { background-color: #4CAF50; }")
        arm_layout.addWidget(self.arm_btn)

        self.disarm_btn = QPushButton("DISARM")
        self.disarm_btn.clicked.connect(self.disarm_vehicle)
        self.disarm_btn.setEnabled(False)
        self.disarm_btn.setStyleSheet("QPushButton { background-color: #f44336; }")
        arm_layout.addWidget(self.disarm_btn)

        layout.addLayout(arm_layout)

        # Flight commands
        self.takeoff_btn = QPushButton("TAKEOFF")
        self.takeoff_btn.clicked.connect(self.takeoff)
        self.takeoff_btn.setEnabled(False)
        layout.addWidget(self.takeoff_btn)

        # Takeoff altitude
        alt_layout = QHBoxLayout()
        alt_layout.addWidget(QLabel("Alt (m):"))
        self.takeoff_altitude = QSpinBox()
        self.takeoff_altitude.setRange(5, 200)
        self.takeoff_altitude.setValue(20)
        alt_layout.addWidget(self.takeoff_altitude)
        layout.addLayout(alt_layout)

        self.land_btn = QPushButton("LAND")
        self.land_btn.clicked.connect(self.land)
        self.land_btn.setEnabled(False)
        layout.addWidget(self.land_btn)

        self.rtl_btn = QPushButton("RETURN TO LAUNCH")
        self.rtl_btn.clicked.connect(self.return_to_launch)
        self.rtl_btn.setEnabled(False)
        layout.addWidget(self.rtl_btn)

        return group

    def create_menus(self):
        """Create application menus"""
        menubar = self.menuBar()

        # File menu
        file_menu = menubar.addMenu("&File")

        new_mission_action = QAction("&New Mission", self)
        new_mission_action.setShortcut("Ctrl+N")
        new_mission_action.triggered.connect(self.new_mission)
        file_menu.addAction(new_mission_action)

        open_mission_action = QAction("&Open Mission", self)
        open_mission_action.setShortcut("Ctrl+O")
        open_mission_action.triggered.connect(self.open_mission)
        file_menu.addAction(open_mission_action)

        save_mission_action = QAction("&Save Mission", self)
        save_mission_action.setShortcut("Ctrl+S")
        save_mission_action.triggered.connect(self.save_mission)
        file_menu.addAction(save_mission_action)

        save_as_action = QAction("Save Mission &As...", self)
        save_as_action.setShortcut("Ctrl+Shift+S")
        save_as_action.triggered.connect(self.save_mission_as)
        file_menu.addAction(save_as_action)

        file_menu.addSeparator()

        exit_action = QAction("E&xit", self)
        exit_action.setShortcut("Ctrl+Q")
        exit_action.triggered.connect(self.close)
        file_menu.addAction(exit_action)

        # Vehicle menu
        vehicle_menu = menubar.addMenu("&Vehicle")

        connect_action = QAction("&Connect", self)
        connect_action.triggered.connect(self.show_connection_dialog)
        vehicle_menu.addAction(connect_action)

        disconnect_action = QAction("&Disconnect", self)
        disconnect_action.triggered.connect(self.disconnect)
        vehicle_menu.addAction(disconnect_action)

        vehicle_menu.addSeparator()

        download_mission_action = QAction("&Download Mission", self)
        download_mission_action.triggered.connect(self.download_mission)
        vehicle_menu.addAction(download_mission_action)

        upload_mission_action = QAction("&Upload Mission", self)
        upload_mission_action.triggered.connect(self.upload_mission)
        vehicle_menu.addAction(upload_mission_action)

        # View menu
        view_menu = menubar.addMenu("&View")

        fullscreen_action = QAction("&Full Screen", self)
        fullscreen_action.setShortcut("F11")
        fullscreen_action.triggered.connect(self.toggle_fullscreen)
        view_menu.addAction(fullscreen_action)

        # Help menu
        help_menu = menubar.addMenu("&Help")

        about_action = QAction("&About", self)
        about_action.triggered.connect(self.show_about)
        help_menu.addAction(about_action)

    def create_status_bar(self):
        """Create status bar"""
        self.status_bar = self.statusBar()

        # Connection status
        self.connection_status_label = QLabel("Disconnected")
        self.status_bar.addWidget(self.connection_status_label)

        # Message count
        self.message_count_label = QLabel("Messages: 0")
        self.status_bar.addPermanentWidget(self.message_count_label)

        # Progress bar for uploads/downloads
        self.progress_bar = QProgressBar()
        self.progress_bar.setVisible(False)
        self.status_bar.addPermanentWidget(self.progress_bar)

    def connect_signals(self):
        """Connect signals between components"""
        # MAVLink connection signals
        self.mavlink_connection.connection_state_changed.connect(self.on_connection_state_changed)
        self.mavlink_connection.status_message.connect(self.show_status_message)
        self.mavlink_connection.message_received.connect(self.on_message_received)
        self.mavlink_connection.connection_lost.connect(self.on_connection_lost)

        # Waypoint manager signals
        self.waypoint_manager.mission_uploaded.connect(self.on_mission_uploaded)
        self.waypoint_manager.mission_downloaded.connect(self.on_mission_downloaded)
        self.waypoint_manager.mission_progress.connect(self.on_mission_progress)

        # Map signals (if available)
        if self.map_widget:
            try:
                self.map_widget.waypoint_added.connect(self.waypoint_manager.add_waypoint)
                self.waypoint_manager.waypoint_added.connect(self.map_widget.add_waypoint)
                self.waypoint_manager.waypoint_removed.connect(self.map_widget.remove_waypoint)
                self.waypoint_manager.waypoint_updated.connect(self.map_widget.update_waypoint)
            except AttributeError:
                pass  # Signals may not be implemented yet

    # Connection methods

    def show_connection_dialog(self):
        """Show connection dialog"""
        if self.connection_dialog.exec() == 1:  # Accepted
            connection_params = self.connection_dialog.get_connection_params()
            self.connect_to_vehicle(connection_params)

    def connect_to_vehicle(self, params: dict):
        """Connect to vehicle with specified parameters"""
        connection_type = params['type']

        if connection_type == ConnectionType.SERIAL:
            success = self.mavlink_connection.connect_serial(
                params['port'], params['baudrate']
            )
        elif connection_type == ConnectionType.UDP:
            success = self.mavlink_connection.connect_udp(
                params['host'], params['port'], params.get('local_port', 14551)
            )
        elif connection_type == ConnectionType.TCP:
            success = self.mavlink_connection.connect_tcp(
                params['host'], params['port']
            )
        else:
            success = False

        if not success:
            QMessageBox.warning(self, "Connection Failed", 
                              "Failed to connect to vehicle")

    def disconnect(self):
        """Disconnect from vehicle"""
        self.mavlink_connection.disconnect()

    # Flight control methods

    def arm_vehicle(self):
        """Arm the vehicle"""
        if self.mavlink_connection.arm_disarm(True):
            self.show_status_message("Arming vehicle...")
        else:
            QMessageBox.warning(self, "Error", "Failed to send arm command")

    def disarm_vehicle(self):
        """Disarm the vehicle"""
        if self.mavlink_connection.arm_disarm(False):
            self.show_status_message("Disarming vehicle...")
        else:
            QMessageBox.warning(self, "Error", "Failed to send disarm command")

    def takeoff(self):
        """Takeoff to specified altitude"""
        altitude = self.takeoff_altitude.value()
        if self.mavlink_connection.takeoff(altitude):
            self.show_status_message(f"Taking off to {altitude}m...")
        else:
            QMessageBox.warning(self, "Error", "Failed to send takeoff command")

    def land(self):
        """Land at current position"""
        if self.mavlink_connection.land():
            self.show_status_message("Landing...")
        else:
            QMessageBox.warning(self, "Error", "Failed to send land command")

    def return_to_launch(self):
        """Return to launch point"""
        if self.mavlink_connection.return_to_launch():
            self.show_status_message("Returning to launch...")
        else:
            QMessageBox.warning(self, "Error", "Failed to send RTL command")

    def set_flight_mode(self, mode: str):
        """Set flight mode"""
        if self.mavlink_connection.set_mode(mode):
            self.show_status_message(f"Setting mode to {mode}...")
        else:
            QMessageBox.warning(self, "Error", f"Failed to set mode to {mode}")

    # Mission methods

    def new_mission(self):
        """Create new mission"""
        if self.waypoint_manager.get_waypoint_count() > 0:
            reply = QMessageBox.question(self, "New Mission", 
                                       "Current mission will be lost. Continue?")
            if reply != QMessageBox.StandardButton.Yes:
                return

        self.waypoint_manager.new_mission()
        self.show_status_message("New mission created")

    def open_mission(self):
        """Open mission from file"""
        filename, _ = QFileDialog.getOpenFileName(
            self, "Open Mission", "", "Mission Files (*.json);;All Files (*)"
        )

        if filename:
            if self.waypoint_manager.load_mission(filename):
                self.show_status_message(f"Mission loaded from {filename}")
            else:
                QMessageBox.warning(self, "Error", "Failed to load mission")

    def save_mission(self):
        """Save current mission"""
        # TODO: Implement save to last known file
        self.save_mission_as()

    def save_mission_as(self):
        """Save mission as new file"""
        filename, _ = QFileDialog.getSaveFileName(
            self, "Save Mission", "", "Mission Files (*.json);;All Files (*)"
        )

        if filename:
            if not filename.endswith('.json'):
                filename += '.json'

            if self.waypoint_manager.save_mission(filename):
                self.show_status_message(f"Mission saved to {filename}")
            else:
                QMessageBox.warning(self, "Error", "Failed to save mission")

    def upload_mission(self):
        """Upload mission to vehicle"""
        if not self.mavlink_connection.is_connected():
            QMessageBox.warning(self, "Error", "Not connected to vehicle")
            return

        count = self.waypoint_manager.get_waypoint_count()
        if count == 0:
            QMessageBox.information(self, "Info", "No waypoints to upload")
            return

        reply = QMessageBox.question(self, "Upload Mission", 
                                   f"Upload {count} waypoints to vehicle?")
        if reply == QMessageBox.StandardButton.Yes:
            self.waypoint_manager.upload_mission()

    def download_mission(self):
        """Download mission from vehicle"""
        if not self.mavlink_connection.is_connected():
            QMessageBox.warning(self, "Error", "Not connected to vehicle")
            return

        if self.waypoint_manager.get_waypoint_count() > 0:
            reply = QMessageBox.question(self, "Download Mission", 
                                       "Current mission will be replaced. Continue?")
            if reply != QMessageBox.StandardButton.Yes:
                return

        self.waypoint_manager.download_mission()

    # Event handlers

    def on_connection_state_changed(self, state: ConnectionState):
        """Handle connection state changes"""
        self.connection_status = state

        if state == ConnectionState.CONNECTED:
            self.connection_status_label.setText("Connected")
            self.connect_btn.setEnabled(False)
            self.disconnect_btn.setEnabled(True)
            self.enable_flight_controls(True)

        elif state == ConnectionState.DISCONNECTED:
            self.connection_status_label.setText("Disconnected")
            self.connect_btn.setEnabled(True)
            self.disconnect_btn.setEnabled(False)
            self.enable_flight_controls(False)

        elif state == ConnectionState.CONNECTING:
            self.connection_status_label.setText("Connecting...")

        elif state == ConnectionState.ERROR:
            self.connection_status_label.setText("Connection Error")

    def on_connection_lost(self):
        """Handle connection lost"""
        QMessageBox.warning(self, "Connection Lost", 
                          "Connection to vehicle lost")

    def on_message_received(self, msg):
        """Handle received MAVLink message"""
        # Update message count
        info = self.mavlink_connection.get_connection_info()
        self.message_count_label.setText(f"Messages: {info['messages_received']}")

        # Update vehicle state based on message type
        msg_type = msg.get_type()

        if msg_type == 'HEARTBEAT':
            self.update_vehicle_state(msg)
        elif msg_type == 'SYS_STATUS':
            self.update_system_status(msg)
        elif msg_type == 'GLOBAL_POSITION_INT':
            self.update_position(msg)
        elif msg_type == 'ATTITUDE':
            self.update_attitude(msg)

        # Forward to instruments
        if self.flight_instruments:
            try:
                self.flight_instruments.update_from_message(msg)
            except AttributeError:
                pass

        if self.vehicle_status:
            try:
                self.vehicle_status.update_from_message(msg)
            except AttributeError:
                pass

        if self.map_widget:
            try:
                self.map_widget.update_from_message(msg)
            except AttributeError:
                pass

    def update_vehicle_state(self, msg):
        """Update vehicle state from HEARTBEAT message"""
        # Update armed state
        armed = bool(msg.base_mode & 128)  # MAV_MODE_FLAG_SAFETY_ARMED
        if armed != self.vehicle_armed:
            self.vehicle_armed = armed
            self.update_arm_buttons()

        # Update mode
        # Note: This is simplified - ArduPilot uses custom modes
        self.vehicle_mode = f"Mode {msg.custom_mode}"

    def update_system_status(self, msg):
        """Update from SYS_STATUS message"""
        # Battery voltage, current, etc.
        pass

    def update_position(self, msg):
        """Update from GLOBAL_POSITION_INT message"""
        # Lat, lon, alt
        if self.map_widget:
            try:
                lat = msg.lat / 1e7
                lon = msg.lon / 1e7
                alt = msg.alt / 1000.0
                self.map_widget.update_vehicle_position(lat, lon, alt)
            except AttributeError:
                pass

    def update_attitude(self, msg):
        """Update from ATTITUDE message"""
        # Roll, pitch, yaw
        pass

    def update_arm_buttons(self):
        """Update arm/disarm button states"""
        if self.vehicle_armed:
            self.arm_btn.setEnabled(False)
            self.disarm_btn.setEnabled(True)
        else:
            self.arm_btn.setEnabled(True)
            self.disarm_btn.setEnabled(False)

    def enable_flight_controls(self, enabled: bool):
        """Enable/disable flight control buttons"""
        self.mode_combo.setEnabled(enabled)
        if not enabled:
            self.arm_btn.setEnabled(False)
            self.disarm_btn.setEnabled(False)
        else:
            self.update_arm_buttons()

        self.takeoff_btn.setEnabled(enabled)
        self.land_btn.setEnabled(enabled)
        self.rtl_btn.setEnabled(enabled)

    def on_mission_uploaded(self, success: bool):
        """Handle mission upload completion"""
        self.progress_bar.setVisible(False)
        if success:
            self.show_status_message("Mission uploaded successfully")
        else:
            self.show_status_message("Mission upload failed")
            QMessageBox.warning(self, "Upload Failed", "Mission upload failed")

    def on_mission_downloaded(self, success: bool):
        """Handle mission download completion"""
        self.progress_bar.setVisible(False)
        if success:
            count = self.waypoint_manager.get_waypoint_count()
            self.show_status_message(f"Mission downloaded: {count} waypoints")
        else:
            self.show_status_message("Mission download failed")
            QMessageBox.warning(self, "Download Failed", "Mission download failed")

    def on_mission_progress(self, current: int, total: int):
        """Handle mission upload/download progress"""
        self.progress_bar.setVisible(True)
        self.progress_bar.setMaximum(total)
        self.progress_bar.setValue(current)

    def show_status_message(self, message: str):
        """Show status message"""
        self.status_bar.showMessage(message, 3000)

    def update_status(self):
        """Update status periodically"""
        if self.mavlink_connection.is_connected():
            info = self.mavlink_connection.get_connection_info()
            self.message_count_label.setText(f"Messages: {info['messages_received']}")

    def toggle_fullscreen(self):
        """Toggle fullscreen mode"""
        if self.isFullScreen():
            self.showNormal()
        else:
            self.showFullScreen()

    def show_about(self):
        """Show about dialog"""
        QMessageBox.about(self, "About ArduPilot GCS", 
                         "ArduPilot Ground Control Station\n\n"
                         "A comprehensive ground control station for ArduPilot vehicles\n"
                         "Built with Python and PyQt6")

    def restore_settings(self):
        """Restore application settings"""
        # Window geometry
        geometry = self.settings.value("geometry")
        if geometry:
            self.restoreGeometry(geometry)

        # Window state
        state = self.settings.value("windowState")
        if state:
            self.restoreState(state)

        # Splitter sizes
        splitter_sizes = self.settings.value("splitterSizes")
        if splitter_sizes:
            try:
                sizes = [int(s) for s in splitter_sizes]
                self.central_widget.setSizes(sizes)
            except:
                pass

    def save_settings(self):
        """Save application settings"""
        self.settings.setValue("geometry", self.saveGeometry())
        self.settings.setValue("windowState", self.saveState())
        self.settings.setValue("splitterSizes", 
                             [str(s) for s in self.central_widget.sizes()])

    def closeEvent(self, event):
        """Handle application close event"""
        # Disconnect if connected
        if self.mavlink_connection.is_connected():
            self.mavlink_connection.disconnect()

        # Save settings
        self.save_settings()

        event.accept()


def main():
    """Main entry point"""
    app = QApplication(sys.argv)

    # Set application properties
    app.setApplicationName("ArduPilot GCS")
    app.setApplicationVersion("1.0.0")
    app.setOrganizationName("ArduPilot")
    app.setOrganizationDomain("ardupilot.org")

    # Create and show main window
    window = MainWindow()
    window.show()

    # Start event loop
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
