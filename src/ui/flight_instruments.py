
"""
Flight instruments widget showing attitude, compass, etc.
"""

from PyQt6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
                            QLabel, QGroupBox, QFrame)
from PyQt6.QtCore import Qt, pyqtSignal, QTimer
from PyQt6.QtGui import QPainter, QPen, QBrush, QColor, QPolygonF, QFont
import math

try:
    from pymavlink.mavutil import mavlink
except ImportError:
    mavlink = None


class AttitudeIndicator(QWidget):
    """Attitude indicator (artificial horizon)"""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.roll = 0.0  # radians
        self.pitch = 0.0  # radians
        self.setMinimumSize(200, 200)

    def update_attitude(self, roll, pitch):
        """Update attitude values"""
        self.roll = roll
        self.pitch = pitch
        self.update()

    def paintEvent(self, event):
        """Paint the attitude indicator"""
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        width = self.width()
        height = self.height()
        size = min(width, height)

        # Center coordinates
        cx = width / 2
        cy = height / 2
        radius = size / 2 - 10

        # Save original transform
        painter.save()

        # Move to center and rotate
        painter.translate(cx, cy)
        painter.rotate(math.degrees(self.roll))

        # Draw sky (blue)
        sky_rect = QPolygonF([
            [-radius*2, -radius*2],
            [radius*2, -radius*2],
            [radius*2, 0],
            [-radius*2, 0]
        ])
        painter.fillPath(sky_rect, QColor(135, 206, 235))

        # Draw ground (brown)
        ground_rect = QPolygonF([
            [-radius*2, 0],
            [radius*2, 0], 
            [radius*2, radius*2],
            [-radius*2, radius*2]
        ])
        painter.fillPath(ground_rect, QColor(139, 69, 19))

        # Draw horizon line
        painter.setPen(QPen(QColor.white, 3))
        painter.drawLine(-radius*2, 0, radius*2, 0)

        # Draw pitch lines
        painter.setPen(QPen(QColor.white, 1))
        pitch_scale = radius / (math.pi/4)  # Scale factor

        for angle in range(-60, 61, 10):
            if angle == 0:
                continue
            y = -angle * pitch_scale / 10
            if abs(y) < radius:
                if angle % 20 == 0:
                    # Major pitch lines
                    painter.drawLine(-radius/2, y, radius/2, y)
                    painter.drawText(radius/2 + 5, y + 5, f"{angle}°")
                    painter.drawText(-radius/2 - 25, y + 5, f"{angle}°")
                else:
                    # Minor pitch lines
                    painter.drawLine(-radius/4, y, radius/4, y)

        painter.restore()

        # Draw aircraft symbol (fixed to display)
        painter.setPen(QPen(QColor.yellow, 3))
        painter.drawLine(cx - 30, cy, cx - 10, cy)
        painter.drawLine(cx + 10, cy, cx + 30, cy)
        painter.drawLine(cx, cy - 5, cx, cy + 5)

        # Draw roll scale
        painter.setPen(QPen(QColor.white, 2))
        for angle in range(-60, 61, 10):
            rad = math.radians(angle)
            x1 = cx + (radius - 5) * math.sin(rad)
            y1 = cy - (radius - 5) * math.cos(rad)
            x2 = cx + radius * math.sin(rad)
            y2 = cy - radius * math.cos(rad)
            painter.drawLine(x1, y1, x2, y2)

            if angle % 30 == 0:
                x_text = cx + (radius + 15) * math.sin(rad)
                y_text = cy - (radius + 15) * math.cos(rad)
                painter.drawText(x_text - 10, y_text + 5, f"{abs(angle)}")

        # Draw roll indicator
        painter.save()
        painter.translate(cx, cy)
        painter.rotate(math.degrees(self.roll))
        painter.setPen(QPen(QColor.red, 3))
        painter.drawLine(0, -radius + 5, 0, -radius + 15)
        painter.restore()


class CompassRose(QWidget):
    """Compass rose widget"""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.heading = 0.0  # degrees
        self.setMinimumSize(150, 150)

    def update_heading(self, heading_degrees):
        """Update heading value"""
        self.heading = heading_degrees % 360
        self.update()

    def paintEvent(self, event):
        """Paint the compass"""
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        width = self.width()
        height = self.height()
        size = min(width, height)

        cx = width / 2
        cy = height / 2
        radius = size / 2 - 10

        # Draw outer circle
        painter.setPen(QPen(QColor.black, 2))
        painter.setBrush(QBrush(QColor.white))
        painter.drawEllipse(cx - radius, cy - radius, radius * 2, radius * 2)

        painter.save()
        painter.translate(cx, cy)
        painter.rotate(-self.heading)

        # Draw compass marks
        painter.setPen(QPen(QColor.black, 1))
        font = painter.font()
        font.setPointSize(10)
        painter.setFont(font)

        directions = ['N', 'NE', 'E', 'SE', 'S', 'SW', 'W', 'NW']

        for i in range(8):
            angle = i * 45
            rad = math.radians(angle)

            # Direction marks
            x1 = (radius - 20) * math.sin(rad)
            y1 = -(radius - 20) * math.cos(rad)
            x2 = radius * math.sin(rad)
            y2 = -radius * math.cos(rad)

            painter.setPen(QPen(QColor.black, 2))
            painter.drawLine(x1, y1, x2, y2)

            # Direction labels
            x_text = (radius - 35) * math.sin(rad)
            y_text = -(radius - 35) * math.cos(rad)
            painter.drawText(x_text - 10, y_text + 5, directions[i])

        # Draw degree marks
        painter.setPen(QPen(QColor.gray, 1))
        for deg in range(0, 360, 10):
            if deg % 45 != 0:  # Skip main directions
                rad = math.radians(deg)
                x1 = (radius - 10) * math.sin(rad)
                y1 = -(radius - 10) * math.cos(rad)
                x2 = radius * math.sin(rad)
                y2 = -radius * math.cos(rad)
                painter.drawLine(x1, y1, x2, y2)

        painter.restore()

        # Draw heading indicator
        painter.setPen(QPen(QColor.red, 3))
        painter.drawLine(cx, cy - radius + 5, cx, cy - radius + 20)

        # Draw heading text
        painter.setPen(QPen(QColor.black, 2))
        font.setPointSize(14)
        font.setBold(True)
        painter.setFont(font)
        heading_text = f"{int(self.heading):03d}°"
        text_width = painter.fontMetrics().horizontalAdvance(heading_text)
        painter.drawText(cx - text_width/2, cy + radius - 5, heading_text)


class FlightInstrumentsWidget(QWidget):
    """Widget containing flight instruments"""

    def __init__(self, parent=None):
        super().__init__(parent)

        # Data storage
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        self.init_ui()

        # Update timer
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_instruments)
        self.update_timer.start(100)  # 10 Hz

    def init_ui(self):
        """Initialize UI"""
        layout = QVBoxLayout(self)

        # Title
        title = QLabel("Flight Instruments")
        title_font = QFont()
        title_font.setBold(True)
        title_font.setPointSize(12)
        title.setFont(title_font)
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(title)

        # Instruments layout
        instruments_layout = QHBoxLayout()

        # Attitude indicator
        attitude_group = QGroupBox("Attitude")
        attitude_layout = QVBoxLayout(attitude_group)

        self.attitude_indicator = AttitudeIndicator()
        attitude_layout.addWidget(self.attitude_indicator)

        # Attitude values
        attitude_values_layout = QGridLayout()
        attitude_values_layout.addWidget(QLabel("Roll:"), 0, 0)
        self.roll_label = QLabel("0.0°")
        attitude_values_layout.addWidget(self.roll_label, 0, 1)

        attitude_values_layout.addWidget(QLabel("Pitch:"), 1, 0)
        self.pitch_label = QLabel("0.0°")
        attitude_values_layout.addWidget(self.pitch_label, 1, 1)

        attitude_layout.addLayout(attitude_values_layout)
        instruments_layout.addWidget(attitude_group)

        # Compass
        compass_group = QGroupBox("Compass")
        compass_layout = QVBoxLayout(compass_group)

        self.compass = CompassRose()
        compass_layout.addWidget(self.compass)

        # Heading value
        heading_layout = QHBoxLayout()
        heading_layout.addWidget(QLabel("Heading:"))
        self.heading_label = QLabel("0°")
        heading_layout.addWidget(self.heading_label)
        heading_layout.addStretch()
        compass_layout.addLayout(heading_layout)

        instruments_layout.addWidget(compass_group)

        layout.addLayout(instruments_layout)

    def update_from_message(self, message):
        """Update instruments from MAVLink message"""
        if not message:
            return

        msg_type = message.get_type()

        if msg_type == 'ATTITUDE':
            if hasattr(message, 'roll'):
                self.roll = message.roll
            if hasattr(message, 'pitch'):
                self.pitch = message.pitch
            if hasattr(message, 'yaw'):
                self.yaw = message.yaw

    def update_instruments(self):
        """Update instrument displays"""
        # Update attitude indicator
        self.attitude_indicator.update_attitude(self.roll, self.pitch)

        # Update compass
        heading_deg = math.degrees(self.yaw) % 360
        self.compass.update_heading(heading_deg)

        # Update text labels
        self.roll_label.setText(f"{math.degrees(self.roll):.1f}°")
        self.pitch_label.setText(f"{math.degrees(self.pitch):.1f}°")
        self.heading_label.setText(f"{heading_deg:.0f}°")
