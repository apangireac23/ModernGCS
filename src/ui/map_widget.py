
"""
Mapping functionality with ESRI tile support and waypoint management
"""

import math
import requests
from typing import List, Tuple, Optional, Dict, Any
from PyQt6.QtCore import QObject, pyqtSignal, QThread, QTimer, QPointF, QRectF, Qt
from PyQt6.QtGui import QPixmap, QPainter, QPen, QBrush, QColor, QFont, QPolygonF
from PyQt6.QtWidgets import QWidget, QVBoxLayout, QLabel
import numpy as np

class Waypoint:
    """Represents a waypoint with GPS coordinates and metadata"""

    def __init__(self, lat: float, lon: float, alt: float = 0, wp_type: str = "WAYPOINT"):
        self.lat = lat
        self.lon = lon
        self.alt = alt
        self.type = wp_type
        self.id = 0
        self.param1 = 0
        self.param2 = 0
        self.param3 = 0
        self.param4 = 0

    def __str__(self):
        return f"WP{self.id}: ({self.lat:.6f}, {self.lon:.6f}, {self.alt:.1f}m) [{self.type}]"

class MapTileLoader(QThread):
    """Background thread for loading map tiles"""

    tile_loaded = pyqtSignal(int, int, int, QPixmap)  # zoom, x, y, pixmap
    loading_error = pyqtSignal(str)

    def __init__(self, tile_server_url: str):
        super().__init__()
        self.tile_server_url = tile_server_url
        self.tile_queue = []
        self.is_running = False

    def add_tile_request(self, zoom: int, x: int, y: int):
        """Add a tile to the loading queue"""
        self.tile_queue.append((zoom, x, y))

    def run(self):
        """Main thread execution"""
        self.is_running = True
        while self.is_running and self.tile_queue:
            try:
                zoom, x, y = self.tile_queue.pop(0)
                self._load_tile(zoom, x, y)
            except IndexError:
                break

    def stop(self):
        """Stop the tile loading thread"""
        self.is_running = False
        self.quit()
        self.wait()

    def _load_tile(self, zoom: int, x: int, y: int):
        """Load a single tile"""
        try:
            # Format the URL with tile coordinates
            url = self.tile_server_url.format(z=zoom, x=x, y=y)

            # Download the tile
            response = requests.get(url, timeout=10)
            response.raise_for_status()

            # Create QPixmap from the image data
            pixmap = QPixmap()
            pixmap.loadFromData(response.content)

            if not pixmap.isNull():
                self.tile_loaded.emit(zoom, x, y, pixmap)
            else:
                self.loading_error.emit(f"Invalid image data for tile {zoom}/{x}/{y}")

        except Exception as e:
            self.loading_error.emit(f"Failed to load tile {zoom}/{x}/{y}: {str(e)}")

class MapWidget(QWidget):
    """
    Interactive map widget with ESRI tile support
    """

    # Signals
    waypoint_added = pyqtSignal(object)  # Waypoint
    waypoint_moved = pyqtSignal(int, object)  # index, Waypoint
    waypoint_removed = pyqtSignal(int)  # index
    map_clicked = pyqtSignal(float, float)  # lat, lon

    def __init__(self, parent=None):
        super().__init__(parent)

        # Map settings
        self.tile_size = 256
        self.zoom_level = 15
        self.center_lat = 37.7749  # San Francisco
        self.center_lon = -122.4194
        self.tile_server = "https://services.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}"

        # Waypoints
        self.waypoints: List[Waypoint] = []
        self.waypoint_radius = 8
        self.selected_waypoint = -1
        self.dragging_waypoint = False

        # Vehicle position
        self.vehicle_lat = 0
        self.vehicle_lon = 0
        self.vehicle_heading = 0
        self.show_vehicle = False

        # Tile management
        self.tiles: Dict[Tuple[int, int, int], QPixmap] = {}  # (zoom, x, y) -> QPixmap
        self.tile_loader = MapTileLoader(self.tile_server)
        self.tile_loader.tile_loaded.connect(self._on_tile_loaded)
        self.tile_loader.loading_error.connect(self._on_tile_error)

        # UI setup
        self.setMinimumSize(400, 300)
        self.setMouseTracking(True)

        # Load initial tiles
        self._load_visible_tiles()

    def set_tile_server(self, url: str):
        """Set the tile server URL"""
        self.tile_server = url
        self.tile_loader.tile_server_url = url
        self.tiles.clear()
        self._load_visible_tiles()
        self.update()

    def set_center(self, lat: float, lon: float):
        """Set the map center"""
        self.center_lat = lat
        self.center_lon = lon
        self._load_visible_tiles()
        self.update()

    def set_zoom(self, zoom: int):
        """Set the zoom level"""
        zoom = max(1, min(20, zoom))  # Clamp zoom level
        if zoom != self.zoom_level:
            self.zoom_level = zoom
            self.tiles.clear()  # Clear tiles for new zoom level
            self._load_visible_tiles()
            self.update()

    def add_waypoint(self, lat: float, lon: float, alt: float = 10):
        """Add a waypoint"""
        wp = Waypoint(lat, lon, alt)
        wp.id = len(self.waypoints) + 1
        self.waypoints.append(wp)
        self.waypoint_added.emit(wp)
        self.update()
        return wp

    def remove_waypoint(self, index: int):
        """Remove a waypoint by index"""
        if 0 <= index < len(self.waypoints):
            del self.waypoints[index]
            # Renumber waypoints
            for i, wp in enumerate(self.waypoints):
                wp.id = i + 1
            self.waypoint_removed.emit(index)
            self.update()

    def clear_waypoints(self):
        """Clear all waypoints"""
        self.waypoints.clear()
        self.selected_waypoint = -1
        self.update()

    def update_vehicle_position(self, lat: float, lon: float, heading: float = 0):
        """Update vehicle position on map"""
        self.vehicle_lat = lat
        self.vehicle_lon = lon
        self.vehicle_heading = heading
        self.show_vehicle = True
        self.update()

    def hide_vehicle(self):
        """Hide vehicle from map"""
        self.show_vehicle = False
        self.update()

    def _load_visible_tiles(self):
        """Load tiles visible in the current view"""
        # Calculate tile bounds for current view
        width = self.width()
        height = self.height()

        # Get tile coordinates for current center
        center_tile_x, center_tile_y = self._lat_lon_to_tile(self.center_lat, self.center_lon, self.zoom_level)

        # Calculate how many tiles we need in each direction
        tiles_x = int(width / self.tile_size) + 2
        tiles_y = int(height / self.tile_size) + 2

        # Load tiles around the center
        for dx in range(-tiles_x//2, tiles_x//2 + 1):
            for dy in range(-tiles_y//2, tiles_y//2 + 1):
                tile_x = center_tile_x + dx
                tile_y = center_tile_y + dy

                # Check if tile is valid
                max_tile = 2 ** self.zoom_level
                if 0 <= tile_x < max_tile and 0 <= tile_y < max_tile:
                    tile_key = (self.zoom_level, tile_x, tile_y)
                    if tile_key not in self.tiles:
                        self.tile_loader.add_tile_request(self.zoom_level, tile_x, tile_y)

        # Start loading if not already running
        if not self.tile_loader.isRunning():
            self.tile_loader.start()

    def _lat_lon_to_tile(self, lat: float, lon: float, zoom: int) -> Tuple[int, int]:
        """Convert lat/lon to tile coordinates"""
        lat_rad = math.radians(lat)
        n = 2.0 ** zoom
        x = int((lon + 180.0) / 360.0 * n)
        y = int((1.0 - math.asinh(math.tan(lat_rad)) / math.pi) / 2.0 * n)
        return x, y

    def _tile_to_lat_lon(self, x: int, y: int, zoom: int) -> Tuple[float, float]:
        """Convert tile coordinates to lat/lon"""
        n = 2.0 ** zoom
        lon = x / n * 360.0 - 180.0
        lat_rad = math.atan(math.sinh(math.pi * (1 - 2 * y / n)))
        lat = math.degrees(lat_rad)
        return lat, lon

    def _lat_lon_to_pixel(self, lat: float, lon: float) -> QPointF:
        """Convert lat/lon to pixel coordinates in widget"""
        # Get tile coordinates
        tile_x, tile_y = self._lat_lon_to_tile(lat, lon, self.zoom_level)
        center_tile_x, center_tile_y = self._lat_lon_to_tile(self.center_lat, self.center_lon, self.zoom_level)

        # Calculate pixel offset within tile
        n = 2.0 ** self.zoom_level
        pixel_x = ((lon + 180.0) / 360.0 * n - tile_x) * self.tile_size
        lat_rad = math.radians(lat)
        pixel_y = ((1.0 - math.asinh(math.tan(lat_rad)) / math.pi) / 2.0 * n - tile_y) * self.tile_size

        # Convert to widget coordinates
        widget_x = self.width() / 2 + (tile_x - center_tile_x) * self.tile_size + pixel_x
        widget_y = self.height() / 2 + (tile_y - center_tile_y) * self.tile_size + pixel_y

        return QPointF(widget_x, widget_y)

    def _pixel_to_lat_lon(self, pixel: QPointF) -> Tuple[float, float]:
        """Convert pixel coordinates to lat/lon"""
        # Calculate offset from center
        center_pixel_x = self.width() / 2
        center_pixel_y = self.height() / 2

        dx = pixel.x() - center_pixel_x
        dy = pixel.y() - center_pixel_y

        # Convert to tile coordinates
        center_tile_x, center_tile_y = self._lat_lon_to_tile(self.center_lat, self.center_lon, self.zoom_level)

        tile_x = center_tile_x + dx / self.tile_size
        tile_y = center_tile_y + dy / self.tile_size

        # Convert to lat/lon
        n = 2.0 ** self.zoom_level
        lon = tile_x / n * 360.0 - 180.0
        lat_rad = math.atan(math.sinh(math.pi * (1 - 2 * tile_y / n)))
        lat = math.degrees(lat_rad)

        return lat, lon

    def _on_tile_loaded(self, zoom: int, x: int, y: int, pixmap: QPixmap):
        """Handle loaded tile"""
        if zoom == self.zoom_level:  # Only accept tiles for current zoom level
            self.tiles[(zoom, x, y)] = pixmap
            self.update()

    def _on_tile_error(self, error: str):
        """Handle tile loading error"""
        print(f"Tile loading error: {error}")

    def paintEvent(self, event):
        """Paint the map and overlays"""
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        # Fill background
        painter.fillRect(self.rect(), QColor(200, 200, 200))

        # Draw tiles
        self._draw_tiles(painter)

        # Draw waypoints
        self._draw_waypoints(painter)

        # Draw vehicle
        if self.show_vehicle:
            self._draw_vehicle(painter)

        # Draw waypoint lines
        self._draw_waypoint_lines(painter)

    def _draw_tiles(self, painter: QPainter):
        """Draw map tiles"""
        center_tile_x, center_tile_y = self._lat_lon_to_tile(self.center_lat, self.center_lon, self.zoom_level)

        # Calculate tile bounds
        tiles_x = int(self.width() / self.tile_size) + 2
        tiles_y = int(self.height() / self.tile_size) + 2

        for dx in range(-tiles_x//2, tiles_x//2 + 1):
            for dy in range(-tiles_y//2, tiles_y//2 + 1):
                tile_x = center_tile_x + dx
                tile_y = center_tile_y + dy

                tile_key = (self.zoom_level, tile_x, tile_y)
                if tile_key in self.tiles:
                    pixmap = self.tiles[tile_key]

                    # Calculate tile position
                    pos_x = self.width() / 2 + dx * self.tile_size - self.tile_size / 2
                    pos_y = self.height() / 2 + dy * self.tile_size - self.tile_size / 2

                    painter.drawPixmap(int(pos_x), int(pos_y), pixmap)

    def _draw_waypoints(self, painter: QPainter):
        """Draw waypoints on the map"""
        for i, wp in enumerate(self.waypoints):
            pixel_pos = self._lat_lon_to_pixel(wp.lat, wp.lon)

            # Choose colors
            if i == self.selected_waypoint:
                fill_color = QColor(255, 255, 0, 200)  # Yellow for selected
                border_color = QColor(255, 255, 0)
            else:
                fill_color = QColor(0, 255, 0, 150)  # Green for normal
                border_color = QColor(0, 200, 0)

            # Draw waypoint circle
            painter.setBrush(QBrush(fill_color))
            painter.setPen(QPen(border_color, 2))
            painter.drawEllipse(pixel_pos, self.waypoint_radius, self.waypoint_radius)

            # Draw waypoint number
            painter.setPen(QPen(QColor(0, 0, 0)))
            painter.setFont(QFont("Arial", 8, QFont.Weight.Bold))
            painter.drawText(pixel_pos.x() - 4, pixel_pos.y() + 4, str(wp.id))

    def _draw_vehicle(self, painter: QPainter):
        """Draw vehicle position and heading"""
        if not self.show_vehicle:
            return

        vehicle_pos = self._lat_lon_to_pixel(self.vehicle_lat, self.vehicle_lon)

        # Draw vehicle as triangle pointing in heading direction
        painter.setBrush(QBrush(QColor(255, 0, 0, 200)))
        painter.setPen(QPen(QColor(200, 0, 0), 2))

        # Create triangle points (relative to vehicle position)
        size = 12
        triangle = QPolygonF([
            QPointF(0, -size),      # Nose
            QPointF(-size/2, size), # Left wing
            QPointF(size/2, size)   # Right wing
        ])

        # Rotate triangle to match heading
        painter.save()
        painter.translate(vehicle_pos)
        painter.rotate(self.vehicle_heading)
        painter.drawPolygon(triangle)
        painter.restore()

    def _draw_waypoint_lines(self, painter: QPainter):
        """Draw lines connecting waypoints"""
        if len(self.waypoints) < 2:
            return

        painter.setPen(QPen(QColor(0, 0, 255, 150), 2, Qt.PenStyle.DashLine))

        for i in range(len(self.waypoints) - 1):
            start_pos = self._lat_lon_to_pixel(self.waypoints[i].lat, self.waypoints[i].lon)
            end_pos = self._lat_lon_to_pixel(self.waypoints[i + 1].lat, self.waypoints[i + 1].lon)
            painter.drawLine(start_pos, end_pos)

    def mousePressEvent(self, event):
        """Handle mouse press events"""
        if event.button() == Qt.MouseButton.LeftButton:
            # Check if clicking on a waypoint
            clicked_wp = self._get_waypoint_at_position(event.position())

            if clicked_wp >= 0:
                self.selected_waypoint = clicked_wp
                self.dragging_waypoint = True
            else:
                # Add new waypoint
                lat, lon = self._pixel_to_lat_lon(event.position())
                self.add_waypoint(lat, lon)
                self.map_clicked.emit(lat, lon)

        elif event.button() == Qt.MouseButton.RightButton:
            # Remove waypoint or show context menu
            clicked_wp = self._get_waypoint_at_position(event.position())
            if clicked_wp >= 0:
                self.remove_waypoint(clicked_wp)

        self.update()

    def mouseMoveEvent(self, event):
        """Handle mouse move events"""
        if self.dragging_waypoint and self.selected_waypoint >= 0:
            # Move selected waypoint
            lat, lon = self._pixel_to_lat_lon(event.position())
            wp = self.waypoints[self.selected_waypoint]
            wp.lat = lat
            wp.lon = lon
            self.waypoint_moved.emit(self.selected_waypoint, wp)
            self.update()

    def mouseReleaseEvent(self, event):
        """Handle mouse release events"""
        if event.button() == Qt.MouseButton.LeftButton:
            self.dragging_waypoint = False

    def wheelEvent(self, event):
        """Handle mouse wheel events for zooming"""
        zoom_delta = 1 if event.angleDelta().y() > 0 else -1
        self.set_zoom(self.zoom_level + zoom_delta)

    def _get_waypoint_at_position(self, pos: QPointF) -> int:
        """Get waypoint index at given position, or -1 if none"""
        for i, wp in enumerate(self.waypoints):
            wp_pos = self._lat_lon_to_pixel(wp.lat, wp.lon)
            distance = ((pos.x() - wp_pos.x()) ** 2 + (pos.y() - wp_pos.y()) ** 2) ** 0.5
            if distance <= self.waypoint_radius:
                return i
        return -1

    def resizeEvent(self, event):
        """Handle resize events"""
        super().resizeEvent(event)
        self._load_visible_tiles()
