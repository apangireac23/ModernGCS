
"""
Waypoint and mission management for ArduPilot
"""

import json
import math
from typing import List, Optional, Dict, Any, Tuple
from dataclasses import dataclass, asdict
from PyQt6.QtCore import QObject, pyqtSignal
from pymavlink.dialects.v20 import ardupilotmega as mavlink


@dataclass
class Waypoint:
    """Represents a single waypoint"""
    sequence: int
    latitude: float
    longitude: float
    altitude: float
    command: int = mavlink.MAV_CMD_NAV_WAYPOINT
    param1: float = 0.0
    param2: float = 0.0
    param3: float = 0.0
    param4: float = 0.0
    frame: int = mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
    current: bool = False
    autocontinue: bool = True

    def to_mavlink_message(self, target_system: int, target_component: int) -> object:
        """Convert waypoint to MAVLink MISSION_ITEM_INT message"""
        return mavlink.MAVLink_mission_item_int_message(
            target_system=target_system,
            target_component=target_component,
            seq=self.sequence,
            frame=self.frame,
            command=self.command,
            current=1 if self.current else 0,
            autocontinue=1 if self.autocontinue else 0,
            param1=self.param1,
            param2=self.param2,
            param3=self.param3,
            param4=self.param4,
            x=int(self.latitude * 1e7),   # lat in 1e7 degrees
            y=int(self.longitude * 1e7),  # lon in 1e7 degrees
            z=self.altitude,
            mission_type=mavlink.MAV_MISSION_TYPE_MISSION
        )

    @classmethod
    def from_mavlink_message(cls, msg) -> 'Waypoint':
        """Create waypoint from MAVLink MISSION_ITEM_INT message"""
        return cls(
            sequence=msg.seq,
            latitude=msg.x / 1e7,
            longitude=msg.y / 1e7,
            altitude=msg.z,
            command=msg.command,
            param1=msg.param1,
            param2=msg.param2,
            param3=msg.param3,
            param4=msg.param4,
            frame=msg.frame,
            current=bool(msg.current),
            autocontinue=bool(msg.autocontinue)
        )

    def distance_to(self, other: 'Waypoint') -> float:
        """Calculate distance to another waypoint in meters"""
        return self.distance_between_coords(
            self.latitude, self.longitude,
            other.latitude, other.longitude
        )

    @staticmethod
    def distance_between_coords(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        """Calculate distance between two coordinates in meters"""
        # Haversine formula
        R = 6371000  # Earth's radius in meters

        lat1_rad = math.radians(lat1)
        lat2_rad = math.radians(lat2)
        delta_lat = math.radians(lat2 - lat1)
        delta_lon = math.radians(lon2 - lon1)

        a = (math.sin(delta_lat/2) * math.sin(delta_lat/2) + 
             math.cos(lat1_rad) * math.cos(lat2_rad) * 
             math.sin(delta_lon/2) * math.sin(delta_lon/2))
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))

        return R * c

    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary"""
        return asdict(self)

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'Waypoint':
        """Create from dictionary"""
        return cls(**data)


class Mission:
    """Represents a complete mission"""

    def __init__(self, name: str = "Untitled Mission"):
        self.name = name
        self.waypoints: List[Waypoint] = []
        self.home_position: Optional[Waypoint] = None
        self.created_time = None
        self.modified_time = None

    def add_waypoint(self, waypoint: Waypoint):
        """Add waypoint to mission"""
        waypoint.sequence = len(self.waypoints)
        self.waypoints.append(waypoint)

    def insert_waypoint(self, index: int, waypoint: Waypoint):
        """Insert waypoint at specific index"""
        waypoint.sequence = index
        self.waypoints.insert(index, waypoint)
        self._renumber_waypoints()

    def remove_waypoint(self, index: int) -> bool:
        """Remove waypoint at index"""
        if 0 <= index < len(self.waypoints):
            del self.waypoints[index]
            self._renumber_waypoints()
            return True
        return False

    def move_waypoint(self, from_index: int, to_index: int) -> bool:
        """Move waypoint from one position to another"""
        if (0 <= from_index < len(self.waypoints) and 
            0 <= to_index < len(self.waypoints)):
            waypoint = self.waypoints.pop(from_index)
            self.waypoints.insert(to_index, waypoint)
            self._renumber_waypoints()
            return True
        return False

    def update_waypoint(self, index: int, waypoint: Waypoint) -> bool:
        """Update waypoint at index"""
        if 0 <= index < len(self.waypoints):
            waypoint.sequence = index
            self.waypoints[index] = waypoint
            return True
        return False

    def get_waypoint(self, index: int) -> Optional[Waypoint]:
        """Get waypoint at index"""
        if 0 <= index < len(self.waypoints):
            return self.waypoints[index]
        return None

    def clear(self):
        """Clear all waypoints"""
        self.waypoints.clear()

    def get_total_distance(self) -> float:
        """Calculate total mission distance in meters"""
        if len(self.waypoints) < 2:
            return 0.0

        total = 0.0
        for i in range(1, len(self.waypoints)):
            total += self.waypoints[i-1].distance_to(self.waypoints[i])
        return total

    def get_bounds(self) -> Optional[Dict[str, float]]:
        """Get mission bounding box"""
        if not self.waypoints:
            return None

        lats = [wp.latitude for wp in self.waypoints]
        lons = [wp.longitude for wp in self.waypoints]

        return {
            'min_lat': min(lats),
            'max_lat': max(lats),
            'min_lon': min(lons),
            'max_lon': max(lons)
        }

    def _renumber_waypoints(self):
        """Renumber all waypoints sequentially"""
        for i, waypoint in enumerate(self.waypoints):
            waypoint.sequence = i

    def to_dict(self) -> Dict[str, Any]:
        """Convert mission to dictionary"""
        return {
            'name': self.name,
            'waypoints': [wp.to_dict() for wp in self.waypoints],
            'home_position': self.home_position.to_dict() if self.home_position else None,
            'created_time': self.created_time,
            'modified_time': self.modified_time
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'Mission':
        """Create mission from dictionary"""
        mission = cls(data.get('name', 'Untitled Mission'))
        mission.waypoints = [Waypoint.from_dict(wp_data) for wp_data in data.get('waypoints', [])]
        if data.get('home_position'):
            mission.home_position = Waypoint.from_dict(data['home_position'])
        mission.created_time = data.get('created_time')
        mission.modified_time = data.get('modified_time')
        return mission

    def save_to_file(self, filepath: str):
        """Save mission to JSON file"""
        import time
        self.modified_time = time.time()
        if not self.created_time:
            self.created_time = self.modified_time

        with open(filepath, 'w') as f:
            json.dump(self.to_dict(), f, indent=2)

    @classmethod
    def load_from_file(cls, filepath: str) -> 'Mission':
        """Load mission from JSON file"""
        with open(filepath, 'r') as f:
            data = json.load(f)
        return cls.from_dict(data)


class WaypointManager(QObject):
    """Manages waypoints and missions for the GCS"""

    # Signals
    mission_changed = pyqtSignal()
    waypoint_added = pyqtSignal(int)      # index
    waypoint_removed = pyqtSignal(int)    # index
    waypoint_updated = pyqtSignal(int)    # index
    mission_uploaded = pyqtSignal(bool)   # success
    mission_downloaded = pyqtSignal(bool) # success
    mission_progress = pyqtSignal(int, int) # current, total

    def __init__(self, connection=None):
        super().__init__()

        self.connection = connection
        self.current_mission = Mission()

        # Upload/download state
        self._upload_in_progress = False
        self._download_in_progress = False
        self._upload_waypoints = []
        self._upload_index = 0
        self._expected_count = 0
        self._received_waypoints = []

        # Connect to MAVLink messages if connection provided
        if self.connection:
            self.connection.message_received.connect(self._handle_mavlink_message)

        # Message handlers
        self._message_handlers = {
            'MISSION_REQUEST_INT': self._handle_mission_request,
            'MISSION_ACK': self._handle_mission_ack,
            'MISSION_COUNT': self._handle_mission_count,
            'MISSION_ITEM_INT': self._handle_mission_item,
            'MISSION_CURRENT': self._handle_mission_current,
            'MISSION_ITEM_REACHED': self._handle_mission_item_reached
        }

    def set_connection(self, connection):
        """Set MAVLink connection"""
        if self.connection:
            self.connection.message_received.disconnect(self._handle_mavlink_message)

        self.connection = connection
        if self.connection:
            self.connection.message_received.connect(self._handle_mavlink_message)

    def _handle_mavlink_message(self, msg):
        """Handle incoming MAVLink messages"""
        msg_type = msg.get_type()
        if msg_type in self._message_handlers:
            self._message_handlers[msg_type](msg)

    # Mission management

    def new_mission(self, name: str = "New Mission"):
        """Create new mission"""
        self.current_mission = Mission(name)
        self.mission_changed.emit()

    def load_mission(self, filepath: str) -> bool:
        """Load mission from file"""
        try:
            self.current_mission = Mission.load_from_file(filepath)
            self.mission_changed.emit()
            return True
        except Exception as e:
            print(f"Failed to load mission: {e}")
            return False

    def save_mission(self, filepath: str) -> bool:
        """Save mission to file"""
        try:
            self.current_mission.save_to_file(filepath)
            return True
        except Exception as e:
            print(f"Failed to save mission: {e}")
            return False

    # Waypoint operations

    def add_waypoint(self, lat: float, lon: float, alt: float = 20.0,
                    command: int = mavlink.MAV_CMD_NAV_WAYPOINT) -> int:
        """Add waypoint to current mission"""
        waypoint = Waypoint(
            sequence=len(self.current_mission.waypoints),
            latitude=lat,
            longitude=lon,
            altitude=alt,
            command=command
        )

        self.current_mission.add_waypoint(waypoint)
        self.waypoint_added.emit(waypoint.sequence)
        self.mission_changed.emit()
        return waypoint.sequence

    def remove_waypoint(self, index: int) -> bool:
        """Remove waypoint from mission"""
        if self.current_mission.remove_waypoint(index):
            self.waypoint_removed.emit(index)
            self.mission_changed.emit()
            return True
        return False

    def update_waypoint(self, index: int, lat: float, lon: float, 
                       alt: float = None) -> bool:
        """Update waypoint position"""
        waypoint = self.current_mission.get_waypoint(index)
        if waypoint:
            waypoint.latitude = lat
            waypoint.longitude = lon
            if alt is not None:
                waypoint.altitude = alt
            self.waypoint_updated.emit(index)
            self.mission_changed.emit()
            return True
        return False

    def get_waypoint(self, index: int) -> Optional[Waypoint]:
        """Get waypoint at index"""
        return self.current_mission.get_waypoint(index)

    def get_waypoints(self) -> List[Waypoint]:
        """Get all waypoints"""
        return self.current_mission.waypoints.copy()

    def get_waypoint_count(self) -> int:
        """Get number of waypoints"""
        return len(self.current_mission.waypoints)

    def clear_mission(self):
        """Clear all waypoints"""
        self.current_mission.clear()
        self.mission_changed.emit()

    # MAVLink mission upload/download

    def upload_mission(self) -> bool:
        """Upload current mission to vehicle"""
        if not self.connection or not self.connection.is_connected():
            print("No connection available")
            return False

        if self._upload_in_progress:
            print("Upload already in progress")
            return False

        try:
            self._upload_in_progress = True
            self._upload_waypoints = self.current_mission.waypoints.copy()
            self._upload_index = 0

            # Send mission count
            count = len(self._upload_waypoints)
            msg = self.connection.mavlink_connection.mav.mission_count_encode(
                self.connection.target_system_id,
                self.connection.target_component_id,
                count,
                mavlink.MAV_MISSION_TYPE_MISSION
            )

            success = self.connection.send_message(msg)
            if not success:
                self._upload_in_progress = False

            return success

        except Exception as e:
            print(f"Failed to start mission upload: {e}")
            self._upload_in_progress = False
            return False

    def download_mission(self) -> bool:
        """Download mission from vehicle"""
        if not self.connection or not self.connection.is_connected():
            print("No connection available")
            return False

        if self._download_in_progress:
            print("Download already in progress")
            return False

        try:
            self._download_in_progress = True
            self._received_waypoints = []
            self._expected_count = 0

            # Request mission list
            msg = self.connection.mavlink_connection.mav.mission_request_list_encode(
                self.connection.target_system_id,
                self.connection.target_component_id,
                mavlink.MAV_MISSION_TYPE_MISSION
            )

            success = self.connection.send_message(msg)
            if not success:
                self._download_in_progress = False

            return success

        except Exception as e:
            print(f"Failed to start mission download: {e}")
            self._download_in_progress = False
            return False

    def _handle_mission_request(self, msg):
        """Handle MISSION_REQUEST_INT from vehicle"""
        if not self._upload_in_progress:
            return

        try:
            seq = msg.seq

            if seq < len(self._upload_waypoints):
                waypoint = self._upload_waypoints[seq]
                mission_msg = waypoint.to_mavlink_message(
                    self.connection.target_system_id,
                    self.connection.target_component_id
                )

                self.connection.send_message(mission_msg)
                self.mission_progress.emit(seq + 1, len(self._upload_waypoints))
            else:
                print(f"Requested waypoint {seq} is out of range")

        except Exception as e:
            print(f"Error handling mission request: {e}")

    def _handle_mission_ack(self, msg):
        """Handle MISSION_ACK from vehicle"""
        if self._upload_in_progress:
            self._upload_in_progress = False
            success = msg.type == mavlink.MAV_MISSION_ACCEPTED
            self.mission_uploaded.emit(success)

            if success:
                print("Mission upload successful")
            else:
                print(f"Mission upload failed: {msg.type}")

        elif self._download_in_progress:
            self._download_in_progress = False
            success = msg.type == mavlink.MAV_MISSION_ACCEPTED

            if success and len(self._received_waypoints) == self._expected_count:
                # Update current mission
                self.current_mission.clear()
                for wp_data in self._received_waypoints:
                    waypoint = Waypoint.from_mavlink_message(wp_data)
                    self.current_mission.add_waypoint(waypoint)

                self.mission_changed.emit()
                print(f"Mission download successful: {len(self._received_waypoints)} waypoints")
            else:
                print(f"Mission download failed: {msg.type}")

            self.mission_downloaded.emit(success)

    def _handle_mission_count(self, msg):
        """Handle MISSION_COUNT from vehicle"""
        if not self._download_in_progress:
            return

        try:
            self._expected_count = msg.count
            self._received_waypoints = [None] * msg.count

            # Request first waypoint
            if msg.count > 0:
                request_msg = self.connection.mavlink_connection.mav.mission_request_int_encode(
                    self.connection.target_system_id,
                    self.connection.target_component_id,
                    0,  # sequence
                    mavlink.MAV_MISSION_TYPE_MISSION
                )
                self.connection.send_message(request_msg)
            else:
                # Empty mission
                self._download_in_progress = False
                self.mission_downloaded.emit(True)

        except Exception as e:
            print(f"Error handling mission count: {e}")

    def _handle_mission_item(self, msg):
        """Handle MISSION_ITEM_INT from vehicle"""
        if not self._download_in_progress:
            return

        try:
            seq = msg.seq

            if seq < len(self._received_waypoints):
                self._received_waypoints[seq] = msg
                self.mission_progress.emit(seq + 1, self._expected_count)

                # Request next waypoint
                next_seq = seq + 1
                if next_seq < self._expected_count:
                    request_msg = self.connection.mavlink_connection.mav.mission_request_int_encode(
                        self.connection.target_system_id,
                        self.connection.target_component_id,
                        next_seq,
                        mavlink.MAV_MISSION_TYPE_MISSION
                    )
                    self.connection.send_message(request_msg)
                else:
                    # All waypoints received, send ACK
                    ack_msg = self.connection.mavlink_connection.mav.mission_ack_encode(
                        self.connection.target_system_id,
                        self.connection.target_component_id,
                        mavlink.MAV_MISSION_ACCEPTED,
                        mavlink.MAV_MISSION_TYPE_MISSION
                    )
                    self.connection.send_message(ack_msg)
            else:
                print(f"Received waypoint {seq} is out of range")

        except Exception as e:
            print(f"Error handling mission item: {e}")

    def _handle_mission_current(self, msg):
        """Handle MISSION_CURRENT from vehicle"""
        try:
            current_seq = msg.seq
            # Update current waypoint status
            for i, waypoint in enumerate(self.current_mission.waypoints):
                waypoint.current = (i == current_seq)
            self.mission_changed.emit()
        except Exception as e:
            print(f"Error handling mission current: {e}")

    def _handle_mission_item_reached(self, msg):
        """Handle MISSION_ITEM_REACHED from vehicle"""
        try:
            reached_seq = msg.seq
            print(f"Waypoint {reached_seq} reached")
        except Exception as e:
            print(f"Error handling mission item reached: {e}")

    # Utility methods

    def get_mission_info(self) -> Dict[str, Any]:
        """Get mission information"""
        bounds = self.current_mission.get_bounds()
        return {
            'name': self.current_mission.name,
            'waypoint_count': len(self.current_mission.waypoints),
            'total_distance': self.current_mission.get_total_distance(),
            'bounds': bounds,
            'has_home': self.current_mission.home_position is not None
        }

    def create_survey_mission(self, center_lat: float, center_lon: float,
                            width: float, height: float, altitude: float = 50.0,
                            spacing: float = 20.0) -> bool:
        """Create a survey/lawn mower pattern mission"""
        try:
            self.clear_mission()

            # Calculate corner coordinates (simplified rectangular pattern)
            # This is a basic implementation - more sophisticated survey patterns
            # would account for wind direction, camera overlap, etc.

            # Convert distances to approximate lat/lon offsets
            lat_offset = (height / 2) / 111320  # ~111320 meters per degree latitude
            lon_offset = (width / 2) / (111320 * math.cos(math.radians(center_lat)))

            # Create survey grid
            y_steps = int(height / spacing) + 1

            for i in range(y_steps):
                y = center_lat + lat_offset - (i * spacing / 111320)

                if i % 2 == 0:  # Even rows: left to right
                    x1 = center_lon - lon_offset
                    x2 = center_lon + lon_offset
                else:  # Odd rows: right to left  
                    x1 = center_lon + lon_offset
                    x2 = center_lon - lon_offset

                # Add start point of row
                self.add_waypoint(y, x1, altitude)
                # Add end point of row
                self.add_waypoint(y, x2, altitude)

            return True

        except Exception as e:
            print(f"Failed to create survey mission: {e}")
            return False
