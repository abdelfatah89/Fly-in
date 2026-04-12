"""MapData model - container for the complete map structure."""

from typing import Dict, List, Optional, Set
from .zone import Zone, ZoneType
from .connection import Connection
from .drone import Drone


class MapData:
    """Container for the complete map data structure.

    This class holds all zones, connections, and drones, and provides
    helper methods for navigation and pathfinding.
    """

    def __init__(self) -> None:
        """Initialize an empty map."""
        self.zones: Dict[str, Zone] = {}
        self.connections: List[Connection] = []
        self.drones: List[Drone] = []

        self.start_zone_name: Optional[str] = None
        self.end_zone_name: Optional[str] = None

        # Adjacency list for efficient neighbor lookup
        self._adjacency: Dict[str, Set[str]] = {}
        self._built: bool = False

    def add_zone(self, zone: Zone) -> None:
        """Add a zone to the map.

        Args:
            zone: Zone to add

        Raises:
            ValueError: If zone name already exists
        """
        if zone.name in self.zones:
            raise ValueError(f"Zone '{zone.name}' already exists")

        self.zones[zone.name] = zone
        self._adjacency[zone.name] = set()

        # Track start and end zones
        if zone.zone_type == ZoneType.START:
            self.start_zone_name = zone.name
        elif zone.zone_type == ZoneType.END:
            self.end_zone_name = zone.name

        self._built = False

    def add_connection(self, connection: Connection) -> None:
        """Add a connection to the map.

        Args:
            connection: Connection to add

        Raises:
            ValueError: If connection references unknown zones
        """
        zone1, zone2 = connection.get_zones()

        if zone1 not in self.zones:
            raise ValueError(f"Zone '{zone1}' not found in map")
        if zone2 not in self.zones:
            raise ValueError(f"Zone '{zone2}' not found in map")

        self.connections.append(connection)

        # Update adjacency list (bidirectional)
        self._adjacency[zone1].add(zone2)
        self._adjacency[zone2].add(zone1)

        self._built = False

    def create_drones(self, num_drones: int) -> None:
        """Create drones at the start zone.

        Args:
            num_drones: Number of drones to create

        Raises:
            ValueError: If no start zone is defined
        """
        if self.start_zone_name is None:
            raise ValueError("No start zone defined")

        self.drones = [
            Drone(drone_id=i, start_zone_name=self.start_zone_name)
            for i in range(num_drones)
        ]

        # Update start zone occupancy
        start_zone = self.zones[self.start_zone_name]
        start_zone.current_occupancy = num_drones

    def get_neighbors(self, zone_name: str) -> List[str]:
        """Get all zones adjacent to the given zone.

        Args:
            zone_name: Name of zone to get neighbors for

        Returns:
            List of adjacent zone names

        Raises:
            ValueError: If zone doesn't exist
        """
        if zone_name not in self._adjacency:
            raise ValueError(f"Zone '{zone_name}' not found")

        return list(self._adjacency[zone_name])

    def get_connection(self, zone1: str, zone2: str) -> Optional[Connection]:
        """Get the connection between two zones.

        Args:
            zone1: First zone name
            zone2: Second zone name

        Returns:
            Connection if found, None otherwise
        """
        for conn in self.connections:
            zones = conn.get_zones()
            if (zone1 in zones) and (zone2 in zones):
                return conn
        return None

    def validate(self) -> None:
        """Validate the map structure.

        Raises:
            ValueError: If map is invalid
        """
        if not self.zones:
            raise ValueError("Map has no zones")

        if self.start_zone_name is None:
            raise ValueError("No start zone defined")

        if self.end_zone_name is None:
            raise ValueError("No end zone defined")

        if self.start_zone_name not in self.zones:
            raise ValueError(f"Start zone '{self.start_zone_name}' not found")

        if self.end_zone_name not in self.zones:
            raise ValueError(f"End zone '{self.end_zone_name}' not found")

        # Validate all connections reference existing zones
        for conn in self.connections:
            zone1, zone2 = conn.get_zones()
            if zone1 not in self.zones:
                raise ValueError(f"Connection references unknown zone '{zone1}'")
            if zone2 not in self.zones:
                raise ValueError(f"Connection references unknown zone '{zone2}'")

        self._built = True

    def reset_state(self) -> None:
        """Reset all dynamic state (occupancy, usage, drone positions)."""
        # Reset zone occupancy
        for zone in self.zones.values():
            zone.reset_occupancy()

        # Reset connection usage
        for conn in self.connections:
            conn.reset_usage()

        # Reset drones to start
        if self.start_zone_name and self.drones:
            for drone in self.drones:
                drone.current_zone = self.start_zone_name
                drone.stay_idle()

            # Update start zone occupancy
            self.zones[self.start_zone_name].current_occupancy = len(self.drones)

    def __str__(self) -> str:
        """String representation of the map."""
        return (
            f"MapData(zones={len(self.zones)}, "
            f"connections={len(self.connections)}, "
            f"drones={len(self.drones)})"
        )

    def __repr__(self) -> str:
        """Detailed representation of the map."""
        return (
            f"MapData(zones={len(self.zones)}, "
            f"connections={len(self.connections)}, "
            f"start='{self.start_zone_name}', "
            f"end='{self.end_zone_name}', "
            f"drones={len(self.drones)})"
        )
