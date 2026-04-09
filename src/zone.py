"""Zone model: defines zone types, capacity, and occupancy tracking."""
from enum import Enum


class ZoneType(Enum):
    """Enumeration of possible zone types in the drone network."""

    NORMAL = "normal"
    RESTRICTED = "restricted"
    PRIORITY = "priority"
    BLOCKED = "blocked"


class Zone:
    """Represents a single zone (hub) in the drone network graph.

    Attributes:
        name: Unique identifier for the zone.
        x: Horizontal coordinate on the map.
        y: Vertical coordinate on the map.
        zone_type: The type of zone (normal, restricted, priority, blocked).
        color: Display color for visual rendering.
        max_capacity: Maximum number of drones allowed simultaneously.
        current_occupancy: Number of drones currently in this zone.
    """

    def __init__(self,
                 name: str,
                 x: int,
                 y: int,
                 zone_type: ZoneType,
                 color: str = 'none',
                 max_capacity: int = 1
                 ) -> None:
        """Initialize a Zone with its properties.

        Args:
            name: Unique identifier for the zone.
            x: Horizontal coordinate.
            y: Vertical coordinate.
            zone_type: The type classification of this zone.
            color: Display color string (default 'none').
            max_capacity: Maximum simultaneous drone occupancy (default 1).
        """
        self.name = name
        self.x = x
        self.y = y
        self.zone_type = zone_type
        self.color = color
        self.max_capacity = max_capacity
        self.current_occupancy: int = 0

    @property
    def movement_cost(self) -> int:
        """Return turn cost: 2 for restricted, 1 otherwise."""
        if self.zone_type == ZoneType.RESTRICTED:
            return 2
        return 1

    @property
    def is_blocked(self) -> bool:
        """Return True if this zone is inaccessible."""
        return self.zone_type == ZoneType.BLOCKED

    @property
    def has_space(self) -> bool:
        """Return True if this zone can accept another drone."""
        return self.current_occupancy < self.max_capacity

    def add_drone(self) -> bool:
        """Increment occupancy if capacity allows.

        Returns:
            True if a drone was added, False if at capacity.
        """
        if not self.has_space:
            return False
        self.current_occupancy += 1
        return True

    def remove_drone(self) -> bool:
        """Decrement occupancy if a drone is present.

        Returns:
            True if a drone was removed, False if zone was empty.
        """
        if self.current_occupancy <= 0:
            return False
        self.current_occupancy -= 1
        return True

    def reset_occupancy(self) -> None:
        """Reset the current occupancy counter to zero."""
        self.current_occupancy = 0
