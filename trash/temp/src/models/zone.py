"""Zone model - represents a node in the drone navigation graph."""

from enum import Enum


class ZoneType(Enum):
    """Types of zones with different movement costs."""

    NORMAL = "normal"           # Cost: 1 turn
    RESTRICTED = "restricted"   # Cost: 2 turns (drone in transit)
    PRIORITY = "priority"       # Cost: 1 turn (preferred in pathfinding)
    BLOCKED = "blocked"         # Cannot be entered
    START = "start"             # Starting zone (unlimited capacity)
    END = "end"                 # Goal zone (unlimited capacity)


class Zone:
    """Represents a zone (node) in the map.

    A zone is a location where drones can be positioned.
    Each zone has a type, capacity, and position.
    """

    def __init__(
        self,
        name: str,
        x: float,
        y: float,
        zone_type: ZoneType,
        max_capacity: int = 1,
        color: str = "gray"
    ) -> None:
        """Initialize a zone.

        Args:
            name: Unique identifier for the zone
            x: X coordinate for visualization
            y: Y coordinate for visualization
            zone_type: Type of zone (affects movement cost)
            max_capacity: Maximum drones allowed simultaneously
            color: Color for GUI visualization
        """
        self.name: str = name
        self.x: float = x
        self.y: float = y
        self.zone_type: ZoneType = zone_type
        self.max_capacity: int = max_capacity
        self.color: str = color

        # Current occupancy (number of drones currently in this zone)
        self.current_occupancy: int = 0

    @property
    def movement_cost(self) -> int:
        """Get the number of turns required to move through this zone.

        Returns:
            Number of turns (1 for normal/priority, 2 for restricted)
        """
        if self.zone_type == ZoneType.RESTRICTED:
            return 2
        return 1

    @property
    def is_blocked(self) -> bool:
        """Check if this zone cannot be entered.

        Returns:
            True if zone is blocked
        """
        return self.zone_type == ZoneType.BLOCKED

    @property
    def is_unlimited_capacity(self) -> bool:
        """Check if this zone has unlimited capacity.

        Returns:
            True if zone is start or end (unlimited capacity)
        """
        return self.zone_type in (ZoneType.START, ZoneType.END)

    @property
    def has_space(self) -> bool:
        """Check if zone has space for another drone.

        Returns:
            True if there's space available
        """
        if self.is_unlimited_capacity:
            return True
        return self.current_occupancy < self.max_capacity

    def add_drone(self) -> bool:
        """Add a drone to this zone if space is available.

        Returns:
            True if drone was added, False if no space
        """
        if not self.has_space:
            return False
        self.current_occupancy += 1
        return True

    def remove_drone(self) -> bool:
        """Remove a drone from this zone.

        Returns:
            True if drone was removed, False if zone was empty
        """
        if self.current_occupancy <= 0:
            return False
        self.current_occupancy -= 1
        return True

    def reset_occupancy(self) -> None:
        """Reset occupancy count to zero."""
        self.current_occupancy = 0

    def __str__(self) -> str:
        """String representation of the zone."""
        capacity_str = "∞" if self.is_unlimited_capacity else str(self.max_capacity)
        return (
            f"Zone({self.name}, type={self.zone_type.value}, "
            f"occupancy={self.current_occupancy}/{capacity_str})"
        )

    def __repr__(self) -> str:
        """Detailed representation of the zone."""
        return (
            f"Zone(name='{self.name}', pos=({self.x}, {self.y}), "
            f"type={self.zone_type.value}, capacity={self.max_capacity})"
        )
