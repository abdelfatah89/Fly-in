from enum import Enum


class ZoneType(Enum):
    NORMAL = "normal"           # Cost: 1 turn
    RESTRICTED = "restricted"   # Cost: 2 turns (drone in transit)
    PRIORITY = "priority"       # Cost: 1 turn (preferred in pathfinding)
    BLOCKED = "blocked"         # Cannot be entered
    START = "start"             # Starting zone (unlimited capacity)
    END = "end" 


class Zone:
    def __init__(self,
                 name: str, x: float, y: float,
                 zone_type: ZoneType,
                 color: str = 'none',
                 max_capacity: int = 1
                 ) -> None:

        self.name = name
        self.x = x
        self.y = y
        self.zone_type: ZoneType = zone_type
        self.max_capacity: int = max_capacity
        self.color: str = color
        self.current_occupancy: int = 0

    @property
    def movement_cost(self) -> int:
        """Return movement cost at this zone."""
        if self.zone_type == ZoneType.RESTRICTED:
            return 2
        return 1

    @property
    def is_blocked(self) -> bool:
        """Check if this zone blocked or not."""
        return self.zone_type == ZoneType.BLOCKED

    @property
    def is_unlimited_capacity(self) -> bool:
        """Check if this zone is Start or Goal zone."""
        return self.zone_type in (ZoneType.START, ZoneType.END)

    @property
    def has_space(self) -> bool:
        """Check if current occupancy reach max capacity or not yet"""
        if self.is_unlimited_capacity:
            return True
        return self.current_occupancy < self.max_capacity

    def add_drone(self) -> bool:
        """Add drone to zone, increase drone counter"""
        if not self.has_space:
            return False
        self.current_occupancy += 1
        return True

    def remove_drone(self) -> bool:
        """Remove drone to zone, decrease drone counter"""
        if self.current_occupancy <= 0:
            return False
        self.current_occupancy -= 1
        return True

    def reset_occupancy(self) -> None:
        """reset drone counter"""
        self.current_occupancy = 0
