from enum import Enum


class ZoneType(Enum):
    NORMAL = "normal"           # Cost: 1 turn
    RESTRICTED = "restricted"   # Cost: 2 turns (drone in transit)
    PRIORITY = "priority"       # Cost: 1 turn (preferred in pathfinding)
    BLOCKED = "blocked"         # Cannot be entered
    START = "start"             # Starting zone (unlimited capacity)
    END = "end"                 # Ending zone (unlimited capacity)


class Zone:
    def __init__(self,
                 name: str,
                 x: int,
                 y: int,
                 zone_type: ZoneType,
                 color: str = 'none',
                 max_capacity: int = 1
                 ) -> None:
        # ----------Initialization---------- #
        self.name = name
        self.x = x
        self.y = y
        self.zone_type = zone_type
        self.color = color
        self.max_capacity = max_capacity
        self.current_occupancy: int = 0

    @property
    def movement_cost(self) -> int:
        if self.zone_type == ZoneType.RESTRICTED:
            return 2
        return 1

    @property
    def is_blocked(self) -> bool:
        return self.zone_type == ZoneType.BLOCKED

    @property
    def has_space(self):
        return self.current_occupancy < self.max_capacity

    def add_drone(self) -> bool:
        if not self.has_space:
            return False
        self.current_occupancy += 1
        return True

    def remove_drone(self) -> bool:
        if self.current_occupancy <= 0:
            return False
        self.current_occupancy -= 1
        return True

    def reset_occupancy(self):
        self.current_occupancy = 0
