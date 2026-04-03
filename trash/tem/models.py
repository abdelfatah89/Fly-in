from enum import Enum
from typing import Optional, List, Dict, Tuple


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
                 max_drones: int = 1
                 ) -> None:

        self.name = name
        self.x = x
        self.y = y
        self.zone_type: ZoneType = zone_type
        self.max_drones: int = max_drones
        self.color: str = color
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
    def is_unlimited_capacity(self) -> bool:
        return self.zone_type in (ZoneType.START, ZoneType.END)

    @property
    def has_space(self) -> bool:
        if self.is_unlimited_capacity:
            return True
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

    def reset_occupancy(self) -> None:
        self.current_occupancy = 0

class Connection:
    def __init__(self, zone1: str, zone2: str, max_capacity: int):
        self.zone1 = zone1
        self.zone2 = zone2
        self.max_capacity = max_capacity
        self.name = f"{self.zone1}-{self.zone2}"
        self.current_usage: int = 0
        self._key: Optional[Tuple[str, str]] = None

    def connects(self, zone_name: str) -> bool:
        return zone_name in (self.zone1, self.zone2)

    def get_other_zone(self, zone_name: str) -> str:
        if zone_name == self.zone1.name:
            return self.zone2
        elif zone_name == self.zone2.name:
            return self.zone1
        else:
            print(
                f"Zone '{zone_name}' is not part of connection "
                f"{self.zone1_name}-{self.zone2_name}"
                )
            return False

    def get_zones(self) -> Tuple[str, str]:
        return (self.zone1_name, self.zone2_name)

    @property
    def has_capacity(self) -> bool:
        return self.current_usage < self.max_capacity

    def use_capacity(self) -> bool:
        if not self.has_capacity:
            return False
        self.current_usage -= 1
        return True

    def reset_usage(self) -> None:
        self.current_usage = 0



    @property
    def key(self) -> Tuple[str, str]:
        """Return a canonical (sorted) tuple of the two zone names."""
        if self._key is None:
            self._key = tuple(sorted([self.zone1, self.zone2]))
        return self._key


class Graph:
    def __init__(self,
                 start_zone: Zone,
                 end_zone: Zone,
                 num_drones: int,
                 zones: Dict[str, Zone],
                 connections: List[Connection]
                 ) -> None:

        self.num_drones = num_drones
        self.zones: Dict[str, Zone] = zones
        self.connections: List[Connection] = connections
        self.start_zone = start_zone
        self.end_zone = end_zone

    def get_neighbors(self,
                      zone: Zone) -> List[Tuple[Zone, Connection]]:
        neighbors: List[Tuple[Zone, Connection]] = []
        for conn in self.connections:
            if conn.zone1 == zone.name:
                neighbor_zone = self.zones[conn.zone2]
                neighbors.append((neighbor_zone, conn))
            elif conn.zone2 == zone.name:
                neighbor_zone = self.zones[conn.zone1]
                neighbors.append((neighbor_zone, conn))
        return neighbors

    def has_connection(self, zone1: Zone, zone2: Zone) -> bool:
        z_neighbors = self.get_neighbors(zone1)
        if zone2 in z_neighbors:
            return True
        return False
