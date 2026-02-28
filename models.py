from enum import Enum
from typing import Optional, List, Dict, Tuple


class ZoneType(Enum):
    NORMAL = 1
    PRIORITY = 1
    RESTRICTED = 2
    BLOCKED = float('inf')


class Zone:
    def __init__(self, name: str, x: int, y: int,
                 color: str,
                 zone_type: ZoneType = ZoneType.NORMAL,
                 max_drones: int = 1
                 ):
        self.name = name
        self.x = x
        self.y = y
        self.zone_type: ZoneType = zone_type
        self.max_drones: int = max_drones
        self.color: str = color
        self.neighbors: List[Tuple[Zone, Connection]] = []


class Connection:
    def __init__(self, zone1: str, zone2: str, max_capacity: int):
        self.zone1 = zone1
        self.zone2 = zone2
        self.max_capacity = max_capacity
        self.name = f"{self.zone1}-{self.zone2}"
        self._key: Optional[Tuple[str, str]] = None

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
                 connections: Dict[str, Connection]
                 ) -> None:

        self.num_drones = num_drones
        self.zones: Dict[str, Zone] = zones
        self.connections: Dict[str, Connection] = connections
        self.start_zone = start_zone
        self.end_zone = end_zone

    def get_neighbors(self,
                      zone: Zone) -> List[Tuple[Zone, Connection]]:
        neighbors: List[Tuple[Zone, Connection]] = []
        for conn in self.connections.values():
            if conn.zone1 == zone.name:
                neighbor_zone = self.zones[conn.zone2]
                neighbors.append((neighbor_zone, conn))
            elif conn.zone2 == zone.name:
                neighbor_zone = self.zones[conn.zone1]
                neighbors.append((neighbor_zone, conn))
        return neighbors

    def has_connection(self, zone1: Zone, zone2: Zone) -> bool:
        neighbors = self.get_neighbors(zone1)
        if zone2 in neighbors:
            return True
        return False
