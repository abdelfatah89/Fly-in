from enum import Enum
from typing import Optional, List, Dict, Tuple


class ZoneType(Enum):
    NORMAL = "normal"
    PRIORITY = "priority"
    RESTRICTED = "restricted"
    BLOCKED = "blocked"


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
        self.key: Tuple[str, str]


class Graph:
    def __init__(self, start_zone: Zone,
                 end_zone: Zone, num_drones: int) -> None:
        self.num_drones = num_drones
        self.zones: Dict[str, Zone] = {}
        self.connections: Dict[str, Connection] = {}
        self.start_zone = start_zone
        self.end_zone = end_zone

    def get_neighbors(self, zone: Zone) -> List[Zone]:
        neighbors: List[Zone] = []
        return neighbors

    def get_connections(
            self, zone1: Zone, zone2: Zone) -> Optional[Connection]:
        conn: Optional[Connection] = None
        return conn
