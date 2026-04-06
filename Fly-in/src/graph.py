from typing import Dict, List, Optional, Tuple
from .zone import Zone, ZoneType
from .connection import Connection
from .drone import Drone


class Graph:
    def __init__(self,
                 nb_drones: int,
                 start_hub: Zone,
                 end_hub: Zone,
                 zones: Dict[str, Zone],
                 connections: List[Connection]
                 ) -> None:
        self.adjacency: dict[str, list[tuple[Zone, Connection]]] = {}
        self.nb_drones = nb_drones
        self.start_hub = start_hub
        self.end_hub = end_hub
        self.zones = zones
        self.connections = connections
        self.drones: List[Drone] = [
            Drone(i, start_hub) for i in range(1, nb_drones + 1)
        ]
        self.adjacency = self._build_adjacency()

    def get_zone_cost(self, zone: Zone) -> float:
        if zone.zone_type == ZoneType.RESTRICTED:
            return 2.0
        elif zone.zone_type == ZoneType.PRIORITY:
            return 0.5
        elif zone.zone_type == ZoneType.BLOCKED:
            return float('inf')
        else:
            return 1.0

    def _build_adjacency(self) -> Dict[str, List[Tuple[Zone, Connection]]]:
        adjacency: dict[str, list[tuple[Zone, Connection]]] = {}
        for zone in self.zones:
            neighbors: List[Tuple[Zone, Connection]] = []
            for conn in self.connections:
                if conn.zone1 == zone:
                    neighbor_zone = self.zones[conn.zone2]
                    neighbors.append((neighbor_zone, conn))
                elif conn.zone2 == zone:
                    neighbor_zone = self.zones[conn.zone1]
                    neighbors.append((neighbor_zone, conn))
            adjacency[zone] = neighbors
        return adjacency

    def get_neighbors(self, zone: Zone) -> List[Tuple[Zone, Connection]]:
        return self.adjacency.get(zone.name, [])

    def get_connection(self, zone1: Zone, zone2: Zone) -> Optional[Connection]:
        for conn in self.connections:
            if zone1.name in conn.zones and zone2.name in conn.zones:
                return conn
        return None
