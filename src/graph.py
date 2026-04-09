"""Graph model: adjacency structure, zone costs, and drone initialization."""
from typing import Dict, List, Optional, Tuple
from .zone import Zone, ZoneType
from .connection import Connection
from .drone import Drone


class Graph:
    """Represents the complete zone network as an adjacency-list graph.

    Attributes:
        nb_drones: Total number of drones in the simulation.
        start_hub: The starting zone where all drones begin.
        end_hub: The destination zone where drones must arrive.
        zones: Dictionary mapping zone names to Zone objects.
        connections: List of all Connection objects in the graph.
        drones: List of Drone objects initialized at the start hub.
        adjacency: Adjacency list mapping zone names to neighbor tuples.
    """

    def __init__(self,
                 nb_drones: int,
                 start_hub: Zone,
                 end_hub: Zone,
                 zones: Dict[str, Zone],
                 connections: List[Connection]
                 ) -> None:
        """Initialize the graph with zones, connections, and drones.

        Args:
            nb_drones: Number of drones to create.
            start_hub: The starting zone.
            end_hub: The destination zone.
            zones: Dictionary of all zones.
            connections: List of all connections.
        """
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
        """Return the pathfinding cost for entering a zone.

        Args:
            zone: The target zone.

        Returns:
            Cost value: 0.5 for priority, 1.0 for normal,
            2.0 for restricted, infinity for blocked.
        """
        if zone.zone_type == ZoneType.RESTRICTED:
            return 2.0
        elif zone.zone_type == ZoneType.PRIORITY:
            return 0.5
        elif zone.zone_type == ZoneType.BLOCKED:
            return float('inf')
        else:
            return 1.0

    def _build_adjacency(
        self,
    ) -> Dict[str, List[Tuple[Zone, Connection]]]:
        """Build the adjacency list from zones and connections.

        Returns:
            Dictionary mapping each zone name to a list of
            (neighbor Zone, Connection) tuples.
        """
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

    def get_neighbors(
        self, zone: Zone
    ) -> List[Tuple[Zone, Connection]]:
        """Return the list of neighboring zones and their connections.

        Args:
            zone: The zone to query neighbors for.

        Returns:
            List of (neighbor Zone, Connection) tuples.
        """
        return self.adjacency.get(zone.name, [])

    def get_connection(
        self, zone1: Zone, zone2: Zone
    ) -> Optional[Connection]:
        """Find the connection between two zones, if it exists.

        Args:
            zone1: First zone.
            zone2: Second zone.

        Returns:
            The Connection object, or None if no direct link exists.
        """
        for conn in self.connections:
            if zone1.name in conn.zones and zone2.name in conn.zones:
                return conn
        return None
