"""Pathfinding module: Dijkstra's algorithm with forbidden zone support."""
from typing import List, Optional, Dict, Set, Tuple
from heapq import heappop, heappush
from .graph import Graph
from .zone import Zone, ZoneType


class PathFindingError(Exception):
    """Raised when path reconstruction fails."""

    pass


class Dijkstra:
    """Dijkstra's shortest-path algorithm for the drone zone network.

    Supports optional forbidden zones and connections to enable
    dynamic rerouting without modifying the underlying graph.

    Attributes:
        graph: The Graph instance to search.
    """

    def __init__(self, graph: Graph) -> None:
        """Initialize the pathfinder with a graph.

        Args:
            graph: The zone network graph to search.
        """
        self.graph = graph

    def find_path(self,
                  start: Zone,
                  goal: Zone,
                  forbidden_zones: Optional[Set[str]] = None,
                  forbidden_connections:
                      Optional[Set[Tuple[str, str]]] = None
                  ) -> List[Zone]:
        """Find the shortest path from start to goal using Dijkstra.

        When two paths have equal cost, the path passing through more
        priority zones is preferred.  Because priority zones have cost
        0.0, each one exactly offsets one extra normal hop, and the
        negative priority count in the queue breaks the tie.

        Args:
            start: The starting zone.
            goal: The destination zone.
            forbidden_zones: Set of zone names to exclude from search.
            forbidden_connections: Set of connection keys to exclude.

        Returns:
            Ordered list of Zone objects from start to goal,
            or empty list if no path exists.
        """
        if start.name == goal.name:
            return [start]

        counter = 0
        # Queue entries: (cost, -priority_count, counter, zone_name)
        priority_queue: List[Tuple[float, int, int, str]] = [
            (0.0, 0, counter, start.name)]
        # Best known (cost, -priority_count) per zone
        best: Dict[str, Tuple[float, int]] = {
            start.name: (0.0, 0)}
        way_back: Dict[str, Optional[str]] = {start.name: None}
        visited: Set[str] = set()

        while priority_queue:
            current_dist, neg_prio, _, current_zone_name = (
                heappop(priority_queue))
            dist_key = (current_dist, neg_prio)
            if dist_key > best.get(
                    current_zone_name, (float('inf'), 0)):
                continue
            if current_zone_name in visited:
                continue
            visited.add(current_zone_name)

            if current_zone_name == goal.name:
                return self.get_path(start, goal, way_back)

            current_zone = self.graph.zones[current_zone_name]
            neighbors = self.graph.get_neighbors(current_zone)
            for neighbor_zone, _connection in neighbors:
                if neighbor_zone.name in visited:
                    continue
                if neighbor_zone.is_blocked:
                    continue
                if (forbidden_zones
                        and neighbor_zone.name in forbidden_zones):
                    continue
                if (forbidden_connections
                        and _connection.key
                        in forbidden_connections):
                    continue

                neighbor_cost = self.graph.get_zone_cost(
                    neighbor_zone)
                to_neighbor_dist = current_dist + neighbor_cost
                new_neg_prio = neg_prio + (
                    -1 if neighbor_zone.zone_type
                    == ZoneType.PRIORITY else 0)
                neighbor_key = (to_neighbor_dist, new_neg_prio)
                neighbor_name = neighbor_zone.name

                if neighbor_key < best.get(
                        neighbor_name, (float('inf'), 0)):
                    best[neighbor_name] = neighbor_key
                    way_back[neighbor_name] = current_zone.name
                    counter += 1
                    heappush(priority_queue, (
                        to_neighbor_dist, new_neg_prio,
                        counter, neighbor_name))
        return []

    def get_path(self,
                 start: Zone,
                 goal: Zone,
                 way_back: Dict[str, Optional[str]]
                 ) -> List[Zone]:
        """Reconstruct the path from start to goal using the parent map.

        Args:
            start: The starting zone.
            goal: The destination zone.
            way_back: Dictionary mapping each zone name to its predecessor.

        Returns:
            Ordered list of Zone objects from start to goal.

        Raises:
            PathFindingError: If the reconstructed path is invalid.
        """
        path_names: List[str] = []
        current_name: Optional[str] = goal.name

        while current_name is not None:
            path_names.append(current_name)
            current_name = way_back.get(current_name)
        path_names.reverse()

        if path_names[0] != start.name:
            raise PathFindingError("Invalid path reconstruction")

        return [self.graph.zones[name] for name in path_names]
