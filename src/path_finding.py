"""Pathfinding module: Dijkstra's algorithm with forbidden zone support."""
from typing import List, Optional, Dict, Set, Tuple
from heapq import heappop, heappush
from .graph import Graph
from .zone import Zone, ZoneType

_HOP_EPS = 0.001


class PathFindingError(Exception):
    """Raised when path reconstruction fails."""

    pass


class Dijkstra:
    """Dijkstra's shortest-path algorithm for the drone zone network.

    Supports optional forbidden zones and connections to enable
    dynamic rerouting without modifying the underlying graph.

    Priority zones cost 0.0 and non-priority zones receive a tiny
    hop penalty (_HOP_EPS) so that a single priority neighbor is
    preferred (equal non-priority hop count → tie → tiebreaker wins)
    but long detours through many priority zones are not.

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

        Non-priority zones receive a tiny hop penalty so that paths
        through priority neighbors are preferred when the number of
        non-priority hops is equal, but longer detours are penalised.

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
        priority_queue: List[Tuple[float, int, int, str]] = [
            (0.0, 0, counter, start.name)]
        shortest_dist: Dict[str, float] = {start.name: 0.0}
        best_prio: Dict[str, int] = {start.name: 0}
        way_back: Dict[str, Optional[str]] = {start.name: None}
        visited: Set[str] = set()

        while priority_queue:
            current_dist, neg_prio, _, current_zone_name = heappop(
                priority_queue)
            if current_dist > shortest_dist.get(
                    current_zone_name, float('inf')):
                continue
            if current_zone_name in visited:
                continue
            visited.add(current_zone_name)
            current_prio = -neg_prio

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
                        and _connection.key in forbidden_connections):
                    continue

                is_prio = (
                    neighbor_zone.zone_type == ZoneType.PRIORITY)
                neighbor_cost = self.graph.get_zone_cost(
                    neighbor_zone)
                if not is_prio:
                    neighbor_cost += _HOP_EPS
                to_neighbor_dist = current_dist + neighbor_cost
                to_neighbor_prio = current_prio + (
                    1 if is_prio else 0)
                neighbor_name = neighbor_zone.name

                better_dist = (
                    to_neighbor_dist
                    < shortest_dist.get(neighbor_name, float('inf')))
                same_dist_more_prio = (
                    to_neighbor_dist
                    == shortest_dist.get(neighbor_name, float('inf'))
                    and to_neighbor_prio
                    > best_prio.get(neighbor_name, -1))

                if better_dist or same_dist_more_prio:
                    shortest_dist[neighbor_name] = to_neighbor_dist
                    best_prio[neighbor_name] = to_neighbor_prio
                    way_back[neighbor_name] = current_zone.name
                    counter += 1
                    heappush(priority_queue, (
                        to_neighbor_dist,
                        -to_neighbor_prio,
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
