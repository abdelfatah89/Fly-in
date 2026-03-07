"""Dijkstra pathfinding algorithm.

Simple implementation of Dijkstra's algorithm for finding
shortest paths in the drone navigation graph.
"""

import heapq
from typing import Dict, List, Optional, Tuple
from ..models import MapData, Zone, ZoneType


class Pathfinder:
    """Finds shortest paths using Dijkstra's algorithm.

    This implementation considers zone movement costs:
    - Normal zones: cost = 1.0
    - Restricted zones: cost = 2.0
    - Priority zones: cost = 0.5 (preferred)
    - Blocked zones: cannot be traversed
    """

    def __init__(self, map_data: MapData) -> None:
        """Initialize pathfinder with map data.

        Args:
            map_data: The map to find paths in
        """
        self.map_data = map_data

    def find_path(
        self,
        start: str,
        goal: str
    ) -> Optional[List[str]]:
        """Find shortest path from start to goal using Dijkstra.

        Args:
            start: Starting zone name
            goal: Goal zone name

        Returns:
            List of zone names forming the path (including start and goal),
            or None if no path exists
        """
        # Validate zones exist
        if start not in self.map_data.zones:
            raise ValueError(f"Start zone '{start}' not found")
        if goal not in self.map_data.zones:
            raise ValueError(f"Goal zone '{goal}' not found")

        # If already at goal
        if start == goal:
            return [start]

        # Dijkstra's algorithm
        # Priority queue: (distance, zone_name)
        pq: List[Tuple[float, str]] = [(0.0, start)]

        # Best distances found so far
        distances: Dict[str, float] = {start: 0.0}

        # Track previous zone in optimal path
        previous: Dict[str, Optional[str]] = {start: None}

        # Visited zones
        visited: set[str] = set()

        while pq:
            current_dist, current_zone = heapq.heappop(pq)

            # Skip if already visited
            if current_zone in visited:
                continue

            visited.add(current_zone)

            # Found goal
            if current_zone == goal:
                return self._reconstruct_path(previous, start, goal)

            # Check all neighbors
            neighbors = self.map_data.get_neighbors(current_zone)
            for neighbor in neighbors:
                # Skip visited
                if neighbor in visited:
                    continue

                neighbor_zone = self.map_data.zones[neighbor]

                # Skip blocked zones
                if neighbor_zone.is_blocked:
                    continue

                # Calculate cost to neighbor
                edge_cost = self._get_zone_cost(neighbor_zone)
                new_dist = current_dist + edge_cost

                # Update if better path found
                if neighbor not in distances or new_dist < distances[neighbor]:
                    distances[neighbor] = new_dist
                    previous[neighbor] = current_zone
                    heapq.heappush(pq, (new_dist, neighbor))

        # No path found
        return None

    def _get_zone_cost(self, zone: Zone) -> float:
        """Get the cost of entering a zone.

        Args:
            zone: Zone to get cost for

        Returns:
            Cost value (lower is better)
        """
        if zone.zone_type == ZoneType.RESTRICTED:
            # Restricted zones take 2 turns
            return 2.0
        elif zone.zone_type == ZoneType.PRIORITY:
            # Priority zones are preferred (lower cost)
            return 0.5
        elif zone.zone_type == ZoneType.BLOCKED:
            # Should never reach here due to blocking check
            return float('inf')
        else:
            # Normal, start, end zones
            return 1.0

    def _reconstruct_path(
        self,
        previous: Dict[str, Optional[str]],
        start: str,
        goal: str
    ) -> List[str]:
        """Reconstruct path from previous pointers.

        Args:
            previous: Dictionary mapping zone to previous zone
            start: Starting zone
            goal: Goal zone

        Returns:
            List of zone names forming the path
        """
        path: List[str] = []
        current: Optional[str] = goal

        while current is not None:
            path.append(current)
            current = previous.get(current)

        path.reverse()

        # Validate path starts at start
        if path[0] != start:
            raise RuntimeError(
                f"Path reconstruction failed: expected start '{start}', "
                f"got '{path[0]}'"
            )

        return path

    def get_distance(self, start: str, goal: str) -> Optional[float]:
        """Get shortest distance from start to goal.

        Args:
            start: Starting zone name
            goal: Goal zone name

        Returns:
            Shortest distance, or None if no path exists
        """
        # Run Dijkstra but only track distances
        if start not in self.map_data.zones:
            raise ValueError(f"Start zone '{start}' not found")
        if goal not in self.map_data.zones:
            raise ValueError(f"Goal zone '{goal}' not found")

        if start == goal:
            return 0.0

        pq: List[Tuple[float, str]] = [(0.0, start)]
        distances: Dict[str, float] = {start: 0.0}
        visited: set[str] = set()

        while pq:
            current_dist, current_zone = heapq.heappop(pq)

            if current_zone in visited:
                continue

            visited.add(current_zone)

            if current_zone == goal:
                return current_dist

            neighbors = self.map_data.get_neighbors(current_zone)
            for neighbor in neighbors:
                if neighbor in visited:
                    continue

                neighbor_zone = self.map_data.zones[neighbor]
                if neighbor_zone.is_blocked:
                    continue

                edge_cost = self._get_zone_cost(neighbor_zone)
                new_dist = current_dist + edge_cost

                if neighbor not in distances or new_dist < distances[neighbor]:
                    distances[neighbor] = new_dist
                    heapq.heappush(pq, (new_dist, neighbor))

        return None
