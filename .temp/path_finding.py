from typing import Dict, List, Optional, Tuple, Set
from zone import Zone, ZoneType
from map_parser import MapParser, Graph
from heapq import heappop, heappush


class Dijkstra:
    def __init__(self, graph: Graph):
        self.graph = graph

    # Find the shortest path between two zones
    def find_path(self,
                  start: str,
                  goal: str
                  ) -> Optional[List[str]]:
        if start not in self.graph:
            raise ValueError(f"Start zone '{start} not found")
        if goal not in self.graph:
            raise ValueError(f"Goal zone '{start} not found")

        if start == goal:
            return [start]

        # Dijkstra's algorithm
        # Priority queue: (distance, zone_name)
        # Use a list of tuples for the priority queue
        pq: List[Tuple[float, str]] = [(0.0, start)]

        # Best distances found so far
        distances: Dict[str, float] = {start: 0.0}

        # Track previous zone in optimal path
        previous: Dict[str, Optional[str]] = {start: None}

        # Visited zones
        visited: Set[str] = set()

        # Process the priority queue
        # Use a while loop to process the priority queue
        while pq:
            current_dist, current_zone = heappop(pq)

            if current_zone in visited:
                continue
            visited.add(current_zone)

            if current_zone == goal:
                return self.get_path(previous, start, goal)

            neighbors = self.graph.get_neighbors(current_zone)
            for neighbor in neighbors:
                if neighbor in visited:
                    continue

                neighbor_zone = self.graph.zones[neighbor]
                if neighbor_zone.is_blocked:
                    continue

                neighbor_cost = self.graph.get_zone_cost(neighbor_zone)
                new_dist = neighbor_cost + current_dist

                if neighbor not in distances or new_dist < distances[neighbor]:
                    distances[neighbor] = new_dist
                    previous[neighbor] = current_zone
                    heappush(pq, (new_dist, neighbor))

        return None

    def get_path(self,
                 previous: Dict[str, Optional[str]],
                 start: str,
                 goal: str) -> List[str]:
        path: List[str] = []
        current: Optional[str] = goal

        while current is not None:
            path.append(current)
            current = previous.get(current)
        path.reverse()

        if path[0] != start:
            raise ValueError(
                f"Path reconstruction failed: expected start '{start}', "
                f"got '{path[0]}'"
            )

    def get_distance(self,
                  start: str,
                  goal: str
                  ) -> Optional[float]:
        if start not in self.graph:
            raise ValueError(f"Start zone '{start} not found")
        if goal not in self.graph:
            raise ValueError(f"Goal zone '{start} not found")

        if start == goal:
            return [start]

        pq = List[Tuple[float, str]] = [(0.0, start)]

        distances: Dict[str, float] = {start: 0.0}

        previous: Dict[str, Optional[str]] = {start: None}

        visited: Set[str] = set()

        while pq:
            current_dist, current_zone = heappop(pq)

            if current_zone in visited:
                continue
            visited.add(current_zone)

            if current_zone == goal:
                return current_dist

            neighbors = self.graph.get_neighbors(current_zone)
            for neighbor in neighbors:
                if neighbor in visited:
                    continue

                neighbor_zone = self.graph.zones[neighbor]
                if neighbor_zone.is_blocked:
                    continue

                neighbor_cost = self.graph.get_zone_cost(neighbor_zone)
                new_dist = neighbor_cost + current_dist

                if neighbor not in distances or new_dist < distances[neighbor]:
                    distances[neighbor] = new_dist
                    previous[neighbor] = current_zone
                    heappush(pq, (new_dist, neighbor))

        return 0.0
