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
        if start not in self.graph.zones:
            raise ValueError(f"Start zone '{start}' not found")
        if goal not in self.graph.zones:
            raise ValueError(f"Goal zone '{goal}' not found")

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
            # Pop the smallest distance from the priority queue
            current_dist, current_zone = heappop(pq)

            # Skip if the zone has already been visited
            if current_zone in visited:
                continue
            visited.add(current_zone)

            # If the goal has been reached, return the path
            if current_zone == goal:
                return self.get_path(previous, start, goal)

            # Get the neighbors of the current zone
            zone_obj = self.graph.zones[current_zone]
            neighbors = self.graph.get_neighbors(zone_obj)
            for neighbor in neighbors:
                # Get the neighbor zone
                neighbor_zone = neighbor[0]
                neighbor_name = neighbor_zone.name

                # Skip if the neighbor has already been visited
                if neighbor_name in visited:
                    continue

                # Skip if the neighbor is blocked
                if neighbor_zone.is_blocked:
                    continue

                # Get the cost of the neighbor zone
                neighbor_cost = self.graph.get_zone_cost(neighbor_zone)
                new_dist = neighbor_cost + current_dist

                # Update the distance if a shorter path is found
                if neighbor_name not in distances or new_dist < distances[neighbor_name]:
                    distances[neighbor_name] = new_dist
                    previous[neighbor_name] = current_zone
                    heappush(pq, (new_dist, neighbor_name))

        return None

    # Reconstruct the path from the previous dictionary
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

        # Check if the path is valid
        if path[0] != start:
            raise ValueError(
                f"Path reconstruction failed: expected start '{start}', "
                f"got '{path[0]}'"
            )
        return path

    # Find the shortest distance between two zones
    def get_distance(self,
                  start: str,
                  goal: str
                  ) -> Optional[float]:
        # Check if the start and goal zones exist
        if start not in self.graph.zones:
            raise ValueError(f"Start zone '{start}' not found")
        if goal not in self.graph.zones:
            raise ValueError(f"Goal zone '{goal}' not found")

        # If the start and goal are the same, return the start
        if start == goal:
            return 0.0

        # Priority queue: (distance, zone_name)
        pq: List[Tuple[float, str]] = [(0.0, start)]
        # Best distances found so far
        distances: Dict[str, float] = {start: 0.0}
        # Visited zones
        visited: Set[str] = set()

        # Process the priority queue
        while pq:
            current_dist, current_zone = heappop(pq)

            # Skip if the zone has already been visited
            if current_zone in visited:
                continue
            visited.add(current_zone)

            # Check if the goal has been reached return the distance
            if current_zone == goal:
                return current_dist

            # Get the neighbors of the current zone
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
                    heappush(pq, (new_dist, neighbor))

        return None
