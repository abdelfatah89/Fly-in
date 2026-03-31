from collections import deque
from typing import Dict, List, Optional, Tuple, Set
from zone import Zone, ZoneType
from map_parser import MapParser, Graph
from heapq import heappop, heappush


class Dijkstra:
    def __init__(self, graph: Graph):
        self.graph = graph
        self._blocked_connections: Set[Tuple[str, str]] = set()

    # Temporarily block a connection so it is ignored during pathfinding
    def temporarily_block_connection(self, zone1: str, zone2: str) -> bool:
        """Temporarily block the connection between zone1 and zone2
        so it is ignored during pathfinding."""
        conn = self.graph.get_connection(zone1, zone2)
        if conn is None:
            return False
        self._blocked_connections.add(conn.key)
        return True

    # Restore a single previously blocked connection
    def restore_connection(self, zone1: str, zone2: str) -> bool:
        """Restore a previously blocked connection between zone1 and zone2."""
        conn = self.graph.get_connection(zone1, zone2)
        if conn is None:
            return False
        self._blocked_connections.discard(conn.key)
        return True

    # Restore all temporarily blocked connections at once
    def restore_all_connections(self) -> None:
        """Restore all temporarily blocked connections."""
        self._blocked_connections.clear()

    # Find up to k distinct paths by blocking a key edge after each discovery
    def find_k_paths(self,
                     start: str,
                     goal: str,
                     k: int) -> List[List[str]]:
        """Find up to k distinct paths from start to goal by temporarily
        blocking connections after each path is found."""
        paths: List[List[str]] = []

        for _ in range(k):
            path = self.find_path(start, goal)
            if path is None or path in paths:
                break
            paths.append(path)

            if len(paths) < k:
                blocking_edge = self._find_last_branching_edge(path)
                if blocking_edge is None:
                    break
                self.temporarily_block_connection(*blocking_edge)

        self.restore_all_connections()
        return paths

    # Find the last edge in path where an alternative route to the goal exists
    def _find_last_branching_edge(
            self,
            path: List[str]) -> Optional[Tuple[str, str]]:
        """Return the last (closest to goal) edge (source, target) in path
        where at least one other neighbour of source can still reach the goal
        without revisiting source or any of its ancestors on the path."""
        goal = path[-1]
        for i in range(len(path) - 2, -1, -1):
            source_zone = path[i]
            target_zone = path[i + 1]
            zone = self.graph.zones[source_zone]
            # Exclude source_zone and everything before it to prevent
            # the BFS from looping backward through the original path.
            ancestors_and_source = set(path[:i + 1])
            for nz, conn in self.graph.get_neighbors(zone):
                if (nz.name != target_zone
                        and nz.name not in ancestors_and_source
                        and conn.key not in self._blocked_connections
                        and not nz.is_blocked
                        and self._can_reach_goal_excl(nz.name, goal,
                                                      ancestors_and_source)):
                    return (source_zone, target_zone)
        return None

    # BFS reachability check that avoids a given set of excluded zones
    def _can_reach_goal_excl(self,
                              start: str,
                              goal: str,
                              excluded: Set[str]) -> bool:
        """Return True if goal is reachable from start without visiting
        any zone in excluded (respects currently blocked connections)."""
        queue: deque = deque([start])
        visited: Set[str] = {start}
        while queue:
            current = queue.popleft()
            if current == goal:
                return True
            zone = self.graph.zones[current]
            for nz, conn in self.graph.get_neighbors(zone):
                if (nz.name not in visited
                        and nz.name not in excluded
                        and conn.key not in self._blocked_connections
                        and not nz.is_blocked):
                    visited.add(nz.name)
                    queue.append(nz.name)
        return False

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
                neighbor_conn = neighbor[1]
                neighbor_name = neighbor_zone.name
                
                # Skip if the neighbor has already been visited
                if neighbor_name in visited:
                    continue

                # Skip if the neighbor is blocked
                if neighbor_zone.is_blocked:
                    continue

                # Skip temporarily blocked connections
                if neighbor_conn.key in self._blocked_connections:
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
