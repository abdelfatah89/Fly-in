from typing import List, Optional, Dict, Set
from heapq import heappop, heappush
from .graph import Graph
from .zone import Zone


class PathFindingError(Exception):
    pass


class Dijkstra:
    def __init__(self, graph: Graph) -> None:
        self.graph = graph

    def find_path(self, start: Zone, goal: Zone) -> List[Zone]:
        if start.name == goal.name:
            return [start]

        counter = 0
        priority_queue = [(0.0, counter, start.name)]
        shortest_dist: Dict[str, float] = {start.name: 0.0}
        way_back: Dict[str, Optional[str]] = {start.name: None}
        visited: Set[str] = set()

        while priority_queue:
            current_dist, _, current_zone_name = heappop(priority_queue)
            if current_dist > shortest_dist[current_zone_name]:
                continue
            if current_zone_name in visited:
                continue
            visited.add(current_zone_name)

            if current_zone_name == goal:
                return self.get_path(start, goal, way_back)

            current_zone = self.graph.zones[current_zone_name]
            neighbors = self.graph.get_neighbors(current_zone)
            for neighbor_zone, _connection in neighbors:
                if neighbor_zone.name in visited:
                    continue
                if neighbor_zone.is_blocked:
                    continue

                neighbor_dist = self.graph.get_zone_cost(neighbor_zone)
                to_neighbor_dist = current_dist + neighbor_dist
                neighbor_name = neighbor_zone.name
                if (neighbor_name not in shortest_dist
                   or to_neighbor_dist < shortest_dist[neighbor_name]):
                    shortest_dist[neighbor_zone.name] = to_neighbor_dist
                    way_back[neighbor_name] = current_zone.name
                    counter += 1
                    heappush(priority_queue, (to_neighbor_dist,
                                              counter, neighbor_zone.name))
        return []

    def get_path(self,
                 start: Zone,
                 goal: Zone,
                 way_back: Dict[str, Optional[str]]
                 ) -> List[Zone]:
        path_names: List[str] = []
        current_name: Optional[str] = goal.name

        while current_name is not None:
            path_names.append(current_name)
            current_name = way_back.get(current_name)
        path_names.reverse()

        if path_names[0] != start.name:
            raise PathFindingError("Invalid path reconstruction")

        return [self.graph.zones[name] for name in path_names]
