from typing import List, Dict, Tuple, Optional, Set
from drone import Drone, DroneStatus
from zone import Zone, ZoneType
from map_parser import Graph
from path_finding import Dijkstra


class MovePlan:
    """Represents a planned move for a single drone."""
    def __init__(self, drone: Drone, from_zone: str, to_zone: Zone,
                 is_restricted: bool = False) -> None:
        self.drone = drone
        self.from_zone = from_zone
        self.to_zone = to_zone
        self.is_restricted = is_restricted


class DynamicSimulator:
    """
    A simulator that advances drones turn-by-turn using dynamic replanning.

    Instead of assigning a single static path to every drone, this simulator recomputes
    a shortest path for each drone every turn, adding an occupancy-based penalty so
    drones spread across multiple routes when congestion builds up.
    """

    def __init__(self, graph: Graph, occupancy_weight: float = 1.0) -> None:
        """
        Args:
            graph: The graph containing zones, connections, and drones.
            occupancy_weight: Multiplier for the congestion penalty. Higher values make
                drones more congestion-sensitive and prefer less crowded routes.
        """
        self.graph = graph
        self.path_finder = Dijkstra(graph)
        self.current_turn: int = 0
        self.output_lines: List[str] = []
        self.occupancy_weight: float = occupancy_weight

    def all_drones_delivered(self) -> bool:
        """Return True if all drones have reached the goal."""
        goal_name = self.graph.end_zone.name
        return all(drone.current_zone == goal_name for drone in self.graph.drones)

    def run(self, max_turns: int = 1000) -> List[str]:
        """Run the simulation until all drones arrive or max_turns is reached."""
        self.current_turn = 0
        self.output_lines = []
        while not self.all_drones_delivered():
            if self.current_turn >= max_turns:
                raise RuntimeError(f"Simulation exceeded {max_turns} turns")
            self.execute_turn()
            self.current_turn += 1
        return self.output_lines

    def execute_turn(self) -> None:
        """Execute a single turn: progress transits, collect intentions, and apply valid moves."""
        self._progress_transits()
        move_plans = self.collect_move_intentions()
        successful = self.validate_and_apply_moves(move_plans)
        self.generate_output(successful)

    def _progress_transits(self) -> None:
        """Advance any drone currently transiting through a restricted zone."""
        for drone in self.graph.drones:
            if drone.status == DroneStatus.IN_TRANSIT:
                drone.progress_transit()
                if drone.status == DroneStatus.WAITING:
                    zone = self.graph.zones[drone.current_zone]
                    zone.add_drone()

    def collect_move_intentions(self) -> List[MovePlan]:
        """
        Collect movement plans for all drones that can move.

        Uses a dynamic path algorithm that adds a congestion penalty so drones prefer
        less crowded routes when possible.
        """
        plans: List[MovePlan] = []
        goal_name = self.graph.end_zone.name
        for drone in self.graph.drones:
            if not drone.can_move():
                continue
            if drone.current_zone == goal_name:
                continue
            path = self.find_dynamic_path(drone.current_zone, goal_name)
            if path is None or len(path) < 2:
                if drone.current_zone == goal_name:
                    drone.mark_delivered()
                continue
            next_zone_name = path[1]
            next_zone = self.graph.zones[next_zone_name]
            is_restricted = (next_zone.zone_type == ZoneType.RESTRICTED)
            plan = MovePlan(drone, drone.current_zone, next_zone, is_restricted)
            plans.append(plan)
        return plans

    def validate_and_apply_moves(self, plans: List[MovePlan]) -> List[MovePlan]:
        """Validate capacity constraints, apply moves, and return the successfully applied plans."""
        zone_usage: Dict[str, int] = {}
        connection_usage: Dict[Tuple[str, str], int] = {}
        successful: List[MovePlan] = []
        for plan in plans:
            if self.can_apply_plan(plan, zone_usage, connection_usage):
                self.apply_move(plan, zone_usage, connection_usage)
                successful.append(plan)
        return successful

    def can_apply_plan(self, plan: MovePlan,
                       zone_usage: Dict[str, int],
                       connection_usage: Dict[Tuple[str, str], int]) -> bool:
        """Return True if the move plan can be applied without exceeding zone/link capacity."""
        to_zone = plan.to_zone
        drones_in_transit = sum(
            1 for d in self.graph.drones
            if d.status == DroneStatus.IN_TRANSIT and d.transit_destination == to_zone.name
        )
        future_occupancy = to_zone.current_occupancy + drones_in_transit + zone_usage.get(to_zone.name, 0)
        if not to_zone.is_unlimited_capacity:
            if future_occupancy >= to_zone.max_capacity:
                return False
        connection = self.graph.get_connection(plan.from_zone, plan.to_zone.name)
        if connection is None:
            return False
        conn_key = tuple(sorted([plan.from_zone, plan.to_zone.name]))
        future_conn_usage = connection.current_usage + connection_usage.get(conn_key, 0)
        if future_conn_usage >= connection.max_capacity:
            return False
        return True

    def apply_move(self, plan: MovePlan,
                   zone_usage: Dict[str, int],
                   connection_usage: Dict[Tuple[str, str], int]) -> None:
        """Apply a single move by updating drone and graph state."""
        drone = plan.drone
        from_zone_name = plan.from_zone
        to_zone_obj = plan.to_zone
        zone_usage[to_zone_obj.name] = zone_usage.get(to_zone_obj.name, 0) + 1
        conn_key = tuple(sorted([from_zone_name, to_zone_obj.name]))
        connection_usage[conn_key] = connection_usage.get(conn_key, 0) + 1
        self.graph.zones[from_zone_name].remove_drone()
        if plan.is_restricted:
            drone.start_restricted_transit(to_zone_obj.name, turns=2)
        else:
            drone.complete_move(to_zone_obj.name)
            to_zone_obj.add_drone()

    def generate_output(self, moves: List[MovePlan]) -> List[str]:
        """Generate the output line for this turn from the executed moves."""
        if not moves:
            return []
        parts: List[str] = []
        for plan in moves:
            parts.append(f"D{plan.drone.drone_id}-{plan.to_zone.name}")
        line = " ".join(parts)
        self.output_lines.append(line)
        return self.output_lines

    def get_turn_count(self) -> int:
        """Return the number of turns executed so far."""
        return self.current_turn

    def print_summary(self) -> None:
        """Print a summary of the simulation results."""
        print("\nSimulation Summary:")
        print(f"  Total turns: {self.current_turn}")
        print(f"  Total drones: {len(self.graph.drones)}")
        print(f"  Output lines: {len(self.output_lines)}")

    def find_dynamic_path(self, start: str, goal: str) -> Optional[List[str]]:
        """
        Compute a path between two zones with an occupancy-based penalty.

        This is a modified Dijkstra where entering a zone includes a congestion penalty
        proportional to its current occupancy, scaled by `occupancy_weight`.
        """
        if start not in self.graph.zones or goal not in self.graph.zones:
            raise ValueError(f"Unknown start or goal zone: {start}, {goal}")
        if start == goal:
            return [start]
        import heapq
        pq: List[Tuple[float, str]] = [(0.0, start)]
        distances: Dict[str, float] = {start: 0.0}
        previous: Dict[str, Optional[str]] = {start: None}
        visited: Set[str] = set()
        # عدد الطائرات فى حالة transit إلى كل منطقة (لاستخدامها فى العقوبة)
        transit_counts: Dict[str, int] = {}
        for d in self.graph.drones:
            if d.status == DroneStatus.IN_TRANSIT and d.transit_destination is not None:
                transit_counts[d.transit_destination] = transit_counts.get(d.transit_destination, 0) + 1
        while pq:
            current_dist, current_zone = heapq.heappop(pq)
            if current_zone in visited:
                continue
            visited.add(current_zone)
            if current_zone == goal:
                # إعادة بناء المسار
                path: List[str] = []
                node: Optional[str] = goal
                while node is not None:
                    path.append(node)
                    node = previous.get(node)
                path.reverse()
                return path
            zone_obj = self.graph.zones[current_zone]
            for neighbor_zone, _ in self.graph.get_neighbors(zone_obj):
                neighbor_name = neighbor_zone.name
                if neighbor_name in visited:
                    continue
                if neighbor_zone.is_blocked:
                    continue
                base_cost = self.graph.get_zone_cost(neighbor_zone)
                penalty = 0.0
                if not neighbor_zone.is_unlimited_capacity:
                    occ = neighbor_zone.current_occupancy + transit_counts.get(neighbor_name, 0)
                    # عقوبة الازدحام = نسبة الإشغال × الوزن
                    penalty = (occ / neighbor_zone.max_capacity if neighbor_zone.max_capacity > 0 else occ) \
                              * self.occupancy_weight
                new_dist = current_dist + base_cost + penalty
                if neighbor_name not in distances or new_dist < distances[neighbor_name]:
                    distances[neighbor_name] = new_dist
                    previous[neighbor_name] = current_zone
                    heapq.heappush(pq, (new_dist, neighbor_name))
        return None
