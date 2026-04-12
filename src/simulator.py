"""Simulation engine: turn-based drone movement with capacity enforcement."""
from typing import List, Optional, Dict, Tuple, Set
from .path_finding import Dijkstra
from .graph import Graph
from .drone import Drone, DroneStatus
from .zone import Zone, ZoneType
from .connection import Connection


class MovePlan:
    """Represents a planned drone movement for a single turn.

    Attributes:
        drone: The drone that intends to move.
        from_zone: The zone the drone is currently in.
        to_zone: The destination zone.
        connection: The connection used for this move.
        is_restricted: True if the destination is a restricted zone.
    """

    def __init__(self,
                 drone: Drone,
                 from_zone: Zone,
                 to_zone: Zone,
                 connection: Connection,
                 is_restricted: bool = False) -> None:
        """Initialize a move plan.

        Args:
            drone: The drone to move.
            from_zone: Current zone of the drone.
            to_zone: Target zone for the move.
            connection: The connection linking the two zones.
            is_restricted: Whether the target is a restricted zone.
        """
        self.drone = drone
        self.from_zone = from_zone
        self.to_zone = to_zone
        self.connection = connection
        self.is_restricted = is_restricted


class Simulator:
    """Orchestrates the turn-based drone delivery simulation.

    Handles path assignment, move scheduling, capacity enforcement,
    conflict resolution, and dynamic rerouting.

    Attributes:
        graph: The zone network graph.
        path_finder: The Dijkstra pathfinding engine.
        current_turn: The current simulation turn number.
        output_lines: Accumulated output lines for each turn.
    """

    def __init__(self, graph: Graph) -> None:
        """Initialize the simulator with a graph and precompute paths.

        Args:
            graph: The zone network graph containing drones and zones.
        """
        self.graph = graph
        self.path_finder = Dijkstra(graph)
        self.current_turn: int = 0
        self.output_lines: List[str] = []
        self._assign_initial_paths()

    @staticmethod
    def _path_transit_time(path: List[Zone]) -> int:
        """Calculate the total transit time for a path.

        Args:
            path: Ordered list of zones in the path.

        Returns:
            Total number of turns to traverse the path.
        """
        return sum(2 if z.zone_type == ZoneType.RESTRICTED else 1
                   for z in path[1:])

    def _compute_diverse_paths(
        self, k: int = 5
    ) -> List[List[Zone]]:
        """Compute up to k diverse paths from start to goal.

        Iteratively forbids bottleneck zones from previous paths
        to discover alternative routes.

        Args:
            k: Maximum number of diverse paths to find.

        Returns:
            List of distinct paths, each as a list of Zone objects.
        """
        paths: List[List[Zone]] = []
        forbidden: Set[str] = set()
        start = self.graph.start_hub
        goal = self.graph.end_hub
        for _ in range(k):
            path = self.path_finder.find_path(
                start, goal,
                forbidden_zones=forbidden if forbidden else None)
            if not path or len(path) < 2:
                break
            paths.append(path)
            prev_size = len(forbidden)
            for z in path[1:-1]:
                if (z.zone_type == ZoneType.RESTRICTED
                        and z.max_capacity <= 1):
                    forbidden.add(z.name)
            if len(forbidden) == prev_size:
                break
        if len(paths) > 1:
            times = [self._path_transit_time(p) for p in paths]
            min_time = min(times)
            paths = [p for p, t in zip(paths, times)
                     if t <= min_time + 1]
        return paths

    def _assign_initial_paths(self) -> None:
        """Assign precomputed diverse paths to drones in round-robin."""
        paths = self._compute_diverse_paths()
        if not paths:
            print("No paths found for drones.")
            exit(1)

        for i, drone in enumerate(self.graph.drones):
            drone.path = list(paths[i % len(paths)])
            drone.path_index = 0

    def all_delivered(self) -> bool:
        """Return True if all drones have reached the end hub."""
        goal_zone = self.graph.end_hub
        return all(d.current_zone.name == goal_zone.name
                   for d in self.graph.drones)

    def _progress_transit(self) -> None:
        """Advance all in-transit drones by one turn."""
        for drone in self.graph.drones:
            was_in_transit = (
                drone.drone_status == DroneStatus.IN_TRANSIT)
            if was_in_transit:
                drone.progress_transit()
                if drone.drone_status == DroneStatus.WAITING:
                    if drone.current_zone != self.graph.end_hub:
                        drone.current_zone.add_drone()
                    drone.path_index += 1

    def get_next_path_for_drone(
        self, drone: Drone
    ) -> Optional[List[Zone]]:
        """Get or recompute the path for a drone.

        If the drone's cached path is still valid, returns it.
        Otherwise, computes a new path from the drone's current
        position to the end hub.

        Args:
            drone: The drone that needs a path.

        Returns:
            The path as a list of zones, or None if unreachable.
        """
        if (drone.path
            and drone.path_index < len(drone.path)
            and drone.path[drone.path_index] == drone.current_zone
                and drone.path_index < len(drone.path) - 1):
            return drone.path

        path = self.path_finder.find_path(
            drone.current_zone, self.graph.end_hub)
        if not path:
            return None

        drone.path = path
        drone.path_index = 0
        return path

    def collect_move_intentions(self) -> List[MovePlan]:
        """Gather intended moves from all available drones.

        Returns:
            List of MovePlan objects for drones that want to move.
        """
        plans: List[MovePlan] = []
        goal = self.graph.end_hub

        for drone in self.graph.drones:
            if not drone.can_move:
                continue
            if drone.current_zone.name == goal.name:
                continue

            path = self.get_next_path_for_drone(drone)
            if (path is None
                    or drone.path_index >= len(path) - 1):
                continue

            next_zone = path[drone.path_index + 1]
            is_restricted = (
                next_zone.zone_type == ZoneType.RESTRICTED)
            connection = self.graph.get_connection(
                drone.current_zone, next_zone)
            if connection is None:
                continue

            plan = MovePlan(drone,
                            drone.current_zone,
                            next_zone,
                            connection,
                            is_restricted)
            plans.append(plan)
        return plans

    def should_reroute(self, drone: Drone) -> bool:
        """Return True if a blocked drone should attempt rerouting.

        Args:
            drone: The blocked drone.

        Returns:
            True if the drone has waited 1+ turns and should reroute.
        """
        return drone.wait_turns >= 1

    def reroute_drone(
        self,
        drone: Drone,
        forbidden_zones: Optional[Set[str]] = None,
        forbidden_connection:
            Optional[Set[Tuple[str, str]]] = None
    ) -> bool:
        """Attempt to find an alternative path for a blocked drone.

        Args:
            drone: The drone to reroute.
            forbidden_zones: Zone names to avoid in the new path.
            forbidden_connection: Connection keys to avoid.

        Returns:
            True if a valid alternative path was found.
        """
        new_path = self.path_finder.find_path(
            drone.current_zone,
            self.graph.end_hub,
            forbidden_zones=forbidden_zones,
            forbidden_connections=forbidden_connection)
        if not new_path or len(new_path) < 2:
            return False
        drone.path = new_path
        drone.path_index = 0
        drone.wait_turns = 0
        return True

    def can_apply_plan(
        self,
        plan: MovePlan,
        zone_usage: Dict[str, int],
        connection_usage: Dict[Tuple[str, str], int]
    ) -> bool:
        """Check whether a move plan can be executed without violations.

        Verifies zone capacity and connection capacity constraints.

        Args:
            plan: The proposed move.
            zone_usage: Accumulated zone usage for this turn.
            connection_usage: Accumulated connection usage for this turn.

        Returns:
            True if the plan can be applied without constraint violations.
        """
        to_zone = plan.to_zone
        drones_in_transit = sum(
            1 for d in self.graph.drones
            if d.drone_status == DroneStatus.IN_TRANSIT
            and d.target_zone == to_zone)

        expected_occupancy = (to_zone.current_occupancy
                              + drones_in_transit
                              + zone_usage.get(to_zone.name, 0))

        if to_zone not in (self.graph.end_hub,
                           self.graph.start_hub):
            if expected_occupancy >= to_zone.max_capacity:
                return False

        connection = plan.connection
        connection_key: Tuple[str, str] = connection.key
        expected_connection_usage = (
            connection.current_usage
            + connection_usage.get(connection_key, 0))
        if expected_connection_usage >= connection.max_link_capacity:
            return False

        return True

    def apply_plan(
        self,
        plan: MovePlan,
        zone_usage: Dict[str, int],
        connection_usage: Dict[Tuple[str, str], int]
    ) -> None:
        """Execute a validated move plan and update state.

        Args:
            plan: The move plan to apply.
            zone_usage: Accumulated zone usage to update.
            connection_usage: Accumulated connection usage to update.
        """
        drone = plan.drone
        connection = plan.connection
        _from = plan.from_zone
        _to = plan.to_zone

        connection_usage[connection.key] = (
            connection_usage.get(connection.key, 0) + 1)

        if _to not in (self.graph.end_hub,
                       self.graph.start_hub):
            zone_usage[_to.name] = (
                zone_usage.get(_to.name, 0) + 1)

        if _from not in (self.graph.end_hub,
                         self.graph.start_hub):
            _from.remove_drone()

        if plan.is_restricted:
            drone.start_restricted_transit(_to)
        else:
            drone.complete_move(_to)
            drone.path_index += 1
            if _to not in (self.graph.end_hub,
                           self.graph.start_hub):
                _to.add_drone()

    def validate_and_apply_moves(
        self, plans: List[MovePlan]
    ) -> List[MovePlan]:
        """Validate and apply all move plans with priority ordering.

        Drones closest to the goal are prioritized. Blocked drones
        may trigger rerouting.

        Args:
            plans: List of proposed moves for this turn.

        Returns:
            List of successfully applied move plans.
        """
        start = self.graph.start_hub
        plans.sort(key=lambda p: float('inf')
                   if p.drone.current_zone == start
                   else (len(p.drone.path) - p.drone.path_index))
        zone_usage: Dict[str, int] = {}
        connection_usage: Dict[Tuple[str, str], int] = {}
        successful_plans: List[MovePlan] = []
        for plan in plans:
            if self.can_apply_plan(
                    plan, zone_usage, connection_usage):
                self.apply_plan(
                    plan, zone_usage, connection_usage)
                plan.drone.wait_turns = 0
                successful_plans.append(plan)
            else:
                plan.drone.wait_turns += 1
                if self.should_reroute(plan.drone):
                    self.reroute_drone(
                        plan.drone,
                        forbidden_zones={plan.to_zone.name},
                        forbidden_connection={
                            plan.connection.key})
        return successful_plans

    def generate_output_line(
        self, plans: List[MovePlan]
    ) -> str:
        """Format the moves for this turn as a simulation output line.

        Args:
            plans: List of successfully applied moves.

        Returns:
            Space-separated move strings in D<ID>-<zone> format.
        """
        if not plans:
            return ""
        move_strs: List[str] = []
        for plan in plans:
            if plan.is_restricted:
                move_strs.append(
                    f"D{plan.drone.drone_id}"
                    f"-{plan.connection.name}")
            else:
                move_strs.append(
                    f"D{plan.drone.drone_id}"
                    f"-{plan.to_zone.name}")

        line = " ".join(move_strs)
        self.output_lines.append(line)
        return line

    def execute_turn(self) -> str:
        """Execute a single simulation turn.

        Returns:
            The output line for this turn's moves.
        """
        for conn in self.graph.connections:
            conn.reset_usage()
        self._progress_transit()
        move_plans = self.collect_move_intentions()
        successful_plans = self.validate_and_apply_moves(
            move_plans)
        line = self.generate_output_line(successful_plans)
        return line

    def run(self, max_turns: int = 100) -> None:
        """Run the complete simulation until all drones are delivered.

        Args:
            max_turns: Safety limit to prevent infinite loops.
        """
        try:
            self.output_lines = []
            self.current_turn = 0

            while not self.all_delivered():
                if self.current_turn >= max_turns:
                    raise RuntimeError(
                        f"Exceeded maximum turns ({max_turns})"
                        " without delivering all drones"
                    )
                self.execute_turn()
                self.current_turn += 1

            print("#", "=" * 80)
            for line in self.output_lines:
                print(line)
            print("#", "=" * 80)
            print("#", "All drones delivered in",
                  len(self.output_lines), "turns")
            print("#", "=" * 80)

        except Exception as e:
            print(f"Error during simulation: {e}")
            print("No Path Found for some drones."
                  " Simulation terminated.")
