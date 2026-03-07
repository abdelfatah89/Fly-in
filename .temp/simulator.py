from typing import List, Dict, Tuple
from drone import Drone, DroneStatus
from zone import Zone, ZoneType
from map_parser import Graph
from path_finding import Dijkstra


class MovePlan:
    """Represents a planned move for a drone"""
    def __init__(self,
                 drone: Drone,
                 from_zone: Zone,
                 to_zone: Zone,
                 is_restricted: bool = False
                 ) -> None:
        self.drone = drone
        self.from_zone = from_zone
        self.to_zone = to_zone
        self.is_restricted = is_restricted


class Simulator:
    """Simulates drone navigation turn by turn"""
    def __init__(self, graph: Graph) -> None:
        self.graph = graph
        self.path_finder = Dijkstra(graph)
        # Precompute paths for all drones
        self.drones_path: Dict[str, List[str]] = {}
        self.drones_path_index: Dict[int, int] = {}
        # Turn counter
        self.current_turn: int = 0
        # Output lines (one bu turn)
        self.output_lines: List[str] = []
        # initialisation of paths
        self.init_paths()

    def init_paths(self) -> None:
        start = self.graph.start_zone
        goal = self.graph.end_zone

        if start in None or goal in None:
            raise ValueError("Start or goal zone not defined")

        # Find path (all drones have some path)
        path = self.path_finder.get_path(start, goal)
        if path is None:
            raise ValueError(f"No path exists from {start} to {goal}")

        # Set paths to all drones
        for drone in self.graph.drones:
            self.drones_path[drone.drone_id] = path.copy()
            self.drones_path_index[drone.drone_id] = 0

    def run(self) -> List[str]:
        """Rum simuation until all drones delivered."""
        self.current_turn = 0
        self.output_lines = []

        while not self.all_drones_delivered():
            self.execute_turn()
            self.current_turn += 1
        return self.output_lines
    
    def all_drones_delivered(self) -> bool:
        """Check if all drones have been delivered."""
        goal = self.graph.end_zone
        return all(
            drone.current_zone == goal
            for drone in self.graph.drones
        )
    
    def execute_turn(self) -> None:
        """Execure one turn of the simulation."""
        # Progress drones in restricted transit
        self._progress_transits()
        # Collect moves intentions
        move_plans = self.collect_move_intentions()
        # Validate and apply moves
        successful_move = self.validate_and_apply_moves(move_plans)
        # Generate output
        self.generate_output(successful_move)

    def _progress_transits(self) -> None:
        """Progress all drones are in restricted zone transit"""
        for drone in self.graph.drones:
            if drone.status == DroneStatus.IN_TRANSIT:
                drone.progress_transit()
                if drone.status == DroneStatus.WAITING:
                    zone = self.graph.zones[drone.current_zone]
                    zone.add_drone()

    def collect_move_intentions(self) -> List[MovePlan]:
        """Collect move intentions from all waitting drones."""
        plans: List[MovePlan] = []
        for drone in self.graph.zones:
            # Skip if not in waitting status
            if not drone.can_move():
                continue
            # Skip if already at Goal zone
            goal = self.graph.end_zone
            if drone.current_zone == goal:
                continue

            # Get path and path index of drone to get next zone on path
            path = self.drones_path[drone.drone_id]
            current_index = self.drones_path_index[drone.drone_id]

            # If at the end of path, at goal zone, mark as delivered
            if current_index >= len(path) - 1:
                drone.mark_delivered()
                continue

            # Get name and object of next zone
            next_zone_name = path[current_index + 1]
            next_zone = self.graph.zones[next_zone_name]

            # Check if zone a restricted zone
            is_restricted = (next_zone.zone_type == ZoneType.RESTRICTED)
            # Create a MovePlan for move to the next zone
            plan = MovePlan(
                drone=drone,
                from_zone=drone.current_zone,
                to_zone=next_zone_name,
                is_restricted=is_restricted
            )
            plans.append(plan)

        return plans

    def validate_and_apply_moves(
            self,
            plans: List[MovePlan]
            ) -> List[MovePlan]:
        """Validate capacity constraints and apply valid movements."""
        # Track capacity usage for this turn
        zone_usage: Dict[int, int] = {}
        connection_usage: Dict[Tuple[str, str], int] = {}

        successful: List[MovePlan] = []

        for plan in plans:
            if self.can_apply_plan(plan, zone_usage, connection_usage):
                self.apply_plan(plan, zone_usage, connection_usage)
                successful.append(plan)
        return successful
    
    def can_apply_plan(
            self,
            plan: MovePlan,
            zone_usage: Dict[int, int],
            connection_usage: Dict[Tuple[str, str], int]
    ) -> bool:
        """Check if a movement plan can be applied."""
        # Get to_zone and check future capacity if it not begger then max capacity of zone
        to_zone = self.graph.zones[plan.to_zone]
        future_occupancy = to_zone.current_occupancy + zone_usage.get(plan.to_zone, 0)
        if not to_zone.is_unlimited_capacity:
            if future_occupancy >= to_zone.max_capacity:
                return False
        # Get connection between fron/to zone and check if it exict or not
        connection = self.graph.get_connection(plan.from_zone, plan.to_zone)
        if connection is None:
            return False

        # calcule future usage and check if it not begger then max capacity of connection
        conn_key = tuple(sorted([plan.from_zone, plan.to_zone]))
        future_conn_usage = connection.current_usage + connection_usage.get(conn_key, 0)
        if future_conn_usage >= connection.max_capacity:
            return False

        return True

    def apply_move(
            self,
            plan: MovePlan,
            zone_usage: Dict[int, int],
            connection_usage: Dict[Tuple[str, str], int]
    ) -> None:
        """Apply a movement plan."""
        drone = plan.drone
        from_zone = plan.from_zone
        to_zone = plan.to_zone

        # Get from/to zone object
        from_zone_obj = self.graph.zones[from_zone]
        to_zone_obj = self.graph.zones[to_zone]

        # increment zone/connection usage
        zone_usage[to_zone] = zone_usage.get(to_zone, 0) + 1
        conn_key = tuple(sorted([from_zone, to_zone]))
        connection_usage[conn_key] = connection_usage.get(conn_key, 0) + 1

        # remove current drone from from_zone
        from_zone_obj.remove_drone()

        # Check if next zone is restricted and start restricted transit if it
        # else complete move and mark to_zone as current zone of drone and add it to to_zone object
        if plan.is_restricted:
            drone.start_restricted_transit(to_zone, turns=2)
        else:
            drone.complete_move(to_zone)
            to_zone_obj.add_drone()
        
        # increment path index of drone
        self.drones_path_index[drone.drone_id] += 1

    def generate_output(self, moves: List[MovePlan]) -> None:
        """Generate output line for this turn."""
        # No moves this turn - Skip output line
        if not moves:
            return
        # transform plan to string Format: D<ID>-<zone>
        parts: List[str] = []
        for plan in moves:
            drone_id = plan.drone.drone_id
            zone = plan.to_zone
            parts.append(f"D{drone_id}-{zone}")

        # join part of this move into one str
        output_line = " ".join(parts)
        self.output_lines.append(output_line)

    def get_turn_count(self) -> int:
        """Get the number of turns taken."""
        return self.current_turn
    
    def print_summary(self) -> None:
        """Print a summary of the simulation results."""
        print("\nSimulation Summary:")
        print(f"  Total turns: {self.current_turn}")
        print(f"  Total drones: {len(self.map_data.drones)}")
        print(f"  Output lines: {len(self.output_lines)}")

        path = self.drone_paths[0]
        print("  Path used: " + ' -> '.join(path))
