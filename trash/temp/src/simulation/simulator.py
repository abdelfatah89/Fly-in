"""Simulation engine - turn-by-turn drone movement logic."""

from typing import List, Dict, Tuple
from ..models import MapData, Drone, DroneState, ZoneType
from ..pathfinding import Pathfinder


class MovementPlan:
    """Represents a planned movement for a drone."""

    def __init__(
        self,
        drone: Drone,
        from_zone: str,
        to_zone: str,
        is_restricted: bool = False
    ) -> None:
        """Initialize a movement plan.

        Args:
            drone: The drone to move
            from_zone: Current zone
            to_zone: Target zone
            is_restricted: Whether this is a restricted zone movement
        """
        self.drone = drone
        self.from_zone = from_zone
        self.to_zone = to_zone
        self.is_restricted = is_restricted


class Simulator:
    """Simulates drone navigation turn by turn.

    The simulator manages:
    - Pathfinding for all drones
    - Turn-by-turn movement execution
    - Capacity constraint validation
    - Output generation
    """

    def __init__(self, map_data: MapData) -> None:
        """Initialize simulator with map data.

        Args:
            map_data: The map to simulate on
        """
        self.map_data = map_data
        self.pathfinder = Pathfinder(map_data)

        # Precompute paths for all drones
        self.drone_paths: Dict[int, List[str]] = {}
        self.drone_path_index: Dict[int, int] = {}

        # Turn counter
        self.current_turn: int = 0

        # Output lines (one per turn)
        self.output_lines: List[str] = []

        # Initialize paths
        self._compute_paths()

    def _compute_paths(self) -> None:
        """Compute paths for all drones from start to goal."""
        start = self.map_data.start_zone_name
        goal = self.map_data.end_zone_name

        if start is None or goal is None:
            raise ValueError("Start or goal zone not defined")

        # Find path (same for all drones)
        path = self.pathfinder.find_path(start, goal)

        if path is None:
            raise ValueError(f"No path exists from {start} to {goal}")

        # Assign paths to all drones
        for drone in self.map_data.drones:
            self.drone_paths[drone.drone_id] = path.copy()
            self.drone_path_index[drone.drone_id] = 0

    def run(self, max_turns: int = 1000) -> List[str]:
        """Run the simulation until all drones are delivered.

        Args:
            max_turns: Maximum number of turns to simulate

        Returns:
            List of output lines (one per turn)
        """
        self.current_turn = 0
        self.output_lines = []

        while not self._all_drones_delivered():
            if self.current_turn >= max_turns:
                raise RuntimeError(f"Simulation exceeded {max_turns} turns")

            self._execute_turn()
            self.current_turn += 1

        return self.output_lines

    def _all_drones_delivered(self) -> bool:
        """Check if all drones have been delivered.

        Returns:
            True if all drones are at the goal
        """
        goal = self.map_data.end_zone_name
        return all(
            drone.current_zone == goal
            for drone in self.map_data.drones
        )

    def _execute_turn(self) -> None:
        """Execute one turn of the simulation.

        Turn execution phases:
        1. Progress restricted zone transits
        2. Collect movement intentions from idle drones
        3. Validate capacity constraints
        4. Apply valid movements
        5. Generate output
        """
        # Phase 1: Progress drones in restricted transit
        self._progress_transits()

        # Phase 2: Collect movement intentions
        movement_plans = self._collect_movement_intentions()

        # Phase 3: Validate and apply movements
        successful_movements = self._validate_and_apply_movements(movement_plans)

        # Phase 4: Generate output
        self._generate_output(successful_movements)

    def _progress_transits(self) -> None:
        """Progress all drones that are in restricted zone transit."""
        for drone in self.map_data.drones:
            if drone.state == DroneState.IN_TRANSIT:
                drone.progress_transit()

                # If transit complete, update zone occupancy
                if drone.state == DroneState.IDLE:
                    # Drone has arrived at destination
                    zone = self.map_data.zones[drone.current_zone]
                    zone.add_drone()

    def _collect_movement_intentions(self) -> List[MovementPlan]:
        """Collect movement intentions from all idle drones.

        Returns:
            List of planned movements
        """
        plans: List[MovementPlan] = []

        for drone in self.map_data.drones:
            # Skip if not idle
            if not drone.can_move():
                continue

            # Skip if already delivered
            goal = self.map_data.end_zone_name
            if drone.current_zone == goal:
                continue

            # Get next zone on path
            path = self.drone_paths[drone.drone_id]
            current_index = self.drone_path_index[drone.drone_id]

            # If at end of path, mark as delivered
            if current_index >= len(path) - 1:
                drone.mark_delivered()
                continue

            next_zone_name = path[current_index + 1]
            next_zone = self.map_data.zones[next_zone_name]

            # Check if next zone is restricted
            is_restricted = (next_zone.zone_type == ZoneType.RESTRICTED)

            # Create movement plan
            plan = MovementPlan(
                drone=drone,
                from_zone=drone.current_zone,
                to_zone=next_zone_name,
                is_restricted=is_restricted
            )
            plans.append(plan)

        return plans

    def _validate_and_apply_movements(
        self,
        plans: List[MovementPlan]
    ) -> List[MovementPlan]:
        """Validate capacity constraints and apply valid movements.

        Args:
            plans: List of movement plans to validate

        Returns:
            List of successfully applied movements
        """
        # Track capacity usage for this turn
        zone_usage: Dict[str, int] = {}
        connection_usage: Dict[Tuple[str, str], int] = {}

        successful: List[MovementPlan] = []

        # Simple greedy approach: try to apply each movement
        for plan in plans:
            if self._can_apply_movement(plan, zone_usage, connection_usage):
                self._apply_movement(plan, zone_usage, connection_usage)
                successful.append(plan)

        return successful

    def _can_apply_movement(
        self,
        plan: MovementPlan,
        zone_usage: Dict[str, int],
        connection_usage: Dict[Tuple[str, str], int]
    ) -> bool:
        """Check if a movement plan can be applied.

        Args:
            plan: Movement plan to check
            zone_usage: Current zone usage this turn
            connection_usage: Current connection usage this turn

        Returns:
            True if movement is valid
        """
        to_zone = self.map_data.zones[plan.to_zone]

        # Check zone capacity
        future_occupancy = to_zone.current_occupancy + zone_usage.get(plan.to_zone, 0)
        if not to_zone.is_unlimited_capacity:
            if future_occupancy >= to_zone.max_capacity:
                return False

        # Check connection capacity
        connection = self.map_data.get_connection(plan.from_zone, plan.to_zone)
        if connection is None:
            return False  # No connection exists

        conn_key = tuple(sorted([plan.from_zone, plan.to_zone]))
        future_conn_usage = connection.current_usage + connection_usage.get(conn_key, 0)
        if future_conn_usage >= connection.max_capacity:
            return False

        return True

    def _apply_movement(
        self,
        plan: MovementPlan,
        zone_usage: Dict[str, int],
        connection_usage: Dict[Tuple[str, str], int]
    ) -> None:
        """Apply a movement plan.

        Args:
            plan: Movement plan to apply
            zone_usage: Zone usage tracking
            connection_usage: Connection usage tracking
        """
        drone = plan.drone
        from_zone_obj = self.map_data.zones[plan.from_zone]
        to_zone_obj = self.map_data.zones[plan.to_zone]

        # Update usage tracking
        zone_usage[plan.to_zone] = zone_usage.get(plan.to_zone, 0) + 1
        conn_key = tuple(sorted([plan.from_zone, plan.to_zone]))
        connection_usage[conn_key] = connection_usage.get(conn_key, 0) + 1

        # Remove from current zone
        from_zone_obj.remove_drone()

        if plan.is_restricted:
            # Start restricted transit (2 turns)
            drone.start_restricted_transit(plan.to_zone, turns=2)
        else:
            # Immediate arrival for normal/priority zones
            drone.complete_move(plan.to_zone)
            to_zone_obj.add_drone()

        # Update path index
        self.drone_path_index[drone.drone_id] += 1

    def _generate_output(self, movements: List[MovementPlan]) -> None:
        """Generate output line for this turn.

        Args:
            movements: List of movements that occurred
        """
        if not movements:
            # No movements this turn - skip output line
            return

        # Format: D<ID>-<zone> D<ID>-<zone> ...
        parts: List[str] = []

        for plan in movements:
            drone_id = plan.drone.drone_id
            zone = plan.to_zone
            parts.append(f"D{drone_id}-{zone}")

        output_line = " ".join(parts)
        self.output_lines.append(output_line)

    def get_turn_count(self) -> int:
        """Get the number of turns taken.

        Returns:
            Number of simulation turns
        """
        return self.current_turn

    def print_summary(self) -> None:
        """Print a summary of the simulation results."""
        print("\nSimulation Summary:")
        print(f"  Total turns: {self.current_turn}")
        print(f"  Total drones: {len(self.map_data.drones)}")
        print(f"  Output lines: {len(self.output_lines)}")

        path = self.drone_paths[0]
        print("  Path used: " + ' -> '.join(path))
