"""Drone model - represents a drone entity in the simulation."""

from enum import Enum
from typing import Optional


class DroneState(Enum):
    """States a drone can be in during simulation."""

    IDLE = "idle"               # Waiting at a zone
    MOVING = "moving"           # Moving to adjacent zone (normal/priority)
    IN_TRANSIT = "in_transit"   # In transit through restricted zone
    DELIVERED = "delivered"     # Reached goal zone


class Drone:
    """Represents a drone that needs to navigate from start to end.

    Each drone tracks its current position, state, and movement intentions.
    """

    def __init__(self, drone_id: int, start_zone_name: str) -> None:
        """Initialize a drone.

        Args:
            drone_id: Unique identifier for this drone
            start_zone_name: Name of the starting zone
        """
        self.drone_id: int = drone_id
        self.current_zone: str = start_zone_name
        self.state: DroneState = DroneState.IDLE

        # Next zone drone intends to move to (None if staying)
        self.target_zone: Optional[str] = None

        # For restricted zones: turns remaining in transit
        self.transit_turns_remaining: int = 0

        # For restricted zones: the zone we're transiting to
        self.transit_destination: Optional[str] = None

    def is_delivered(self) -> bool:
        """Check if drone has reached the goal.

        Returns:
            True if drone is delivered
        """
        return self.state == DroneState.DELIVERED

    def can_move(self) -> bool:
        """Check if drone can initiate a new movement.

        Returns:
            True if drone is idle and can move
        """
        return self.state == DroneState.IDLE

    def start_normal_move(self, target_zone: str) -> None:
        """Start a normal movement to an adjacent zone.

        Args:
            target_zone: Name of zone to move to
        """
        self.state = DroneState.MOVING
        self.target_zone = target_zone

    def start_restricted_transit(self, target_zone: str, turns: int) -> None:
        """Start transit through a restricted zone.

        Args:
            target_zone: Name of restricted zone to transit through
            turns: Number of turns the transit takes
        """
        self.state = DroneState.IN_TRANSIT
        self.transit_destination = target_zone
        self.transit_turns_remaining = turns

    def complete_move(self, new_zone: str) -> None:
        """Complete a movement to a new zone.

        Args:
            new_zone: Name of zone drone has moved to
        """
        self.current_zone = new_zone
        self.state = DroneState.IDLE
        self.target_zone = None

    def complete_transit(self) -> None:
        """Complete a restricted zone transit."""
        if self.transit_destination is None:
            raise ValueError("No transit destination set")

        self.current_zone = self.transit_destination
        self.state = DroneState.IDLE
        self.transit_destination = None
        self.transit_turns_remaining = 0

    def progress_transit(self) -> None:
        """Advance transit by one turn (for restricted zones)."""
        if self.transit_turns_remaining > 0:
            self.transit_turns_remaining -= 1

        # If transit is complete, update position
        if self.transit_turns_remaining == 0:
            self.complete_transit()

    def mark_delivered(self) -> None:
        """Mark this drone as delivered to the goal."""
        self.state = DroneState.DELIVERED
        self.target_zone = None
        self.transit_destination = None
        self.transit_turns_remaining = 0

    def stay_idle(self) -> None:
        """Explicitly keep drone idle (no movement this turn)."""
        self.state = DroneState.IDLE
        self.target_zone = None

    def __str__(self) -> str:
        """String representation of the drone."""
        return (
            f"Drone(D{self.drone_id}, zone={self.current_zone}, "
            f"state={self.state.value})"
        )

    def __repr__(self) -> str:
        """Detailed representation of the drone."""
        extra = ""
        if self.target_zone:
            extra = f", target={self.target_zone}"
        if self.transit_destination:
            extra = (
                f", transit_to={self.transit_destination}, "
                f"turns_left={self.transit_turns_remaining}"
            )
        return (
            f"Drone(id={self.drone_id}, zone='{self.current_zone}', "
            f"state={self.state.value}{extra})"
        )
