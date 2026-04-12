"""Drone model: defines drone state, movement, and transit mechanics."""
from typing import List
from enum import Enum
from typing import Optional
from .zone import Zone


class DroneStatus(Enum):
    """Enumeration of possible drone states during simulation."""

    WAITING = "waiting"
    FLYING = "flying"
    IN_TRANSIT = "in_transit"
    REACHED = "reached"


class Drone:
    """Represents a single drone navigating the zone network.

    Attributes:
        drone_id: Unique numeric identifier for this drone.
        current_zone: The zone this drone currently occupies.
        drone_status: Current movement state of the drone.
        target_zone: Destination zone when in restricted transit.
        path: Precomputed path as a list of zones to follow.
        path_index: Current position index along the path.
        wait_turns: Number of consecutive turns this drone has been blocked.
        turns_remaining: Turns left to complete a restricted zone transit.
    """

    def __init__(self,
                 drone_id: int,
                 current_zone: Zone
                 ) -> None:
        """Initialize a Drone at a starting zone.

        Args:
            drone_id: Unique identifier for this drone.
            current_zone: The zone where the drone starts.
        """
        self.drone_id = drone_id
        self.current_zone = current_zone
        self.drone_status = DroneStatus.WAITING
        self.target_zone: Optional[Zone] = None

        self.path: List[Zone] = []
        self.path_index: int = 0
        self.wait_turns: int = 0
        self.turns_remaining: int = 0

    @property
    def is_reached(self) -> bool:
        """Return True if this drone has reached the end hub."""
        return self.drone_status == DroneStatus.REACHED

    @property
    def can_move(self) -> bool:
        """Return True if this drone is available for a move this turn."""
        return self.drone_status == DroneStatus.WAITING

    def start_restricted_transit(self, target: Zone) -> None:
        """Begin a 2-turn transit toward a restricted zone.

        Args:
            target: The restricted zone destination.
        """
        self.drone_status = DroneStatus.IN_TRANSIT
        self.target_zone = target
        self.turns_remaining = 2

    def complete_move(self, next_zone: Zone) -> None:
        """Finish moving to a zone and reset transit state.

        Args:
            next_zone: The zone the drone has arrived at.
        """
        self.current_zone = next_zone
        self.drone_status = DroneStatus.WAITING
        self.target_zone = None
        self.turns_remaining = 0

    def progress_transit(self) -> None:
        """Advance one turn of restricted transit and auto-complete if done."""
        if self.turns_remaining > 0:
            self.turns_remaining -= 1
        if self.turns_remaining == 0 and self.target_zone is not None:
            self.complete_move(self.target_zone)

    def mark_reached(self) -> None:
        """Mark this drone as having reached the end hub."""
        self.drone_status = DroneStatus.REACHED
        self.target_zone = None
        self.turns_remaining = 0
