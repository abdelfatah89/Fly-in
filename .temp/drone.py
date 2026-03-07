from enum import Enum
from typing import Optional
from zone import Zone


class DroneStatus(Enum):
    WAITING = "waiting"
    FLYING = "flying"
    IN_TRANSIT = "in_transit"
    REACHED = "reached"


class Drone:
    def __init__(self, drone_id: str, current_zone: Optional[Zone]):
        self.drone_id = drone_id
        self.current_zone = current_zone
        self.status = DroneStatus.WAITING
        
        self.target_zone: Optional[Zone] = None

        # For restricted zones: turns remaining in transit
        self.transit_turns_remaining: int = 0
        # For restricted zones: the zone we're transiting to
        self.transit_destination: Optional[str] = None

    # Check if the drone has reached its target zone
    def is_reached(self) -> bool:
        return self.status == DroneStatus.REACHED

    # Check if the drone can move
    def can_move(self):
        return self.status == DroneStatus.WAITING

    # Start a restricted transit
    def start_restricted_transit(self, target_zone: str, turns: int):
        self.status = DroneStatus.IN_TRANSIT
        self.transit_destination = target_zone
        self.transit_turns_remaining = turns

    # Complete a move
    def complete_move(self, new_zone: str) -> None:
        if self.transit_destination is None:
            raise ValueError("No transit destination set")
        self.current_zone = new_zone
        self.status = DroneStatus.WAITING
        self.target_zone = None
        self.transit_turns_remaining = 0

    # Progress the transit
    def progress_transit(self) -> None:
        if self.transit_turns_remaining > 0:
            self.transit_turns_remaining -= 1
        if self.transit_turns_remaining == 0:
            self.complete_move()

    # Mark the drone as delivered
    def mark_delivered(self) -> None:
        self.status = DroneStatus.REACHED
        self.target_zone = None
        self.transit_destination = None
        self.transit_turns_remaining = 0

    # Mark the drone as waiting
    def stay_waiting(self) -> None:
        self.status = DroneStatus.WAITING
        self.target_zone = None
