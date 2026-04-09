from typing import List
from enum import Enum
from typing import Optional
from .zone import Zone


class DroneStatus(Enum):
    WAITING = "waiting"
    FLYING = "flying"
    IN_TRANSIT = "in_transit"
    REACHED = "reached"


class Drone:
    def __init__(self,
                 drone_id: int,
                 current_zone: Zone
                 ) -> None:
        self.drone_id = drone_id
        self.current_zone = current_zone
        self.drone_status = DroneStatus.WAITING
        self.target_zone: Optional[Zone] = None

        # ----- Rerouting -----
        self.path: List[Zone] = []
        self.path_index: int = 0
        self.wait_turns: int = 0
        self.turns_remaining: int = 0

    @property
    def is_reached(self) -> bool:
        return self.drone_status == DroneStatus.REACHED

    @property
    def can_move(self) -> bool:
        return self.drone_status == DroneStatus.WAITING

    def start_restricted_transit(self, target: Zone) -> None:
        self.drone_status = DroneStatus.IN_TRANSIT
        self.target_zone = target
        self.turns_remaining = 2

    def complete_move(self, next_zone: Zone) -> None:
        self.current_zone = next_zone
        self.drone_status = DroneStatus.WAITING
        self.target_zone = None
        self.turns_remaining = 0

    def progress_transit(self) -> None:
        if self.turns_remaining > 0:
            self.turns_remaining -= 1
        if self.turns_remaining == 0 and self.target_zone is not None:
            self.complete_move(self.target_zone)

    def mark_reached(self) -> None:
        self.drone_status = DroneStatus.REACHED
        self.target_zone = None
        self.turns_remaining = 0
