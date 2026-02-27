from enum import Enum
from typing import Optional, List
from zone import Zone


class DroneStatus(Enum):
    WAITING = "waiting"
    FLYING = "flying"
    REACHED = "reached"


class Drone:
    def __init__(self, drone_id: str, current_zone: Optional[Zone]):
        self.drone_id = drone_id
        self.current_zone = current_zone
        self.destination_zone: Optional[Zone] = None
        self.status = DroneStatus.WAITING
        self.available_connections: List[str] = []

    def set_destination(self, destination_zone: Zone):
        self.destination_zone = destination_zone
        self.status = DroneStatus.FLYING
