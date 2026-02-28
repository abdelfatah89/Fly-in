from enum import Enum


class ZoneType(Enum):
    """Enum representing the type of a zone and its movement cost."""
    NORMAL = 1      # cost 1 turn
    PRIORITY = 1    # cost 1 turn (but preferred in pathfinding)
    RESTRICTED = 2  # cost 2 turns
    BLOCKED = float('inf')  # inaccessible

    def movement_cost(self) -> float:
        """Return the number of turns required to enter this zone."""
        return self.value


print(type(ZoneType.NORMAL))  # Output: 1
