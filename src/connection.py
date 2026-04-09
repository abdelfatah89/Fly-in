"""Connection model: defines bidirectional links between zones."""
from typing import Optional, Tuple
from .zone import Zone


class Connection:
    """Represents a bidirectional connection (edge) between two zones.

    Attributes:
        zone1: Name of the first connected zone.
        zone2: Name of the second connected zone.
        max_link_capacity: Maximum drones that can traverse per turn.
        name: Display name in 'zone1-zone2' format.
        current_usage: Number of drones using this connection this turn.
    """

    def __init__(self,
                 zone1: str,
                 zone2: str,
                 max_link_capacity: int
                 ) -> None:
        """Initialize a Connection between two zones.

        Args:
            zone1: Name of the first zone.
            zone2: Name of the second zone.
            max_link_capacity: Maximum simultaneous traversals per turn.
        """
        self.zone1 = zone1
        self.zone2 = zone2
        self.max_link_capacity = max_link_capacity
        self.name = f"{self.zone1}-{self.zone2}"
        self.current_usage: int = 0
        self._key: Optional[Tuple[str, str]] = None

    def is_connected(self, zone: Zone) -> bool:
        """Check if the zone is an endpoint of this connection."""
        return zone.name in (self.zone1, self.zone2)

    def get_other_zone(self, zone: Zone) -> Optional[str]:
        """Return the name of the zone on the other end of this connection.

        Args:
            zone: One endpoint zone.

        Returns:
            Name of the other zone, or None if the zone is not an endpoint.
        """
        if zone.name == self.zone1:
            return self.zone2
        if zone.name == self.zone2:
            return self.zone1
        return None

    @property
    def zones(self) -> Tuple[str, str]:
        """Return a tuple of the two connected zone names."""
        return (self.zone1, self.zone2)

    @property
    def has_capacity(self) -> bool:
        """Check if this link can accept another traversal."""
        return self.current_usage < self.max_link_capacity

    def use_capacity(self) -> bool:
        """Consume one unit of link capacity for this turn.

        Returns:
            True if capacity was available and consumed, False otherwise.
        """
        if not self.has_capacity:
            return False
        self.current_usage += 1
        return True

    def reset_usage(self) -> None:
        """Reset the per-turn usage counter to zero."""
        self.current_usage = 0

    @property
    def key(self) -> Tuple[str, str]:
        """Return a canonical sorted tuple key for duplicate detection."""
        if self._key is None:
            sorted_zones = sorted([self.zone1, self.zone2])
            self._key = (sorted_zones[0], sorted_zones[1])
        return self._key
