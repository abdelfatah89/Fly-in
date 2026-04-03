"""Connection model - represents an edge between two zones."""

from typing import Tuple


class Connection:
    """Represents a bidirectional connection (edge) between two zones.

    Connections allow drones to move between zones. Each connection
    has a capacity limiting simultaneous traversals.
    """

    def __init__(
        self,
        zone1_name: str,
        zone2_name: str,
        max_capacity: int = 1
    ) -> None:
        """Initialize a connection.

        Args:
            zone1_name: Name of first zone
            zone2_name: Name of second zone
            max_capacity: Maximum drones that can traverse simultaneously
        """
        self.zone1_name: str = zone1_name
        self.zone2_name: str = zone2_name
        self.max_capacity: int = max_capacity

        # Track current usage for capacity management
        self.current_usage: int = 0

    def connects(self, zone_name: str) -> bool:
        """Check if this connection involves a specific zone.

        Args:
            zone_name: Name of zone to check

        Returns:
            True if zone is one of the connected zones
        """
        return zone_name in (self.zone1_name, self.zone2_name)

    def get_other_zone(self, zone_name: str) -> str:
        """Get the other zone connected by this connection.

        Args:
            zone_name: Name of one zone

        Returns:
            Name of the other zone

        Raises:
            ValueError: If zone_name is not part of this connection
        """
        if zone_name == self.zone1_name:
            return self.zone2_name
        elif zone_name == self.zone2_name:
            return self.zone1_name
        else:
            raise ValueError(
                f"Zone '{zone_name}' is not part of connection "
                f"{self.zone1_name}-{self.zone2_name}"
            )

    def get_zones(self) -> Tuple[str, str]:
        """Get both zones connected by this connection.

        Returns:
            Tuple of (zone1_name, zone2_name)
        """
        return (self.zone1_name, self.zone2_name)

    @property
    def has_capacity(self) -> bool:
        """Check if connection has capacity for another traversal.

        Returns:
            True if capacity is available
        """
        return self.current_usage < self.max_capacity

    def use_capacity(self) -> bool:
        """Use one unit of connection capacity.

        Returns:
            True if capacity was available and used, False otherwise
        """
        if not self.has_capacity:
            return False
        self.current_usage += 1
        return True

    def release_capacity(self) -> bool:
        """Release one unit of connection capacity.

        Returns:
            True if capacity was released, False if usage was already 0
        """
        if self.current_usage <= 0:
            return False
        self.current_usage -= 1
        return True

    def reset_usage(self) -> None:
        """Reset usage count to zero."""
        self.current_usage = 0

    def __str__(self) -> str:
        """String representation of the connection."""
        return (
            f"Connection({self.zone1_name}-{self.zone2_name}, "
            f"usage={self.current_usage}/{self.max_capacity})"
        )

    def __repr__(self) -> str:
        """Detailed representation of the connection."""
        return (
            f"Connection(zone1='{self.zone1_name}', zone2='{self.zone2_name}', "
            f"capacity={self.max_capacity})"
        )
