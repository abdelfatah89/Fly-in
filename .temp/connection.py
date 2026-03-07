from typing import Optional , Tuple


class Connection:
    def __init__(self, zone1: str, zone2: str, max_capacity: int):
        self.zone1 = zone1
        self.zone2 = zone2
        self.max_capacity = max_capacity
        self.name = f"{self.zone1}-{self.zone2}"
        self.current_usage: int = 0
        self._key: Optional[Tuple[str, str]] = None

    # Check if a zone is connected to this connection
    def connects(self, zone_name: str) -> bool:
        return zone_name in (self.zone1, self.zone2)

    # Get the other zone connected to this connection
    def get_other_zone(self, zone_name: str) -> str:
        if zone_name == self.zone1.name:
            return self.zone2
        elif zone_name == self.zone2.name:
            return self.zone1
        else:
            print(
                f"Zone '{zone_name}' is not part of connection "
                f"{self.zone1_name}-{self.zone2_name}"
                )
            return False

    # Get the zones connected to this connection
    def get_zones(self) -> Tuple[str, str]:
        return (self.zone1_name, self.zone2_name)

    # Check if there is capacity left on this connection
    @property
    def has_capacity(self) -> bool:
        return self.current_usage < self.max_capacity

    # Use capacity on this connection
    def use_capacity(self) -> bool:
        if not self.has_capacity:
            return False
        self.current_usage -= 1
        return True

    # Reset the usage of this connection
    def reset_usage(self) -> None:
        self.current_usage = 0

    # Get a canonical (sorted) tuple of the two zone names
    @property
    def key(self) -> Tuple[str, str]:
        """Return a canonical (sorted) tuple of the two zone names."""
        if self._key is None:
            self._key = tuple(sorted([self.zone1, self.zone2]))
        return self._key
