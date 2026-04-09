from typing import Optional, Tuple
from .zone import Zone


class Connection:
    def __init__(self,
                 zone1: str,
                 zone2: str,
                 max_link_capacity: int
                 ) -> None:
        # ------- init -------- #
        self.zone1 = zone1
        self.zone2 = zone2
        self.max_link_capacity = max_link_capacity
        self.name = f"{self.zone1}-{self.zone2}"
        self.current_usage: int = 0
        self._key: Optional[Tuple[str, str]] = None

    def is_connected(self, zone: Zone) -> bool:
        return zone.name in (self.zone1, self.zone2)

    def get_other_zone(self, zone: Zone) -> Optional[str]:
        if zone.name == self.zone1:
            return self.zone2
        if zone.name == self.zone2:
            return self.zone1
        return None

    @property
    def zones(self) -> Tuple[str, str]:
        return (self.zone1, self.zone2)

    @property
    def has_capacity(self) -> bool:
        return self.current_usage < self.max_link_capacity

    def use_capacity(self) -> bool:
        if not self.has_capacity:
            return False
        self.current_usage += 1
        return True

    def reset_usage(self) -> None:
        self.current_usage = 0

    @property
    def key(self) -> Tuple[str, str]:
        if self._key is None:
            sorted_zones = sorted([self.zone1, self.zone2])
            self._key = (sorted_zones[0], sorted_zones[1])
        return self._key
