from typing import Dict, Any, List, Optional, Tuple
from models import Graph, Zone, Connection, ZoneType


class MapParser:
    def __init__(self, map_file: str) -> None:
        self.map_file = map_file
        self.zones: Dict[str, Zone] = {}
        self.connections: List[Connection] = []
        self.num_drones: Optional[int] = 0
        self.start_zone: Optional[Zone] = None
        self.end_zone: Optional[Zone] = None
        self.parse_map()
        self.graph = Graph(
            self.start_zone,
            self.end_zone,
            self.num_drones,
            self.zones,
            self.connections
        )

    def parse_map(self) -> Optional[Graph]:
        try:
            with open(self.map_file, 'r') as file:
                for n, line in enumerate(file, 1):
                    line = line.strip()
                    if not line or line.startswith('#'):
                        continue
                    self._parse_line(line)

            if self.start_zone is None:
                raise ValueError("Start hub (start_hub) is missing.")
            if self.end_zone is None:
                raise ValueError("End hub (end_hub) is missing.")
            if self.start_zone.zone_type == ZoneType.BLOCKED:
                raise ValueError("Start hub cannot be of type BLOCKED")
            if self.end_zone.zone_type == ZoneType.BLOCKED:
                raise ValueError("End hub cannot be of type BLOCKED")
            if self.num_drones is None:
                raise ValueError("Number of drones (nb_drones) is missing.")

            for conn in self.connections:
                if (conn.zone1 not in self.zones
                        or conn.zone2 not in self.zones):
                    raise ValueError(f"Connection {conn.zone1}-{conn.zone2}"
                                     "references unknown zone")
            return self.graph
        except Exception as e:
            print(f"Error parsing line {n}: {line}")
            print(f"Details: {e}")
            return None

    def _parse_line(self, line: str) -> None:
        if ':' not in line:
            raise ValueError("Line must contain a key and"
                             " value separated by ':'")
        key, value = line.split(':', 1)
        key = key.strip()
        value = value.strip()

        if key == 'nb_drones':
            if self.num_drones is not None:
                raise ValueError("Multiple nb_drones definitions")
            self._parse_nb_drones(value)
        elif key in ('hub', 'start_hub', 'end_hub'):
            self._parse_zone(key, value)
        elif key == 'connection':
            self._parse_connection(value)
        else:
            raise ValueError(f"Unknown key: {key}")

    def _parse_nb_drones(self, value: str) -> None:
        if not value.isdigit() or int(value) <= 0:
            raise ValueError("nb_drones must be a positive integer")
        self.num_drones = int(value)

    def _parse_zone(self, key: str, value: str) -> None:
        parts = value.split()
        zone_type: ZoneType = ZoneType.NORMAL
        color = 'none'
        max_drones = 1
        if len(parts) < 3:
            raise ValueError("Zone definition must have"
                             " at least name and coordinates")
        name = parts[0]
        if '-' in name or ' ' in name:
            raise ValueError(f"Zone name '{name}' "
                             "cannot contain dash or space")
        if name in self.zones:
            raise ValueError(f"Duplicate zone name: {name}")
        try:
            x = int(parts[1])
            y = int(parts[2])
        except ValueError:
            raise ValueError("Coordinates must be integers")
        if x < 0 or y < 0:
            raise ValueError("Coordinates must be non-negative")
        if len(parts) > 3:
            metadata = ' '.join(parts[3:])
            if metadata.startswith('[') and metadata.endswith(']'):
                metadata = metadata[1:-1]
                for meta in metadata.split():
                    if '=' not in meta:
                        raise ValueError(
                            "Metadata must be in key=value format")
                    meta_key, meta_value = meta.split('=', 1)
                    if meta_key == 'zone':
                        if meta_value.upper() not in ZoneType.__members__:
                            raise ValueError(
                                f"Invalid zone type: {meta_value}")
                        zone_type = ZoneType[meta_value.upper()]
                    elif meta_key == 'color':
                        if ' ' in meta_value:
                            raise ValueError(f"Color '{meta_value}' "
                                             "must be a single word")
                        color = meta_value
                    elif meta_key == 'max_drones':
                        if not meta_value.isdigit() or int(meta_value) <= 0:
                            raise ValueError("max_drones must be a "
                                             "positive integer")
                        max_drones = int(meta_value)
                    else:
                        raise ValueError(f"Unknown metadata key: {meta_key}")
            else:
                raise ValueError("Metadata must be enclosed in [metadata]")
        zone = Zone(name, x, y, color, zone_type, max_drones)
        self.zones[name] = zone
        if key == 'start_hub':
            if self.start_zone is not None:
                raise ValueError("Multiple start_hub definitions found")
            self.start_zone = zone
        elif key == 'end_hub':
            if self.end_zone is not None:
                raise ValueError("Multiple end_hub definitions found")
            self.end_zone = zone

    def _parse_connection(self, value: str) -> None:
        parts = value.split()
        if not parts:
            raise ValueError("Empty connection line")
        if '-' not in parts[0]:
            raise ValueError("Connection must be in the format 'zone1-zone2'")
        zone1, zone2 = parts[0].split('-', 1)
        if not zone1 or not zone2:
            raise ValueError("Connection must specify two zones")
        if zone1 == zone2:
            raise ValueError("Connection cannot be between the same zone")
        if zone1 not in self.zones or zone2 not in self.zones:
            raise ValueError("Connection references unknown"
                             f" zone: {zone1} or {zone2}")
        sorted_pair = sorted([zone1, zone2])
        edge: Tuple[str, str] = (sorted_pair[0], sorted_pair[1])
        if any(conn.key == edge for conn in self.connections):
            raise ValueError(f"Duplicate connection between {zone1}-{zone2}")

        max_capacity = 1
        if len(parts) > 1:
            raise ValueError(
                "Connection definition must have at least two zones")
        if zone1 == zone2:
            raise ValueError("Connection cannot be between the same zone")
        if len(parts) > 2:
            metadata = ' '.join(parts[2:])
            if metadata.startswith('[') and metadata.endswith(']'):
                metadata = metadata[1:-1]
                if '=' not in metadata:
                    raise ValueError("Metadata must be in key=value format")
                meta_key, meta_value = metadata.split('=', 1)
                if meta_key == 'max_link_capacity':
                    if not meta_value.isdigit() or int(meta_value) <= 0:
                        raise ValueError("max_link_capacity must"
                                         " be a positive integer")
                    max_capacity = int(meta_value)
                else:
                    raise ValueError(f"Unknown metadata key: {meta_key}")
            else:
                raise ValueError("Metadata must be enclosed in [metadata]")
        connection = Connection(zone1, zone2, max_capacity)
        self.connections.append(connection)
