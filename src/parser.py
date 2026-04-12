"""Map file parser: reads and validates the drone network definition."""
from typing import Dict, Any, Tuple, Optional
from .connection import Connection
from .zone import Zone, ZoneType


class ParserError(Exception):
    """Raised when the input map file contains invalid syntax or data."""

    pass


class Parser:
    """Parses a drone network map file into structured data.

    Validates syntax, zone definitions, connections, and metadata.
    Reports errors with line numbers and causes.

    Attributes:
        data_parsed: Dictionary accumulating parsed map components.
    """

    def __init__(self) -> None:
        """Initialize the parser with an empty data structure."""
        self.data_parsed: Dict[str, Any] = {
            "nb_drones": None,
            "start_hub": None,
            "end_hub": None,
            "zones": {},
            "connections": []
        }

    def parse(self, file: str) -> Optional[Dict[str, Any]]:
        """Parse a map file and return the structured data.

        Args:
            file: Path to the map file.

        Returns:
            Dictionary with keys nb_drones, start_hub, end_hub,
            zones, and connections. Returns None on parse failure.
        """
        n, line = 0, "Error at Begin"
        first_data_line_seen = False

        try:
            with open(file, 'r') as f:
                for n, line in enumerate(f, 1):
                    line = line.strip()
                    if not line or line.startswith('#'):
                        continue

                    if not first_data_line_seen:
                        if not line.startswith("nb_drones:"):
                            raise ParserError(
                                "First data line must"
                                " define nb_drones")
                        first_data_line_seen = True

                    self._parse_line(line)
            self._data_checker()
            return self.data_parsed
        except Exception as e:
            print(f"Error parsing line {n}: {line}")
            print(f"Details: {e}")
            return None

    def _parse_line(self, line: str) -> None:
        """Dispatch a single line to the appropriate sub-parser.

        Args:
            line: A stripped, non-empty, non-comment line.

        Raises:
            ParserError: If the line format or key is invalid.
        """
        if ':' not in line:
            raise ParserError(
                "Line must contain a key and value"
                " separated by ':'")
        key, value = line.split(':', 1)
        key = key.strip()
        value = value.strip()

        if key == "nb_drones":
            if self.data_parsed["nb_drones"] is not None:
                raise ParserError(
                    "Multiple nb_drones definitions")
            self._parse_nb_drones(value)
        elif key in ('hub', 'start_hub', 'end_hub'):
            self._parse_zone(key, value)
        elif key == "connection":
            self._parse_connection(value)
        else:
            raise ParserError(f"Unknown Key: {key}")

    def _parse_nb_drones(self, value: str) -> None:
        """Parse and validate the drone count.

        Args:
            value: The string value after 'nb_drones:'.

        Raises:
            ValueError: If the value is not a positive integer.
        """
        if not value.isdigit() or int(value) <= 0:
            raise ValueError(
                "nb_drones must be a positive integer")
        self.data_parsed["nb_drones"] = int(value)

    def _parse_zone(self, key: str, value: str) -> None:
        """Parse a zone definition line with optional metadata.

        Args:
            key: The zone type prefix (hub, start_hub, end_hub).
            value: The zone definition string after the colon.

        Raises:
            ParserError: On invalid name, coordinates, or metadata.
        """
        zone_info: Dict[str, Any] = {
            "zone_type": ZoneType.NORMAL,
            "color": "none",
            "max_capacity": 1
        }
        info = value.split()
        if len(info) < 3:
            raise ParserError(
                "Zone definition must have "
                "at least name and coordinates")
        zone_name = info[0]
        if '-' in zone_name or ' ' in zone_name:
            raise ParserError(
                f"Zone name '{zone_name}' "
                "cannot contain dash or space")
        if zone_name in self.data_parsed["zones"]:
            raise ParserError(
                f"Duplicate zone name: {zone_name}")
        zone_info["name"] = zone_name
        try:
            x = int(info[1])
            y = int(info[2])
            zone_info["x"] = x
            zone_info["y"] = y
        except ValueError:
            raise ParserError("Coordinates must be integers")

        if len(info) > 3:
            metadata = ' '.join(info[3:])
            if (not metadata.startswith('[')
                    or not metadata.endswith(']')):
                raise ParserError(
                    "Metadata must be enclosed"
                    " in [metadata]")
            metadata = metadata[1:-1]
            for data in metadata.split():
                if '=' not in data:
                    raise ParserError(
                        "Metadata must be in"
                        " key=value format")
                data_key, data_value = data.split('=', 1)
                if data_key == "zone":
                    if (data_value.upper()
                            not in ZoneType.__members__):
                        raise ParserError(
                            f"Invalid zone type:"
                            f" {data_value}")
                    zone_info["zone_type"] = (
                        ZoneType[data_value.upper()])
                elif data_key == "color":
                    if ' ' in data_value:
                        raise ParserError(
                            f"Color '{data_value}'"
                            " must be a single word")
                    zone_info["color"] = data_value
                elif data_key == "max_drones":
                    if (not data_value.isdigit()
                            or int(data_value) <= 0):
                        raise ParserError(
                            "max_drones must be a"
                            " positive integer")
                    zone_info["max_capacity"] = (
                        int(data_value))
                else:
                    raise ParserError(
                        f"Unknown metadata key:"
                        f" {data_key}")

        zone = Zone(**zone_info)
        self.data_parsed["zones"][zone.name] = zone
        if key == 'start_hub':
            if self.data_parsed["start_hub"] is not None:
                raise ParserError(
                    "Multiple start_hub definitions found")
            self.data_parsed["start_hub"] = zone
        elif key == 'end_hub':
            if self.data_parsed["end_hub"] is not None:
                raise ParserError(
                    "Multiple end_hub definitions found")
            self.data_parsed["end_hub"] = zone

    def _parse_connection(self, value: str) -> None:
        """Parse a connection definition with optional metadata.

        Args:
            value: The connection definition after 'connection:'.

        Raises:
            ParserError: On invalid format, unknown zones,
            or duplicate connections.
        """
        connection_info: Dict[str, Any] = {
            "max_link_capacity": 1
        }
        info = value.split()
        if not info:
            raise ParserError("Empty connection line")
        if '-' not in info[0]:
            raise ParserError(
                "Connection must be in"
                " the format 'zone1-zone2'")
        zone1, zone2 = info[0].split('-', 1)
        if not zone1 or not zone2:
            raise ParserError(
                "Connection must specify two zones")
        if zone1 == zone2:
            raise ParserError(
                "Connection cannot be between"
                " the same zone")
        if (zone1 not in self.data_parsed["zones"]
           or zone2 not in self.data_parsed["zones"]):
            raise ParserError(
                "Connection references unknown"
                f" zone: {zone1} or {zone2}")
        sorted_zones = sorted([zone1, zone2])
        edge: Tuple[str, str] = (
            sorted_zones[0], sorted_zones[1])
        if any(conn.key == edge
               for conn in self.data_parsed["connections"]):
            raise ParserError(
                f"Duplicate connection"
                f" between {zone1}-{zone2}")
        connection_info["zone1"] = zone1
        connection_info["zone2"] = zone2
        if len(info) > 1:
            metadata = ' '.join(info[1:])
            if (not metadata.startswith('[')
                    or not metadata.endswith(']')):
                raise ParserError(
                    "Metadata must be enclosed"
                    " in [metadata]")
            metadata = metadata[1:-1]
            if '=' not in metadata:
                raise ParserError(
                    "Metadata must be in"
                    " key=value format")
            data_key, data_value = metadata.split('=', 1)
            if data_key == 'max_link_capacity':
                if (not data_value.isdigit()
                        or int(data_value) <= 0):
                    raise ParserError(
                        "max_link_capacity must be"
                        " a positive integer")
                connection_info["max_link_capacity"] = (
                    int(data_value))
            else:
                raise ParserError(
                    f"Unknown metadata key: {data_key}")

        connection = Connection(**connection_info)
        self.data_parsed["connections"].append(connection)

    def _data_checker(self) -> None:
        """Validate that all required data was parsed.

        Raises:
            ParserError: If start_hub, end_hub, or nb_drones
            are missing or invalid.
        """
        if self.data_parsed["start_hub"] is None:
            raise ParserError(
                "Start hub (start_hub) is missing.")
        if self.data_parsed["end_hub"] is None:
            raise ParserError(
                "End hub (end_hub) is missing.")
        if self.data_parsed["start_hub"].zone_type == (
                ZoneType.BLOCKED):
            raise ParserError(
                "Start hub cannot be of type BLOCKED")
        if self.data_parsed["end_hub"].zone_type == (
                ZoneType.BLOCKED):
            raise ParserError(
                "End hub cannot be of type BLOCKED")
        if self.data_parsed["nb_drones"] is None:
            raise ParserError(
                "Number of drones (nb_drones) is missing.")
