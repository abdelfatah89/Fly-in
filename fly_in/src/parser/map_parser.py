"""Map file parser.

This module parses the Fly-in map file format and extracts:
- Number of drones
- Zones (hubs) with their properties
- Connections between zones with capacities
"""

import re
from typing import Dict, List, Any, Optional


class ParseError(Exception):
    """Raised when map file parsing fails."""
    pass


class MapParser:
    """Parser for Fly-in map files.

    The parser handles the following format:
    - nb_drones: <number>
    - start_hub: <name> <x> <y> [attributes]
    - hub: <name> <x> <y> [attributes]
    - end_hub: <name> <x> <y> [attributes]
    - connection: <zone1>-<zone2> [attributes]
    """

    def __init__(self, filepath: str) -> None:
        """Initialize parser with a map file path.

        Args:
            filepath: Path to the map file to parse
        """
        self.filepath: str = filepath
        self.nb_drones: int = 0
        self.zones: Dict[str, Dict[str, Any]] = {}
        self.connections: List[Dict[str, Any]] = []
        self.start_zone: Optional[str] = None
        self.end_zone: Optional[str] = None

    def parse(self) -> None:
        """Parse the map file and populate internal structures.

        Raises:
            ParseError: If the file format is invalid
            FileNotFoundError: If the file doesn't exist
        """
        with open(self.filepath, 'r', encoding='utf-8') as f:
            lines = f.readlines()

        for line_num, line in enumerate(lines, start=1):
            # Strip whitespace and skip empty lines and comments
            line = line.strip()
            if not line or line.startswith('#'):
                continue

            try:
                self._parse_line(line)
            except Exception as e:
                raise ParseError(
                    f"Error parsing line {line_num}: {line}\n{str(e)}"
                )

        # Validate that we have all required elements
        self._validate_map()

    def _parse_line(self, line: str) -> None:
        """Parse a single line from the map file.

        Args:
            line: Stripped line from the file
        """
        if line.startswith('nb_drones:'):
            self._parse_nb_drones(line)
        elif line.startswith('start_hub:'):
            self._parse_zone(line, zone_type='start')
        elif line.startswith('end_hub:'):
            self._parse_zone(line, zone_type='end')
        elif line.startswith('hub:'):
            self._parse_zone(line, zone_type='normal')
        elif line.startswith('connection:'):
            self._parse_connection(line)
        else:
            raise ParseError(f"Unknown line format: {line}")

    def _parse_nb_drones(self, line: str) -> None:
        """Parse the number of drones line.

        Format: nb_drones: <number>

        Args:
            line: The line to parse
        """
        match = re.match(r'nb_drones:\s*(\d+)', line)
        if not match:
            raise ParseError(f"Invalid nb_drones format: {line}")

        self.nb_drones = int(match.group(1))
        if self.nb_drones <= 0:
            raise ParseError("Number of drones must be positive")

    def _parse_zone(self, line: str, zone_type: str) -> None:
        """Parse a zone (hub) line.

        Format: <type>: <name> <x> <y> [attributes]

        Args:
            line: The line to parse
            zone_type: Type of zone (start, end, normal)
        """
        # Remove the prefix
        if zone_type == 'start':
            content = line[len('start_hub:'):].strip()
        elif zone_type == 'end':
            content = line[len('end_hub:'):].strip()
        else:
            content = line[len('hub:'):].strip()

        # Extract attributes if present
        attributes_str = ""
        if '[' in content and ']' in content:
            main_part, attributes_str = content.split('[', 1)
            attributes_str = attributes_str.rstrip(']')
            content = main_part.strip()

        # Parse name and coordinates
        parts = content.split()
        if len(parts) < 3:
            raise ParseError(f"Zone must have name and x,y coordinates: {line}")

        name = parts[0]
        try:
            x = float(parts[1])
            y = float(parts[2])
        except ValueError:
            raise ParseError(f"Invalid coordinates for zone {name}: {line}")

        # Parse attributes
        attributes = self._parse_attributes(attributes_str)

        # Store zone information
        if name in self.zones:
            raise ParseError(f"Duplicate zone name: {name}")

        self.zones[name] = {
            'name': name,
            'x': x,
            'y': y,
            'type': zone_type,
            'attributes': attributes
        }

        # Track start and end zones
        if zone_type == 'start':
            if self.start_zone is not None:
                raise ParseError("Multiple start zones defined")
            self.start_zone = name
        elif zone_type == 'end':
            if self.end_zone is not None:
                raise ParseError("Multiple end zones defined")
            self.end_zone = name

    def _parse_connection(self, line: str) -> None:
        """Parse a connection line.

        Format: connection: <zone1>-<zone2> [attributes]

        Args:
            line: The line to parse
        """
        content = line[len('connection:'):].strip()

        # Extract attributes if present
        attributes_str = ""
        if '[' in content and ']' in content:
            main_part, attributes_str = content.split('[', 1)
            attributes_str = attributes_str.rstrip(']')
            content = main_part.strip()

        # Parse zone names
        if '-' not in content:
            raise ParseError(f"Connection must use format zone1-zone2: {line}")

        parts = content.split('-')
        if len(parts) != 2:
            raise ParseError(f"Connection must connect exactly two zones: {line}")

        zone1 = parts[0].strip()
        zone2 = parts[1].strip()

        if not zone1 or not zone2:
            raise ParseError(f"Empty zone name in connection: {line}")

        # Parse attributes
        attributes = self._parse_attributes(attributes_str)

        # Store connection
        self.connections.append({
            'zone1': zone1,
            'zone2': zone2,
            'attributes': attributes
        })

    def _parse_attributes(self, attributes_str: str) -> Dict[str, str]:
        """Parse attributes from the [key=value ...] format.

        Args:
            attributes_str: String containing attributes

        Returns:
            Dictionary of attribute key-value pairs
        """
        attributes: Dict[str, str] = {}
        if not attributes_str:
            return attributes

        # Split by whitespace and parse key=value pairs
        for attr in attributes_str.split():
            attr = attr.strip()
            if not attr:
                continue

            if '=' in attr:
                key, value = attr.split('=', 1)
                attributes[key.strip()] = value.strip()
            else:
                # Standalone attribute (like a flag)
                attributes[attr] = 'true'

        return attributes

    def _validate_map(self) -> None:
        """Validate that the parsed map is complete and consistent.

        Raises:
            ParseError: If validation fails
        """
        if self.nb_drones == 0:
            raise ParseError("Missing nb_drones declaration")

        if self.start_zone is None:
            raise ParseError("Missing start_hub declaration")

        if self.end_zone is None:
            raise ParseError("Missing end_hub declaration")

        if not self.zones:
            raise ParseError("No zones defined")

        # Validate connections reference existing zones
        for conn in self.connections:
            if conn['zone1'] not in self.zones:
                raise ParseError(f"Connection references unknown zone: {conn['zone1']}")
            if conn['zone2'] not in self.zones:
                raise ParseError(f"Connection references unknown zone: {conn['zone2']}")

    def get_parsed_data(self) -> Dict[str, Any]:
        """Get the parsed map data.

        Returns:
            Dictionary containing:
                - nb_drones: Number of drones
                - zones: Dictionary of zone data
                - connections: List of connection data
                - start_zone: Name of start zone
                - end_zone: Name of end zone
        """
        return {
            'nb_drones': self.nb_drones,
            'zones': self.zones,
            'connections': self.connections,
            'start_zone': self.start_zone,
            'end_zone': self.end_zone
        }
