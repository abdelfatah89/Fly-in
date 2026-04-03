"""Map builder - converts parsed data into MapData objects."""

from typing import Dict, Any
from ..models import Zone, ZoneType, Connection, MapData


class MapBuilder:
    """Builds MapData objects from parsed map file data."""

    @staticmethod
    def build_from_parsed_data(parsed_data: Dict[str, Any]) -> MapData:
        """Build a MapData object from parser output.

        Args:
            parsed_data: Dictionary from MapParser.get_parsed_data()

        Returns:
            Constructed and validated MapData object
        """
        map_data = MapData()

        # Build zones
        for zone_name, zone_info in parsed_data['zones'].items():
            zone = MapBuilder._build_zone(zone_info)
            map_data.add_zone(zone)

        # Build connections
        for conn_info in parsed_data['connections']:
            connection = MapBuilder._build_connection(conn_info)
            map_data.add_connection(connection)

        # Create drones
        map_data.create_drones(parsed_data['nb_drones'])

        # Validate the complete map
        map_data.validate()

        return map_data

    @staticmethod
    def _build_zone(zone_info: Dict[str, Any]) -> Zone:
        """Build a Zone object from parsed zone data.

        Args:
            zone_info: Zone information from parser

        Returns:
            Constructed Zone object
        """
        name = zone_info['name']
        x = zone_info['x']
        y = zone_info['y']
        parser_type = zone_info['type']
        attributes = zone_info['attributes']

        # Determine zone type
        if parser_type == 'start':
            zone_type = ZoneType.START
        elif parser_type == 'end':
            zone_type = ZoneType.END
        else:
            # Check for zone attribute
            zone_attr = attributes.get('zone', 'normal')
            if zone_attr == 'restricted':
                zone_type = ZoneType.RESTRICTED
            elif zone_attr == 'priority':
                zone_type = ZoneType.PRIORITY
            elif zone_attr == 'blocked':
                zone_type = ZoneType.BLOCKED
            else:
                zone_type = ZoneType.NORMAL

        # Get capacity
        max_capacity = 1
        if 'max_drones' in attributes:
            try:
                max_capacity = int(attributes['max_drones'])
            except ValueError:
                max_capacity = 1

        # Get color
        color = attributes.get('color', 'gray')

        return Zone(
            name=name,
            x=x,
            y=y,
            zone_type=zone_type,
            max_capacity=max_capacity,
            color=color
        )

    @staticmethod
    def _build_connection(conn_info: Dict[str, Any]) -> Connection:
        """Build a Connection object from parsed connection data.

        Args:
            conn_info: Connection information from parser

        Returns:
            Constructed Connection object
        """
        zone1 = conn_info['zone1']
        zone2 = conn_info['zone2']
        attributes = conn_info['attributes']

        # Get capacity
        max_capacity = 1
        if 'max_link_capacity' in attributes:
            try:
                max_capacity = int(attributes['max_link_capacity'])
            except ValueError:
                max_capacity = 1

        return Connection(
            zone1_name=zone1,
            zone2_name=zone2,
            max_capacity=max_capacity
        )
