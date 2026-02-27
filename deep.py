class ParseError(Exception):
    pass

class MapParser:
    def __init__(self, filename):
        self.filename = filename
        self.zones = {}
        self.connections = []
        self.num_drones = None
        self.start_zone = None
        self.end_zone = None

    def parse(self):
        with open(self.filename) as f:
            for line_no, line in enumerate(f, 1):
                line = line.strip()
                if not line or line.startswith('#'):
                    continue
                try:
                    self._parse_line(line, line_no)
                except Exception as e:
                    raise ParseError(f"Line {line_no}: {e}")

        # Post‑parse validation
        if self.num_drones is None:
            raise ParseError("Missing nb_drones line")
        if self.start_zone is None:
            raise ParseError("Missing start_hub")
        if self.end_zone is None:
            raise ParseError("Missing end_hub")
        # ... more checks (unique start/end already ensured by single assignment)
        # Validate all connection zones exist
        for conn in self.connections:
            if conn.zone1 not in self.zones or conn.zone2 not in self.zones:
                raise ParseError(f"Connection {conn.zone1}-{conn.zone2} references unknown zone")
        return self._build_graph()

    def _parse_line(self, line, line_no):
        if ':' not in line:
            raise ValueError("Missing colon")
        key, value = line.split(':', 1)
        key = key.strip()
        value = value.strip()

        if key == "nb_drones":
            self._parse_nb_drones(value)
        elif key in ("start_hub", "end_hub", "hub"):
            self._parse_zone(key, value)
        elif key == "connection":
            self._parse_connection(value)
        else:
            raise ValueError(f"Unknown key: {key}")

    def _parse_nb_drones(self, value):
        if not value.isdigit() or int(value) <= 0:
            raise ValueError("nb_drones must be a positive integer")
        self.num_drones = int(value)

    def _parse_zone(self, key, value):
        # Split on whitespace, but metadata may contain spaces, so we need to separate the last part
        parts = value.split()
        if len(parts) < 3:
            raise ValueError("Zone line must have at least name, x, y")
        name = parts[0]
        # Validate name: no dashes or spaces
        if '-' in name or ' ' in name:
            raise ValueError(f"Zone name '{name}' cannot contain dash or space")
        if name in self.zones:
            raise ValueError(f"Duplicate zone name '{name}'")
        try:
            x = int(parts[1])
            y = int(parts[2])
        except ValueError:
            raise ValueError("Coordinates must be integers")
        if x < 0 or y < 0:
            raise ValueError("Coordinates must be non‑negative")

        # Parse metadata if present
        zone_type = "normal"
        color = "none"
        max_drones = 1
        if len(parts) > 3:
            # Everything after the coordinates should be a single bracket‑enclosed string
            meta_str = ' '.join(parts[3:])
            if not (meta_str.startswith('[') and meta_str.endswith(']')):
                raise ValueError("Metadata must be enclosed in brackets")
            meta_str = meta_str[1:-1].strip()
            if meta_str:
                for token in meta_str.split():
                    if '=' not in token:
                        raise ValueError(f"Invalid metadata token: {token}")
                    k, v = token.split('=', 1)
                    k = k.strip().lower()
                    v = v.strip()
                    if k == "zone":
                        if v.lower() not in ("normal", "blocked", "restricted", "priority"):
                            raise ValueError(f"Invalid zone type: {v}")
                        zone_type = v.lower()
                    elif k == "color":
                        if ' ' in v:
                            raise ValueError("Color must be a single word")
                        color = v
                    elif k == "max_drones":
                        if not v.isdigit() or int(v) <= 0:
                            raise ValueError("max_drones must be a positive integer")
                        max_drones = int(v)
                    else:
                        raise ValueError(f"Unknown zone metadata key: {k}")

        zone = Zone(name, x, y, zone_type, color, max_drones)
        self.zones[name] = zone
        if key == "start_hub":
            if self.start_zone is not None:
                raise ValueError("Multiple start_hub definitions")
            self.start_zone = zone
        elif key == "end_hub":
            if self.end_zone is not None:
                raise ValueError("Multiple end_hub definitions")
            self.end_zone = zone
        # hub: just store in zones dict

    def _parse_connection(self, value):
        # Split into zone part and optional metadata
        parts = value.split()
        if len(parts) < 1:
            raise ValueError("Empty connection line")
        zone_part = parts[0]
        if '-' not in zone_part:
            raise ValueError("Connection must contain '-' between zone names")
        z1, z2 = zone_part.split('-', 1)
        if not z1 or not z2:
            raise ValueError("Invalid zone names in connection")
        # Check for duplicate edge (normalized order)
        edge = tuple(sorted([z1, z2]))
        if any(conn.key == edge for conn in self.connections):
            raise ValueError(f"Duplicate connection {z1}-{z2}")

        capacity = 1
        if len(parts) > 1:
            meta = ' '.join(parts[1:])
            if not (meta.startswith('[') and meta.endswith(']')):
                raise ValueError("Connection metadata must be enclosed in brackets")
            meta = meta[1:-1].strip()
            if meta:
                if '=' not in meta:
                    raise ValueError("Connection metadata must be key=value")
                k, v = meta.split('=', 1)
                k = k.strip().lower()
                v = v.strip()
                if k != "max_link_capacity":
                    raise ValueError(f"Unknown connection metadata key: {k}")
                if not v.isdigit() or int(v) <= 0:
                    raise ValueError("max_link_capacity must be a positive integer")
                capacity = int(v)

        # Store connection (zones not yet validated – will be checked after all zones parsed)
        conn = Connection(z1, z2, capacity)
        conn.key = edge  # for duplicate detection
        self.connections.append(conn)