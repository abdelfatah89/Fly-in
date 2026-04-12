"""Visual renderer: Tkinter-based animated simulation display."""
from math import pi, cos, sin
from typing import Tuple, Dict, List
from tkinter import Tk, Canvas
from .graph import Graph
from .simulator import Simulator
from .zone import Zone
from .drone import Drone, DroneStatus


class Renderer(Tk):
    """Tkinter window that animates the drone delivery simulation.

    Draws zones as colored circles, connections as lines, and drones
    as small circles arranged around their current zone. Advances the
    simulation one turn per animation frame.

    Attributes:
        sim: The Simulator driving the simulation logic.
        graph: The Graph containing zones, connections, and drones.
        zones_positions: Mapping of zone names to (Zone, x, y) tuples.
        drone_radius: Pixel radius for drone circles.
        zone_radius: Pixel radius for zone circles.
        grid_spacing: Pixel distance between adjacent grid positions.
        padding: Pixel margin around the drawing area.
    """

    def __init__(self, graph: Graph, sim: Simulator) -> None:
        """Initialize the renderer window and compute zone layout.

        Args:
            graph: The zone network graph.
            sim: The simulation engine.
        """
        self.sim = sim
        self.graph = graph

        self.min_x = min(
            [z.x for z in self.graph.zones.values()])
        self.max_x = max(
            [z.x for z in self.graph.zones.values()])
        self.min_y = min(
            [z.y for z in self.graph.zones.values()])
        self.max_y = max(
            [z.y for z in self.graph.zones.values()])

        self.zones_positions: Dict[
            str, Tuple[Zone, int, int]] = {}
        self.drone_radius = 6
        self.zone_radius = 20
        self.grid_spacing = 150
        self.padding = 60

        self.width, self.height = self.compute_layout()

        super().__init__()
        self.title("Drone Delivery Simulator")
        self.geometry(f"{self.width}x{self.height}")
        self.configure(bg="white")

        self.canvas = Canvas(self,
                             width=self.width,
                             height=self.height,
                             bg="white")
        self.canvas.pack()

        self._set_zones_positions()

    def compute_layout(self) -> Tuple[int, int]:
        """Calculate the canvas dimensions based on zone coordinates.

        Returns:
            Tuple of (width, height) in pixels.
        """
        graph_w = self.max_x - self.min_x
        graph_h = self.max_y - self.min_y

        width = (2 * self.padding + 2 * self.zone_radius
                 + graph_w * self.grid_spacing)
        height = (2 * self.padding + 2 * self.zone_radius
                  + graph_h * self.grid_spacing)

        return width, height

    def _set_zones_positions(self) -> None:
        """Map each zone to its pixel coordinates on the canvas."""
        for name, zone in self.graph.zones.items():
            zone_x = (self.padding
                      + (zone.x - self.min_x)
                      * self.grid_spacing)
            zone_y = (self.padding
                      + (self.max_y - zone.y)
                      * self.grid_spacing)
            self.zones_positions[name] = (
                zone, zone_x, zone_y)

    def _drone_positions(
        self, cx: float, cy: float,
        count: int, index: int
    ) -> Tuple[float, float]:
        """Calculate the position of a drone within a shared zone.

        Distributes multiple drones in a circular fan-out pattern.

        Args:
            cx: Center x of the zone.
            cy: Center y of the zone.
            count: Total number of drones in this zone.
            index: This drone's index among them.

        Returns:
            Tuple of (x, y) pixel coordinates for the drone.
        """
        radius = self.zone_radius * 0.45
        if count == 1:
            return cx, cy
        angle = 2 * pi * index / count
        return (cx + radius * cos(angle),
                cy + radius * sin(angle))

    def draw_connections(self) -> None:
        """Draw all connections as lines between zone centers."""
        for connection in self.graph.connections:
            zone1 = self.graph.zones[connection.zone1]
            zone2 = self.graph.zones[connection.zone2]
            _, x1, y1 = self.zones_positions[zone1.name]
            _, x2, y2 = self.zones_positions[zone2.name]
            self.canvas.create_line(
                x1, y1, x2, y2, fill="black", width=2)

    def draw_zone(
        self, zone: Zone, x: int, y: int
    ) -> None:
        """Draw a single zone circle with its label and occupancy.

        Args:
            zone: The zone to draw.
            x: Pixel x coordinate.
            y: Pixel y coordinate.
        """
        radius = self.zone_radius
        start = self.graph.start_hub
        goal = self.graph.end_hub

        color = (zone.color if zone.color != "none"
                 else "lightgray")
        if color == "rainbow":
            self.draw_rainbow_zone(x, y)
        else:
            self.canvas.create_oval(
                x - radius, y - radius,
                x + radius, y + radius,
                fill=color)

        self.canvas.create_rectangle(
            x - 40, y - 35,
            x + 40, y - 15,
            fill="white",
            outline=""
        )
        self.canvas.create_text(
            x, y - 25,
            text=zone.name,
            font=("Arial", 9, "bold"),
            fill="black"
        )

        if zone.name == start.name:
            count = sum(
                1 for d in self.graph.drones
                if d.current_zone.name == start.name
                and d.drone_status
                != DroneStatus.IN_TRANSIT)
            self.canvas.create_text(
                x, y + radius + 10,
                text=f"[{count}/{self.graph.nb_drones}]",
                font=("Arial", 8, "bold"),
                fill="darkblue"
            )
        elif zone.name == goal.name:
            delivered = sum(
                1 for d in self.graph.drones
                if d.current_zone.name == goal.name
                and d.drone_status
                != DroneStatus.IN_TRANSIT)
            self.canvas.create_text(
                x, y + radius + 10,
                text=(f"[{delivered}"
                      f"/{self.graph.nb_drones}]"),
                font=("Arial", 8, "bold"),
                fill="darkblue"
            )
        else:
            current_count = zone.current_occupancy
            transit_count = len([
                drone for drone in self.graph.drones
                if drone.drone_status
                == DroneStatus.IN_TRANSIT
                and drone.target_zone
                and drone.target_zone.name == zone.name
            ])

            effective = current_count + transit_count
            self.canvas.create_text(
                x, y + radius + 10,
                text=(f"[{effective}"
                      f"/{zone.max_capacity}]"),
                font=("Arial", 8, "bold"),
                fill="darkblue"
            )

    def draw_rainbow_zone(
        self, x: int, y: int
    ) -> None:
        """Draw a zone with concentric rainbow-colored rings.

        Args:
            x: Pixel x coordinate.
            y: Pixel y coordinate.
        """
        colors = [
            "red", "orange", "yellow",
            "green", "cyan", "blue", "purple"]
        for i, color in enumerate(colors):
            radius = self.zone_radius - i * 2
            self.canvas.create_oval(
                x - radius, y - radius,
                x + radius, y + radius,
                outline=color)
        inner = radius - len(color) * 2
        self.canvas.create_oval(
            x - inner, y - inner,
            x + inner, y + inner,
            fill="white", outline="",
            width=0)

    def draw_drones(self) -> None:
        """Draw all drones at their current or transit positions."""
        radius = self.drone_radius
        drones_by_zone: Dict[str, List[Drone]] = {}
        for drone in self.graph.drones:
            if (drone.target_zone
                    and drone.drone_status
                    == DroneStatus.IN_TRANSIT):
                zone_name = drone.target_zone.name
            else:
                zone_name = drone.current_zone.name
            drones_by_zone.setdefault(
                zone_name, []).append(drone)

        for zone_name, drones in drones_by_zone.items():
            _, x, y = self.zones_positions[zone_name]
            count = len(drones)
            for index, drone in enumerate(drones):
                d_x, d_y = self._drone_positions(
                    x, y, count, index)
                self.canvas.create_oval(
                    d_x - radius, d_y - radius,
                    d_x + radius, d_y + radius,
                    fill="orange",
                    outline="darkred",
                    width=1)

    def draw_graph(self) -> None:
        """Draw the complete graph: connections, zones, and drones."""
        self.draw_connections()
        for zone in self.zones_positions.values():
            self.draw_zone(*zone)
        self.draw_drones()

    def animate(self) -> None:
        """Advance one simulation turn and schedule the next frame."""
        if self.sim.all_delivered():
            print("#", "=" * 80)
            print("#", "All drones delivered in",
                  len(self.sim.output_lines), "turns")
            print("#", "=" * 80)
            return
        line = self.sim.execute_turn()
        self.canvas.delete('all')
        self.draw_graph()
        self.after(500, self.animate)
        if line:
            print(line)
