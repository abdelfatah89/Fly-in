import tkinter as tk
from typing import Tuple, Dict, List
import math
from drone import Drone
from zone import Zone


class Renderer:
    def __init__(self, graph, simulator):
        self.current_turn = 0
        self.graph = graph
        self.simulator = simulator

        self.canvas_width = 1900
        self.canvas_height = 700

        self.root = tk.Tk()
        self.root.title("Fly-in Drone Simulation")
        self.root.geometry(f"{self.canvas_width + 20}x{self.canvas_height + 150}")

        self.canvas = tk.Canvas(
            self.root,
            width=self.canvas_width,
            height=self.canvas_height,
            bg="white"
        )
        self.canvas.pack()
        
        # Add turn counter label
        self.turn_label = tk.Label(
            self.root,
            text=f"Turn: {self.current_turn}",
            font=("Arial", 14, "bold")
        )
        self.turn_label.pack()
        
        # Initialize scale before drawing
        self._calculate_scale()
        
        # Draw initial state
        self.draw_map()

    def _is_finished(self) -> bool:
        """Check if simulation is finished."""
        return all(
            d.current_zone == self.graph.end_zone.name
            for d in self.graph.drones
        )

    def _get_zone_color(self, zone: Zone) -> str:
        """Get display color for a zone based on its type.
        
        Args:
            zone: The zone to get color for
            
        Returns:
            Color string
        """
        from zone import ZoneType
        
        type_colors = {
            ZoneType.START: "#90EE90",      # Light green
            ZoneType.END: "#FFD700",        # Gold
            ZoneType.NORMAL: "#87CEEB",     # Sky blue
            ZoneType.PRIORITY: "#98FB98",   # Pale green
            ZoneType.RESTRICTED: "#FFA07A", # Light salmon
            ZoneType.BLOCKED: "#D3D3D3"     # Light gray
        }
        return type_colors.get(zone.zone_type, "#CCCCCC")

    def _calculate_scale(self):
        if not self.graph.zones:
            self.scale = 50
            self.offset_x = 50
            self.offset_y = 50
            return

        xs = [zone.x for zone in self.graph.zones.values()]
        ys = [zone.y for zone in self.graph.zones.values()]

        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)

        padding = 50
        available_width = self.canvas_width - 2 * padding
        available_height = self.canvas_height - 2 * padding

        width_range = max_x - min_x if max_x > min_x else 1
        height_range = max_y - min_y if max_y > min_y else 1

        scale_x = available_width / width_range
        scale_y = available_height / height_range

        self.scale = min(scale_x, scale_y)
        self.offset_x = padding - min_x * self.scale
        self.offset_y = padding - min_y * self.scale

    def _map_to_canvas(self, x: float, y: float) -> Tuple[float, float]:
        canvas_x = x * self.scale + self.offset_x
        canvas_y = y * self.scale + self.offset_y
        return (canvas_x, canvas_y)

    def draw_map(self):
        # Clear canvas before redrawing
        self.canvas.delete("all")
        self._calculate_scale()
        self.draw_connections()
        self.draw_zones()
        self.draw_drones()

    def draw_zones(self) -> None:
        radius = 25

        for zone in self.graph.zones.values():
            cx, cy = self._map_to_canvas(zone.x, zone.y)

            color = self._get_zone_color(zone)
            self.canvas.create_oval(
                cx - radius, cy - radius,
                cx + radius, cy + radius,
                fill=color,
                outline="black",
                width=2
            )

            self.canvas.create_rectangle(
                cx - 40,
                cy - 35,
                cx + 40,
                cy - 15,
                fill="white",
                outline=""
            )
            
            # Draw zone name
            self.canvas.create_text(
                cx, cy - 25,
                text=zone.name,
                font=("Arial", 9, "bold"),
                fill="black"
            )
            
            # Draw capacity if limited
            if not zone.is_unlimited_capacity:
                from drone import DroneStatus
                
                # Count drones actually at this zone
                current_count = zone.current_occupancy
                
                # Add drones in transit to this zone (for restricted zones)
                transit_count = sum(
                    1 for d in self.graph.drones 
                    if d.status == DroneStatus.IN_TRANSIT 
                    and d.transit_destination == zone.name
                )
                
                # Total effective occupancy
                effective_occupancy = current_count + transit_count
                
                self.canvas.create_text(
                    cx, cy + radius + 10,
                    text=f"[{effective_occupancy}/{zone.max_capacity}]",
                    font=("Arial", 8, "bold"),
                    fill="darkblue"
                )

    def draw_connections(self):
        for conn in self.graph.connections:
            zone1 = self.graph.zones[conn.zone1]
            zone2 = self.graph.zones[conn.zone2]

            x1, y1 = self._map_to_canvas(zone1.x, zone1.y)
            x2, y2 = self._map_to_canvas(zone2.x, zone2.y)

            self.canvas.create_line(
                x1, y1,
                x2, y2,
                fill="black",
                width=2
            )
            if conn.max_capacity > 1:
                mid_x = (x1 + x2) / 2
                mid_y = (y1 + y2) / 2

                self.canvas.create_text(
                    mid_x, mid_y,
                    text=f"[{conn.max_capacity}]",
                    font=("Arial", 8, "bold"),
                    fill="blue"
                )

    def draw_drones(self) -> None:
        """Draw all drones at their current positions."""
        from drone import DroneStatus
        
        # Group drones by zone
        drones_per_zone: Dict[str, List[Drone]] = {}
        for drone in self.graph.drones:
            # For drones in transit, draw them at their transit destination
            if drone.status == DroneStatus.IN_TRANSIT and drone.transit_destination:
                zone = drone.transit_destination
            else:
                zone = drone.current_zone
            
            if zone not in drones_per_zone:
                drones_per_zone[zone] = []
            drones_per_zone[zone].append(drone)
        
        # Draw drones
        for zone_name, drones in drones_per_zone.items():
            zone = self.graph.zones[zone_name]
            cx, cy = self._map_to_canvas(zone.x, zone.y)
            
            # Arrange drones in a circle around zone center
            for i, drone in enumerate(drones):
                angle = (2 * 3.14159 * i) / len(drones)
                dx = 8 * math.cos(angle)
                dy = 8 * math.sin(angle)
                
                # Draw drone as small circle
                self.canvas.create_oval(
                    cx + dx - 5, cy + dy - 5,
                    cx + dx + 5, cy + dy + 5,
                    fill="red",
                    outline="darkred",
                    width=1
                )

    def step_simulation(self) -> None:
        """Execute one simulation turn."""
        if self._is_finished():
            return
        
        self.simulator.execute_turn()
        self.current_turn += 1
        self.turn_label.config(text=f"Turn: {self.current_turn}")
        self.draw_map()

    def run_simulation(self):
        if not self._is_finished():
            self.step_simulation()
            self.root.after(500, self.run_simulation)
        else:
            # Simulation complete
            self.turn_label.config(
                text=f"Simulation Complete! Turns: {self.current_turn}",
                fg="green"
            )

    def run(self):
        try:
            # Force window update to ensure canvas is ready
            self.root.update_idletasks()
            self.root.update()
            # Start simulation after ensuring window is ready
            self.root.after(200, self.run_simulation)
            self.root.mainloop()
        except Exception as e:
            print(f"Error running renderer: {e}")
            self.root.destroy()