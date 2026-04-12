"""Tkinter GUI for drone simulation visualization."""

import tkinter as tk
from tkinter import ttk
from typing import Dict, List, Tuple, Optional
from ..models import MapData, Drone, Zone
from ..simulation import Simulator


class SimulationRenderer:
    """Tkinter-based GUI for visualizing drone simulation.
    
    Displays the map, zones, connections, and animates drone movement
    turn by turn.
    """
    
    def __init__(
        self,
        map_data: MapData,
        simulator: Simulator,
        window_width: int = 1000,
        window_height: int = 700
    ) -> None:
        """Initialize the renderer.
        
        Args:
            map_data: The map to visualize
            simulator: The simulator to animate
            window_width: Width of the window
            window_height: Height of the window
        """
        self.map_data = map_data
        self.simulator = simulator
        self.window_width = window_width
        self.window_height = window_height
        
        # Create window
        self.root = tk.Tk()
        self.root.title("Fly-in Drone Simulation")
        self.root.geometry(f"{window_width}x{window_height}")
        
        # Canvas for drawing
        self.canvas_width = window_width - 20
        self.canvas_height = window_height - 150
        self.canvas = tk.Canvas(
            self.root,
            width=self.canvas_width,
            height=self.canvas_height,
            bg="white"
        )
        self.canvas.pack(pady=10)
        
        # Control frame
        self.control_frame = ttk.Frame(self.root)
        self.control_frame.pack(pady=5)
        
        # Buttons
        self.btn_reset = ttk.Button(
            self.control_frame,
            text="Reset",
            command=self.reset_simulation
        )
        self.btn_reset.grid(row=0, column=0, padx=5)
        
        self.btn_step = ttk.Button(
            self.control_frame,
            text="Step",
            command=self.step_simulation
        )
        self.btn_step.grid(row=0, column=1, padx=5)
        
        self.btn_play = ttk.Button(
            self.control_frame,
            text="Play",
            command=self.play_simulation
        )
        self.btn_play.grid(row=0, column=2, padx=5)
        
        self.btn_pause = ttk.Button(
            self.control_frame,
            text="Pause",
            command=self.pause_simulation,
            state="disabled"
        )
        self.btn_pause.grid(row=0, column=3, padx=5)
        
        # Info label
        self.info_label = ttk.Label(
            self.root,
            text="Turn: 0 | Delivered: 0/0",
            font=("Arial", 12)
        )
        self.info_label.pack(pady=5)
        
        # Animation state
        self.current_turn = 0
        self.is_playing = False
        self.play_delay = 500  # milliseconds
        
        # Calculate scaling for map coordinates
        self._calculate_scaling()
        
        # Draw initial state
        self.draw_map()
    
    def _calculate_scaling(self) -> None:
        """Calculate scaling factors to fit map in canvas."""
        if not self.map_data.zones:
            self.scale = 50
            self.offset_x = 50
            self.offset_y = 50
            return
        
        # Find bounds of map coordinates
        xs = [z.x for z in self.map_data.zones.values()]
        ys = [z.y for z in self.map_data.zones.values()]
        
        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)
        
        # Add padding
        padding = 80
        available_width = self.canvas_width - 2 * padding
        available_height = self.canvas_height - 2 * padding
        
        # Calculate scale to fit
        width_range = max_x - min_x if max_x > min_x else 1
        height_range = max_y - min_y if max_y > min_y else 1
        
        scale_x = available_width / width_range
        scale_y = available_height / height_range
        
        self.scale = min(scale_x, scale_y, 100)  # Cap at 100
        
        # Calculate offsets to center
        self.offset_x = padding - min_x * self.scale
        self.offset_y = padding - min_y * self.scale
    
    def _map_to_canvas(self, x: float, y: float) -> Tuple[float, float]:
        """Convert map coordinates to canvas coordinates.
        
        Args:
            x: Map x coordinate
            y: Map y coordinate
            
        Returns:
            Tuple of (canvas_x, canvas_y)
        """
        canvas_x = x * self.scale + self.offset_x
        canvas_y = y * self.scale + self.offset_y
        return (canvas_x, canvas_y)
    
    def draw_map(self) -> None:
        """Draw the complete map with zones, connections, and drones."""
        self.canvas.delete("all")
        
        # Draw connections first (behind zones)
        self._draw_connections()
        
        # Draw zones
        self._draw_zones()
        
        # Draw drones
        self._draw_drones()
        
        # Update info
        self._update_info()
    
    def _draw_connections(self) -> None:
        """Draw all connections as lines."""
        for conn in self.map_data.connections:
            zone1_name, zone2_name = conn.get_zones()
            zone1 = self.map_data.zones[zone1_name]
            zone2 = self.map_data.zones[zone2_name]
            
            x1, y1 = self._map_to_canvas(zone1.x, zone1.y)
            x2, y2 = self._map_to_canvas(zone2.x, zone2.y)
            
            self.canvas.create_line(
                x1, y1, x2, y2,
                fill="gray",
                width=2
            )
            
            # Draw capacity label if > 1
            if conn.max_capacity > 1:
                mid_x = (x1 + x2) / 2
                mid_y = (y1 + y2) / 2
                self.canvas.create_text(
                    mid_x, mid_y,
                    text=f"[{conn.max_capacity}]",
                    fill="blue",
                    font=("Arial", 8)
                )
    
    def _draw_zones(self) -> None:
        """Draw all zones as circles."""
        for zone in self.map_data.zones.values():
            cx, cy = self._map_to_canvas(zone.x, zone.y)
            radius = 20
            
            # Color based on zone type
            color = self._get_zone_color(zone)
            
            # Draw circle
            self.canvas.create_oval(
                cx - radius, cy - radius,
                cx + radius, cy + radius,
                fill=color,
                outline="black",
                width=2
            )
            
            # Draw zone name
            self.canvas.create_text(
                cx, cy - radius - 10,
                text=zone.name,
                font=("Arial", 9, "bold")
            )
            
            # Draw capacity if limited
            if not zone.is_unlimited_capacity:
                self.canvas.create_text(
                    cx, cy + radius + 10,
                    text=f"[{zone.current_occupancy}/{zone.max_capacity}]",
                    font=("Arial", 8),
                    fill="darkblue"
                )
    
    def _get_zone_color(self, zone: Zone) -> str:
        """Get display color for a zone based on its type.
        
        Args:
            zone: The zone to get color for
            
        Returns:
            Color string
        """
        from ..models import ZoneType
        
        type_colors = {
            ZoneType.START: "#90EE90",      # Light green
            ZoneType.END: "#FFD700",        # Gold
            ZoneType.NORMAL: "#87CEEB",     # Sky blue
            ZoneType.PRIORITY: "#98FB98",   # Pale green
            ZoneType.RESTRICTED: "#FFA07A", # Light salmon
            ZoneType.BLOCKED: "#D3D3D3"     # Light gray
        }
        return type_colors.get(zone.zone_type, "#CCCCCC")
    
    def _draw_drones(self) -> None:
        """Draw all drones at their current positions."""
        # Group drones by zone
        drones_per_zone: Dict[str, List[Drone]] = {}
        for drone in self.map_data.drones:
            zone = drone.current_zone
            if zone not in drones_per_zone:
                drones_per_zone[zone] = []
            drones_per_zone[zone].append(drone)
        
        # Draw drones
        for zone_name, drones in drones_per_zone.items():
            zone = self.map_data.zones[zone_name]
            cx, cy = self._map_to_canvas(zone.x, zone.y)
            
            # Arrange drones in a circle around zone center
            for i, drone in enumerate(drones):
                angle = (2 * 3.14159 * i) / len(drones)
                import math
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
    
    def _update_info(self) -> None:
        """Update the information label."""
        delivered = sum(
            1 for d in self.map_data.drones
            if d.current_zone == self.map_data.end_zone_name
        )
        total = len(self.map_data.drones)
        
        self.info_label.config(
            text=f"Turn: {self.current_turn} | Delivered: {delivered}/{total}"
        )
    
    def reset_simulation(self) -> None:
        """Reset the simulation to initial state."""
        self.pause_simulation()
        self.current_turn = 0
        self.map_data.reset_state()
        self.draw_map()
    
    def step_simulation(self) -> None:
        """Execute one simulation turn."""
        if self._is_finished():
            return
        
        self.simulator._execute_turn()
        self.current_turn += 1
        self.draw_map()
    
    def play_simulation(self) -> None:
        """Start automatic playback."""
        self.is_playing = True
        self.btn_play.config(state="disabled")
        self.btn_pause.config(state="normal")
        self._play_step()
    
    def pause_simulation(self) -> None:
        """Pause automatic playback."""
        self.is_playing = False
        self.btn_play.config(state="normal")
        self.btn_pause.config(state="disabled")
    
    def _play_step(self) -> None:
        """Execute one step during playback."""
        if not self.is_playing:
            return
        
        if self._is_finished():
            self.pause_simulation()
            return
        
        self.step_simulation()
        self.root.after(self.play_delay, self._play_step)
    
    def _is_finished(self) -> bool:
        """Check if simulation is finished."""
        return all(
            d.current_zone == self.map_data.end_zone_name
            for d in self.map_data.drones
        )
    
    def run(self) -> None:
        """Start the GUI main loop."""
        self.root.mainloop()
