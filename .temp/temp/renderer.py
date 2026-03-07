import tkinter as tk


class GraphUI:
    def __init__(self, graph):
        self.graph = graph

        self.root = tk.Tk()
        self.root.title("Fly-in Drone Simulation")

        self.canvas_width = 1500
        self.canvas_height = 700

        self.canvas = tk.Canvas(
            self.root,
            width=self.canvas_width,
            height=self.canvas_height,
            bg="white"
        )
        self.draw_connections()
        self.draw_zones()
        self.canvas.pack()

    def _calculate_scale(self):
        max_x = max(zone.x for zone in self.graph.zones.values())
        max_y = max(zone.y for zone in self.graph.zones.values())

        padding = 50

        scale_x = (self.canvas_width - padding) / (max_x + 1)
        scale_y = (self.canvas_height - padding) / (max_y + 1)

        return min(scale_x, scale_y)

    def draw_zones(self):
        scale = self._calculate_scale()
        radius = 25

        for zone in self.graph.zones.values():
            x = zone.x * scale + 50
            y = zone.y * scale + 50

            self.canvas.create_oval(
                x - radius,
                y - radius,
                x + radius,
                y + radius,
                fill=zone.color if zone.color != "none" else "lightgray"
            )

            self.canvas.create_rectangle(
                x - 40,
                y - 35,
                x + 40,
                y - 15,
                fill="white",
                outline=""
            )
            self.canvas.create_text(
                x,
                y - 25,
                text=f"{zone.name} # {zone.max_drones}",
                fill="black",

            )

    def draw_connections(self):
        scale = self._calculate_scale()

        for conn in self.graph.connections:
            zone1 = self.graph.zones[conn.zone1]
            zone2 = self.graph.zones[conn.zone2]

            x1 = zone1.x * scale + 50
            y1 = zone1.y * scale + 50
            x2 = zone2.x * scale + 50
            y2 = zone2.y * scale + 50

            self.canvas.create_line(
                x1,
                y1,
                x2,
                y2,
                width=2
            )
            if conn.max_capacity > 1:
                offset = 10
                mid_x = (x1 + x2) / 2
                mid_y = (y1 + y2) / 2
                mid_y -= offset

                self.canvas.create_text(
                    mid_x,
                    mid_y,
                    text=str(conn.max_capacity),
                    fill="red"
                )

    def run(self):
        self.root.mainloop()