from tkinter import Tk, Canvas
from .graph import Graph
from .simulator import Simulator
from .zone import Zone
from .drone import Drone


class Renderer(Tk):
    def __init__(self, graph: Graph, sim: Simulator) -> None:
        # --- init variables --- #
        self.width = 800
        self.height = 600

        # --- init Window --- #
        super().__init__()
        self.title("Drone Delivery Simulator")
        self.geometry("800x600")
        self.configure(bg="white")

        # --- init Simulator and Graph variables --- #
        self.sim = sim
        self.graph = graph

        # --- init canvas --- #
        self.canvas = Canvas(self,
                             width=self.width,
                             height=self.height,
                             bg="white")
        self.canvas.pack()

    def draw_circle(self, radius: int, object: Zone | Drone) -> None:
        if isinstance(object, Zone):
            self.canvas.create_oval(object.x - radius, object.y - radius,
                                    object.x + radius, object.y + radius,
                                    fill=object.color)
        elif isinstance(object, Drone):
            self.canvas.create_oval(object.current_zone.x - radius,
                                    object.current_zone.y - radius,
                                    object.current_zone.x + radius,
                                    object.current_zone.y + radius,
                                    fill="red", outline="darkred", width=1)

    def draw_line(self, zone1: Zone, zone2: Zone) -> None:
        self.canvas.create_line(zone1.x, zone1.y,
                                zone2.x, zone2.y,
                                fill="black", width=2)

    def draw_graph(self) -> None:
        for zone in self.graph.zones.values():
            self.draw_circle(25, zone)
        for connection in self.graph.connections:
            self.draw_line(self.graph.zones[connection.zone1],
                           self.graph.zones[connection.zone2])
        for drone in self.graph.drones:
            self.draw_circle(5, drone)

    @property
    def _is_simulation_finished(self) -> bool:
        return all(drone.is_reached for drone in self.graph.drones)

    def update(self) -> None:
        if self._is_simulation_finished:
            return
        self.canvas.delete("all")
        self.sim.execute_turn()
        self.draw_graph()
        self.after(100, self.update)
