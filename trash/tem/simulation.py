from zone import Zone, Connection
from typing import Dict, List
from drone import Drone


class Graph:
    def __init__(self, start_zone: Zone, end_zone: Zone):
        self.zones: Dict[str, Zone] = {}
        self.connections: List[Connection] = []
        self.start_zone: Zone = start_zone
        self.end_zone: Zone = end_zone


class Simulation:
    def __init__(self, graph: Graph):
        self.graph: Graph = graph
        self.drones: Dict[str, Drone] = {}
