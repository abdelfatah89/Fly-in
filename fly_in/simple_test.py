from map_parser import MapParser
from simulator import Simulator
from drone import DroneStatus

# Load the map
graph = MapParser("map.txt").graph
simulator = Simulator(graph)

print(f"Loaded {len(graph.zones)} zones, {len(graph.drones)} drones")
print(f"Start: {graph.start_zone.name}, Goal: {graph.end_zone.name}")
print(f"First drone at: {graph.drones[0].current_zone}")
print(f"All delivered initially? {simulator.all_drones_delivered()}")

print("\nRunning 10 turns...")
for i in range(10):
    simulator.execute_turn()
    print(f"Turn {i+1} complete. Delivered? {simulator.all_drones_delivered()}")
