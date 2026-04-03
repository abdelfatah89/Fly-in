from map_parser import MapParser, Graph
from simulator import Simulator
from renderer import Renderer

#print("Starting test...")
#map_parser = MapParser("map.txt")
#print(f"Zones loaded: {len(map_parser.graph.zones)}")
#print(f"Connections: {len(map_parser.graph.connections)}")
#print(f"Start zone: {map_parser.graph.start_zone.name}")
#print(f"End zone: {map_parser.graph.end_zone.name}")
#print(f"Drones: {len(map_parser.graph.drones)}")
#print(f"First drone current_zone: {map_parser.graph.drones[0].current_zone}")
#print(f"First drone current_zone type: {type(map_parser.graph.drones[0].current_zone)}")

#print("\nCreating simulator...")
#simulator = Simulator(map_parser.graph)
#print("Simulator created successfully!")

print("\nCreating renderer...")
graph = MapParser("map.txt").graph
simulator = Simulator(graph)
renderer = Renderer(graph, simulator)
renderer.run()
print("Renderer created successfully!")

#print("\nRunning simulation...")
#simulator.run()
#print("Simulation complete!")
#simulator.print_summary()

#print("\nRendering graph...")
#renderer.run()
#print("Rendering complete!")
