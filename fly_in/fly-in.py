from map_parser import MapParser
from simulator import Simulator
from renderer import Renderer

def main():
    map_parser = MapParser("map.txt")
    graph = map_parser.graph
    simulator = Simulator(graph)
    renderer = Renderer(graph, simulator)
    renderer.run()
    print(f"Map loaded: {len(graph.zones)} zones, {len(graph.drones)} drones")
    print(f"Path: {graph.start_zone.name} -> {graph.end_zone.name}")
    print("Running simulation...")
    output_lines = simulator.run()
    print(f"\n{'='*60}")
    print(f"Simulation completed in {simulator.get_turn_count()} turns")
    print(f"{'='*60}\n")
    for line in output_lines:
        print(line)
    simulator.print_summary()

if __name__ == "__main__":
    main()