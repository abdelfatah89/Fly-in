"""Fly-in: Turn-based drone delivery simulator entry point."""
from src.parser import Parser
from src.graph import Graph
from src.simulator import Simulator
from src.renderer import Renderer
import sys


def main() -> None:
    """Parse the map file and launch the simulation with visual renderer."""
    try:
        if len(sys.argv) != 2:
            print("Usage: python3 fly-in.py <map_file>")
            sys.exit(1)
        map_file = sys.argv[1]
        map_parser = Parser()
        map_data = map_parser.parse(map_file)
        if map_data is None:
            print("Failed to parse the map file.")
            sys.exit(1)

        graph = Graph(**map_data)
        sim = Simulator(graph)
        renderer = Renderer(graph, sim)

        renderer.animate()
        renderer.mainloop()
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
