"""Fly-in: Turn-based drone delivery simulator entry point."""
from src.parser import Parser
from src.graph import Graph
from src.simulator import Simulator
from src.renderer import Renderer
import sys
import os


def main() -> None:
    """Parse the map file and launch the simulation with visual renderer."""
    try:
        maps = {
            "--easy1": "maps/easy/01_linear_path.txt",
            "--easy2": "maps/easy/02_simple_fork.txt",
            "--easy3": "maps/easy/03_basic_capacity.txt",
            "--medium1": "maps/medium/01_dead_end_trap.txt",
            "--medium2": "maps/medium/02_circular_loop.txt",
            "--medium3": "maps/medium/03_priority_puzzle.txt",
            "--hard1": "maps/hard/01_maze_nightmare.txt",
            "--hard2": "maps/hard/02_capacity_hell.txt",
            "--hard3": "maps/hard/03_ultimate_challenge.txt",
            "--challenger": "maps/challenger/01_the_impossible_dream.txt",
        }

        if len(sys.argv) != 2:
            print("Usage: python3 fly-in.py <map_file>")
            sys.exit(1)

        map_file = sys.argv[1]
        if map_file.startswith("--"):
            for map_name, map_path in maps.items():
                if map_file == map_name:
                    map_file = map_path
                    break
        elif not os.path.exists(map_file):
            print(f"Map file {map_file} does not exist.")
            sys.exit(1)

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
