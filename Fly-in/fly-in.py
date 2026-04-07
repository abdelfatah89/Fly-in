from src.parser import Parser
from src.graph import Graph
from src.simulator import Simulator
import sys


def main() -> None:
    if len(sys.argv) != 2:
        raise ValueError("The map file has missing!!.")
    map_file = sys.argv[1]
    map_parser = Parser()
    map_data = map_parser.parse(map_file)
    if map_data is None:
        print("Failed to parse the map file.")
        return

    graph = Graph(**map_data)
    sim = Simulator(graph)

    lines = sim.run()
    for line in lines:
        print(line)
    print("Number of Turns", len(lines))


if __name__ == "__main__":
    main()
