from src.parser import Parser
from src.graph import Graph
import sys


def main() -> None:
    if len(sys.argv) != 2:
        raise ValueError("The map file has missing!!.")
    map_file = sys.argv[1]
    map_parser = Parser()
    map_data = map_parser.parse(map_file)
    # graph = Graph(**map_data)


main()
