from map_parser import MapParser
from renderer import GraphUI


if __name__ == "__main__":
    parser = MapParser("map.txt")
    graph = parser.graph

    ui = GraphUI(graph)
    ui.run()