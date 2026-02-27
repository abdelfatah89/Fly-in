from map_parser import MapParser

if __name__ == "__main__":
    parser = MapParser('map.txt')
    map_data = parser.parse_map()
    for key, value in map_data.items():
        print(f"{key}: {value}")