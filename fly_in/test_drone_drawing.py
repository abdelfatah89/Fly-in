from map_parser import MapParser
from simulator import Simulator
from drone import DroneStatus

# Load the map
graph = MapParser("map.txt").graph
simulator = Simulator(graph)

print("Testing drone drawing logic...")
print(f"Total drones: {len(graph.drones)}")
print(f"Total zones: {len(graph.zones)}")
print()

# Run a few simulation turns
for turn in range(15):
    if simulator.all_drones_delivered():
        print(f"All drones delivered at turn {turn}")
        break
        
    simulator.execute_turn()
    
    # Check for drones in transit
    in_transit = [d for d in graph.drones if d.status == DroneStatus.IN_TRANSIT]
    
    if in_transit:
        print(f"Turn {turn + 1}:")
        print(f"  Drones in transit: {len(in_transit)}")
        for drone in in_transit[:3]:  # Show first 3
            print(f"    Drone {drone.drone_id}: current_zone={drone.current_zone}, transit_dest={drone.transit_destination}, turns_remaining={drone.transit_turns_remaining}")
        
        # Check conv_restricted3 specifically
        conv_r3_transit = [d for d in in_transit if d.transit_destination == "conv_restricted3"]
        if conv_r3_transit:
            print(f"  ** Drones in transit to conv_restricted3: {len(conv_r3_transit)} **")
        print()

# Final check for conv_restricted3
conv_restricted3_drones = [d for d in graph.drones if d.current_zone == "conv_restricted3"]
conv_restricted3_transit = [d for d in graph.drones if d.status == DroneStatus.IN_TRANSIT and d.transit_destination == "conv_restricted3"]

print(f"\nFinal state at conv_restricted3:")
print(f"  Drones WAITING there: {len(conv_restricted3_drones)}")
print(f"  Drones IN_TRANSIT to there: {len(conv_restricted3_transit)}")
print(f"  Total that should be drawn: {len(conv_restricted3_drones) + len(conv_restricted3_transit)}")

if conv_restricted3_drones:
    print(f"  Waiting drones: {[d.drone_id for d in conv_restricted3_drones]}")
if conv_restricted3_transit:
    print(f"  Transit drones: {[d.drone_id for d in conv_restricted3_transit]}")
