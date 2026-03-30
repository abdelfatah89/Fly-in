from map_parser import MapParser
from simulator import Simulator
from drone import DroneStatus

# Load the map
graph = MapParser("map.txt").graph
simulator = Simulator(graph)

print("Testing capacity violation in conv_restricted zones...")
print(f"conv_restricted1 max_capacity: {graph.zones['conv_restricted1'].max_capacity}")
print(f"conv_restricted2 max_capacity: {graph.zones['conv_restricted2'].max_capacity}")
print(f"conv_restricted3 max_capacity: {graph.zones['conv_restricted3'].max_capacity}")
print(f"\nDrone starting positions:")
for d in graph.drones[:5]:  # Show first 5 drones
    print(f"  Drone {d.drone_id}: {d.current_zone} (status: {d.status})")
print(f"  ... ({len(graph.drones)} total drones)")
print(f"\nGoal zone: {graph.end_zone.name}")
print()

# Run simulation and watch for violations
for turn in range(50):
    if simulator.all_drones_delivered():
        print(f"All drones delivered at turn {turn}")
        break
    
    simulator.execute_turn()
    
    # Check each restricted zone
    for zone_name in ['conv_restricted1', 'conv_restricted2', 'conv_restricted3']:
        zone = graph.zones[zone_name]
        
        # Count drones at this zone (WAITING)
        drones_here = [d for d in graph.drones if d.current_zone == zone_name and d.status == DroneStatus.WAITING]
        
        # Count drones in transit to this zone
        drones_in_transit = [d for d in graph.drones if d.status == DroneStatus.IN_TRANSIT and d.transit_destination == zone_name]
        
        total = len(drones_here) + len(drones_in_transit)
        
        if total > zone.max_capacity:
            print(f"⚠️  CAPACITY VIOLATION at Turn {turn + 1}: {zone_name}")
            print(f"   Max capacity: {zone.max_capacity}")
            print(f"   Zone occupancy: {zone.current_occupancy}")
            print(f"   Drones WAITING here: {len(drones_here)} - IDs: {[d.drone_id for d in drones_here]}")
            print(f"   Drones IN_TRANSIT here: {len(drones_in_transit)} - IDs: {[d.drone_id for d in drones_in_transit]}")
            print(f"   Total drones: {total}")
            print()
        elif total > 0:
            print(f"Turn {turn + 1}: {zone_name}: {len(drones_here)} waiting + {len(drones_in_transit)} in transit = {total}/{zone.max_capacity}")

print("\n=== Simulation Complete ===")
