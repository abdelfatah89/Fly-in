"""Test suite for priority zone pathfinding behavior.

Test 1 — Priority neighbor route (1 extra hop) MUST be preferred.
Test 2 — Same-length routes: priority MUST be preferred.
Test 3 — Long detour through many priority zones but adding extra
         non-priority hops must NOT be preferred over a shorter route.
"""
import sys
sys.path.insert(0, ".")

from src.zone import Zone, ZoneType
from src.connection import Connection
from src.graph import Graph
from src.path_finding import Dijkstra


def test_priority_neighbor_preferred():
    """A priority neighbor adds 1 hop but should still be chosen."""
    print("TEST 1: Priority neighbor (1 extra hop) vs shorter normal")
    print("-" * 55)

    start = Zone("start", 0, 0, ZoneType.NORMAL, max_capacity=10)
    goal = Zone("goal", 5, 0, ZoneType.NORMAL, max_capacity=10)
    normal_a = Zone("normal_A", 1, 0, ZoneType.NORMAL)
    normal_b = Zone("normal_B", 2, 0, ZoneType.NORMAL)
    prio_c = Zone("prio_C", 1, 1, ZoneType.PRIORITY)
    normal_d = Zone("normal_D", 2, 1, ZoneType.NORMAL)
    normal_e = Zone("normal_E", 3, 1, ZoneType.NORMAL)

    zones = {z.name: z for z in [start, goal, normal_a, normal_b,
                                  prio_c, normal_d, normal_e]}
    connections = [
        Connection("start", "normal_A", 1),
        Connection("normal_A", "normal_B", 1),
        Connection("normal_B", "goal", 1),
        Connection("start", "prio_C", 1),
        Connection("prio_C", "normal_D", 1),
        Connection("normal_D", "normal_E", 1),
        Connection("normal_E", "goal", 1),
    ]

    graph = Graph(nb_drones=1, start_hub=start, end_hub=goal,
                  zones=zones, connections=connections)
    path = Dijkstra(graph).find_path(start, goal)
    names = [z.name for z in path]
    hops = len(path) - 1

    print(f"  Route chosen: {' -> '.join(names)}  ({hops} hops)")
    ok = "prio_C" in names
    print(f"  {'PASS' if ok else 'FAIL'}: "
          f"{'Priority route chosen' if ok else 'Normal route chosen!'}")
    return ok


def test_same_length_tiebreaker():
    """When hop count is equal, priority route must be preferred."""
    print()
    print("TEST 2: Same-length routes — normal vs priority")
    print("-" * 55)

    start = Zone("start", 0, 0, ZoneType.NORMAL, max_capacity=10)
    goal = Zone("goal", 3, 0, ZoneType.NORMAL, max_capacity=10)
    normal_a = Zone("normal_A", 1, 0, ZoneType.NORMAL)
    prio_b = Zone("prio_B", 1, 1, ZoneType.PRIORITY)

    zones = {z.name: z for z in [start, goal, normal_a, prio_b]}
    connections = [
        Connection("start", "normal_A", 1),
        Connection("normal_A", "goal", 1),
        Connection("start", "prio_B", 1),
        Connection("prio_B", "goal", 1),
    ]

    graph = Graph(nb_drones=1, start_hub=start, end_hub=goal,
                  zones=zones, connections=connections)
    path = Dijkstra(graph).find_path(start, goal)
    names = [z.name for z in path]

    print(f"  Route chosen: {' -> '.join(names)}")
    ok = "prio_B" in names
    print(f"  {'PASS' if ok else 'FAIL'}: "
          f"{'Priority route preferred' if ok else 'Normal route chosen!'}")
    return ok


def test_long_priority_detour_rejected():
    """A detour adding extra non-priority hops must not be chosen."""
    print()
    print("TEST 3: Priority detour with extra non-priority hops")
    print("-" * 55)

    start = Zone("start", 0, 0, ZoneType.NORMAL, max_capacity=10)
    goal = Zone("goal", 5, 0, ZoneType.NORMAL, max_capacity=10)
    n_a = Zone("n_A", 1, 0, ZoneType.NORMAL)
    n_b = Zone("n_B", 2, 0, ZoneType.NORMAL)
    n_c = Zone("n_C", 1, 1, ZoneType.NORMAL)
    n_d = Zone("n_D", 2, 1, ZoneType.NORMAL)
    prio_x = Zone("prio_X", 3, 1, ZoneType.PRIORITY)
    n_e = Zone("n_E", 4, 1, ZoneType.NORMAL)

    zones = {z.name: z for z in [start, goal, n_a, n_b,
                                  n_c, n_d, prio_x, n_e]}
    connections = [
        Connection("start", "n_A", 1),
        Connection("n_A", "n_B", 1),
        Connection("n_B", "goal", 1),
        Connection("start", "n_C", 1),
        Connection("n_C", "n_D", 1),
        Connection("n_D", "prio_X", 1),
        Connection("prio_X", "n_E", 1),
        Connection("n_E", "goal", 1),
    ]

    graph = Graph(nb_drones=1, start_hub=start, end_hub=goal,
                  zones=zones, connections=connections)
    path = Dijkstra(graph).find_path(start, goal)
    names = [z.name for z in path]
    hops = len(path) - 1

    print(f"  Route chosen: {' -> '.join(names)}  ({hops} hops)")
    ok = hops == 3 and "n_A" in names
    print(f"  {'PASS' if ok else 'FAIL'}: "
          f"{'Shorter route chosen (extra non-prio hops rejected)' if ok else 'Longer detour chosen!'}")
    return ok


if __name__ == "__main__":
    r1 = test_priority_neighbor_preferred()
    r2 = test_same_length_tiebreaker()
    r3 = test_long_priority_detour_rejected()
    print()
    print("=" * 55)
    all_ok = r1 and r2 and r3
    print(f"Results: {'ALL PASS' if all_ok else 'SOME FAILED'}")
    print("=" * 55)
