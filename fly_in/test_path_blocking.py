"""Tests for temporarily_block_connection, restore_connection,
restore_all_connections, and find_k_paths in the Dijkstra class."""

import sys
import os
sys.path.insert(0, os.path.dirname(__file__))

from map_parser import MapParser
from path_finding import Dijkstra

# ── helpers ──────────────────────────────────────────────────────────────────

def load_graph(map_file: str):
    path = os.path.join(os.path.dirname(__file__), map_file)
    return MapParser(path).graph


def assert_equal(label, got, expected):
    if got != expected:
        print(f"  FAIL [{label}]: expected {expected!r}, got {got!r}")
        return False
    print(f"  PASS [{label}]")
    return True


def assert_true(label, condition):
    if not condition:
        print(f"  FAIL [{label}]: condition was False")
        return False
    print(f"  PASS [{label}]")
    return True


# ── tests ─────────────────────────────────────────────────────────────────────

def test_temporarily_block_connection():
    """Blocking a connection causes find_path to avoid it."""
    print("\n--- test_temporarily_block_connection ---")
    ok = True
    graph = load_graph("map.txt")
    d = Dijkstra(graph)

    # Baseline path (unblocked)
    baseline = d.find_path(graph.start_zone.name, graph.end_zone.name)
    ok &= assert_true("baseline path found", baseline is not None)

    # Block a connection that is on the baseline path
    if baseline and len(baseline) >= 2:
        z1, z2 = baseline[0], baseline[1]
        blocked = d.temporarily_block_connection(z1, z2)
        ok &= assert_equal("block returns True for known connection", blocked, True)

        # The new path should differ from baseline
        new_path = d.find_path(graph.start_zone.name, graph.end_zone.name)
        if new_path is not None:
            ok &= assert_true(
                "new path avoids blocked edge",
                not (z1 in new_path and z2 in new_path
                     and abs(new_path.index(z1) - new_path.index(z2)) == 1)
            )
        # Clean up
        d.restore_all_connections()

    return ok


def test_block_nonexistent_connection():
    """Blocking a non-existent connection returns False."""
    print("\n--- test_block_nonexistent_connection ---")
    graph = load_graph("map.txt")
    d = Dijkstra(graph)
    # start and end_zone are many hops apart and not directly connected
    result = d.temporarily_block_connection(
        graph.start_zone.name, graph.end_zone.name
    )
    return assert_equal("block returns False for unknown connection", result, False)


def test_restore_connection():
    """restore_connection re-enables a single blocked connection."""
    print("\n--- test_restore_connection ---")
    ok = True
    graph = load_graph("map.txt")
    d = Dijkstra(graph)

    baseline = d.find_path(graph.start_zone.name, graph.end_zone.name)
    ok &= assert_true("baseline path found", baseline is not None)

    if baseline and len(baseline) >= 2:
        z1, z2 = baseline[0], baseline[1]
        d.temporarily_block_connection(z1, z2)

        # Restore that connection
        restored = d.restore_connection(z1, z2)
        ok &= assert_equal("restore returns True for known connection", restored, True)

        # Path should be the same as baseline now
        recovered = d.find_path(graph.start_zone.name, graph.end_zone.name)
        ok &= assert_equal("path restored to baseline", recovered, baseline)

    return ok


def test_restore_nonexistent_connection():
    """restore_connection on an unknown pair returns False."""
    print("\n--- test_restore_nonexistent_connection ---")
    graph = load_graph("map.txt")
    d = Dijkstra(graph)
    # start and end_zone are many hops apart and not directly connected
    result = d.restore_connection(
        graph.start_zone.name, graph.end_zone.name
    )
    return assert_equal("restore returns False for unknown connection", result, False)


def test_restore_all_connections():
    """restore_all_connections clears all blocks at once."""
    print("\n--- test_restore_all_connections ---")
    ok = True
    graph = load_graph("map.txt")
    d = Dijkstra(graph)

    baseline = d.find_path(graph.start_zone.name, graph.end_zone.name)

    # Block several connections along the baseline path
    if baseline and len(baseline) >= 3:
        d.temporarily_block_connection(baseline[0], baseline[1])
        d.temporarily_block_connection(baseline[1], baseline[2])
        ok &= assert_true("blocked connections set is non-empty",
                          len(d._blocked_connections) > 0)

        d.restore_all_connections()
        ok &= assert_equal("blocked set cleared", len(d._blocked_connections), 0)

        recovered = d.find_path(graph.start_zone.name, graph.end_zone.name)
        ok &= assert_equal("path restored to baseline", recovered, baseline)

    return ok


def test_find_k_paths_challenge_map():
    """find_k_paths returns 3 distinct paths on the challenge map."""
    print("\n--- test_find_k_paths_challenge_map ---")
    ok = True
    map_path = os.path.join(os.path.dirname(__file__),
                            "../maps/challenger/01_the_impossible_dream.txt")
    graph = MapParser(map_path).graph
    d = Dijkstra(graph)

    paths = d.find_k_paths(graph.start_zone.name, graph.end_zone.name, 3)
    ok &= assert_true("at least 1 path found", len(paths) >= 1)
    ok &= assert_true("at most 3 paths found", len(paths) <= 3)

    # All paths must start at start and end at goal
    for i, p in enumerate(paths):
        ok &= assert_equal(f"path {i} starts correctly",
                           p[0], graph.start_zone.name)
        ok &= assert_equal(f"path {i} ends correctly",
                           p[-1], graph.end_zone.name)

    # All paths must be distinct
    ok &= assert_equal("paths are distinct", len(paths), len(set(map(tuple, paths))))

    # Blocked connections should be cleared after find_k_paths
    ok &= assert_equal("blocked connections cleared after find_k_paths",
                       len(d._blocked_connections), 0)

    print(f"  Found {len(paths)} distinct path(s):")
    for i, p in enumerate(paths):
        print(f"    Path {i + 1}: {' -> '.join(p)}")

    return ok


def test_find_k_paths_current_map():
    """find_k_paths on the default map returns valid distinct paths."""
    print("\n--- test_find_k_paths_current_map ---")
    ok = True
    graph = load_graph("map.txt")
    d = Dijkstra(graph)

    paths = d.find_k_paths(graph.start_zone.name, graph.end_zone.name, 3)
    ok &= assert_true("found at least 1 path", len(paths) >= 1)
    for i, p in enumerate(paths):
        ok &= assert_equal(f"path {i} starts at start",
                           p[0], graph.start_zone.name)
        ok &= assert_equal(f"path {i} ends at goal",
                           p[-1], graph.end_zone.name)

    # Distinct paths
    ok &= assert_equal("paths are distinct", len(paths), len(set(map(tuple, paths))))

    # No blocked connections remain
    ok &= assert_equal("no blocked connections left",
                       len(d._blocked_connections), 0)

    print(f"  Found {len(paths)} path(s):")
    for i, p in enumerate(paths):
        print(f"    Path {i + 1}: {' -> '.join(p)}")

    return ok


# ── main ──────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    results = [
        test_temporarily_block_connection(),
        test_block_nonexistent_connection(),
        test_restore_connection(),
        test_restore_nonexistent_connection(),
        test_restore_all_connections(),
        test_find_k_paths_current_map(),
        test_find_k_paths_challenge_map(),
    ]
    passed = sum(results)
    total = len(results)
    print(f"\n{'='*40}")
    print(f"Results: {passed}/{total} test groups passed")
    if passed < total:
        sys.exit(1)
