from _typeshed import AnyStr_co
import os
import re
import glob
import pytest
from typing import List, Tuple
from src.parser import Parser
from src.graph import Graph
from src.simulator import Simulator
from src.path_finding import Dijkstra

MAPS_DIR = os.path.join(os.path.dirname(__file__), "..", "maps")
MAP_ROOT = os.path.join(os.path.dirname(__file__), "..", "map.txt")

MAX_TURNS = {
    "challenger": 200,
    "hard": 150,
    "medium": 100,
    "easy": 50,
}


def _collect_map_files() -> List[str]:
    """Discover every .txt map file under maps/."""
    pattern = os.path.join(MAPS_DIR, "**", "*.txt")
    return sorted(glob.glob(pattern, recursive=True))


def _difficulty(path: str) -> str:
    rel = os.path.relpath(path, MAPS_DIR)
    return rel.split(os.sep)[0]


def _max_turns_for(path: str) -> int:
    return MAX_TURNS.get(_difficulty(path), 100)


MAP_FILES = _collect_map_files()
if os.path.isfile(MAP_ROOT):
    MAP_FILES.append(MAP_ROOT)

MAP_IDS = [os.path.relpath(p, os.path.join(MAPS_DIR, "..")) for p in MAP_FILES]

MOVE_TOKEN_RE = re.compile(r"^D(\d+)-[\w-]+$")


# ═══════════════════════════════════════════════════════════════════════════════
#  Parsing tests — every map must parse without error
# ═══════════════════════════════════════════════════════════════════════════════


@pytest.mark.parametrize("map_path", MAP_FILES, ids=MAP_IDS)
class TestMapParsing:

    def test_parser_succeeds(self, map_path: str) -> None:
        data = Parser().parse(map_path)
        assert data is not None, f"Parser returned None for {map_path}"

    def test_has_start_hub(self, map_path: str) -> None:
        data = Parser().parse(map_path)
        assert data is not None
        assert data["start_hub"] is not None

    def test_has_end_hub(self, map_path: str) -> None:
        data = Parser().parse(map_path)
        assert data is not None
        assert data["end_hub"] is not None

    def test_has_positive_drones(self, map_path: str) -> None:
        data = Parser().parse(map_path)
        assert data is not None
        assert data["nb_drones"] > 0

    def test_has_zones(self, map_path: str) -> None:
        data = Parser().parse(map_path)
        assert data is not None
        assert len(data["zones"]) >= 2, "Need at least start and end zones"

    def test_has_connections(self, map_path: str) -> None:
        data = Parser().parse(map_path)
        assert data is not None
        assert len(data["connections"]) >= 1

    def test_start_and_end_are_different(self, map_path: str) -> None:
        data = Parser().parse(map_path)
        assert data is not None
        assert data["start_hub"].name != data["end_hub"].name


# ═══════════════════════════════════════════════════════════════════════════════
#  Graph construction tests
# ═══════════════════════════════════════════════════════════════════════════════


@pytest.mark.parametrize("map_path", MAP_FILES, ids=MAP_IDS)
class TestMapGraph:

    def _build(self, map_path: str) -> Graph:
        data = Parser().parse(map_path)
        assert data is not None
        return Graph(**data)

    def test_graph_builds(self, map_path: str) -> None:
        graph = self._build(map_path)
        assert graph is not None

    def test_drones_created(self, map_path: str) -> None:
        graph = self._build(map_path)
        assert len(graph.drones) == graph.nb_drones

    def test_all_drones_start_at_hub(self, map_path: str) -> None:
        graph = self._build(map_path)
        for drone in graph.drones:
            assert drone.current_zone is graph.start_hub

    def test_adjacency_exists(self, map_path: str) -> None:
        graph = self._build(map_path)
        assert len(graph.adjacency) > 0

    def test_connections_reference_valid_zones(self, map_path: str) -> None:
        graph = self._build(map_path)
        for conn in graph.connections:
            assert conn.zone1 in graph.zones, f"Unknown zone {conn.zone1}"
            assert conn.zone2 in graph.zones, f"Unknown zone {conn.zone2}"


# ═══════════════════════════════════════════════════════════════════════════════
#  Pathfinding tests — a path must exist from start to end
# ═══════════════════════════════════════════════════════════════════════════════


@pytest.mark.parametrize("map_path", MAP_FILES, ids=MAP_IDS)
class TestMapPathfinding:

    def _setup(self, map_path: str) -> Tuple[Graph, Dijkstra]:
        data = Parser().parse(map_path)
        assert data is not None
        graph = Graph(**data)
        return graph, Dijkstra(graph)

    def test_path_exists(self, map_path: str) -> None:
        graph, pf = self._setup(map_path)
        path = pf.find_path(graph.start_hub, graph.end_hub)
        assert len(path) >= 2, "Path must have at least start and end"

    def test_path_starts_at_start_hub(self, map_path: str) -> None:
        graph, pf = self._setup(map_path)
        path = pf.find_path(graph.start_hub, graph.end_hub)
        assert path[0].name == graph.start_hub.name

    def test_path_ends_at_end_hub(self, map_path: str) -> None:
        graph, pf = self._setup(map_path)
        path = pf.find_path(graph.start_hub, graph.end_hub)
        assert path[-1].name == graph.end_hub.name

    def test_path_has_no_blocked_zones(self, map_path: str) -> None:
        graph, pf = self._setup(map_path)
        path = pf.find_path(graph.start_hub, graph.end_hub)
        from src.zone import ZoneType
        for zone in path:
            assert zone.zone_type != ZoneType.BLOCKED

    def test_consecutive_zones_are_connected(self, map_path: str) -> None:
        graph, pf = self._setup(map_path)
        path = pf.find_path(graph.start_hub, graph.end_hub)
        for i in range(len(path) - 1):
            conn = graph.get_connection(path[i], path[i + 1])
            assert conn is not None, (
                f"No connection between {path[i].name} and {path[i+1].name}")


# ═══════════════════════════════════════════════════════════════════════════════
#  Simulation tests — full end-to-end run
# ═══════════════════════════════════════════════════════════════════════════════


@pytest.mark.parametrize("map_path", MAP_FILES, ids=MAP_IDS)
class TestMapSimulation:

    def _run(self, map_path: str) -> Tuple[Graph, Simulator]:
        data = Parser().parse(map_path)
        assert data is not None
        graph = Graph(**data)
        sim = Simulator(graph)
        max_t = _max_turns_for(map_path)
        sim.run(max_turns=max_t)
        return graph, sim

    def test_all_drones_delivered(self, map_path: str) -> None:
        graph, sim = self._run(map_path)
        assert sim.all_delivered(), (
            f"Not all drones delivered after {sim.current_turn} turns "
            f"(output lines: {len(sim.output_lines)})")

    def test_all_drones_at_goal(self, map_path: str) -> None:
        graph, sim = self._run(map_path)
        for drone in graph.drones:
            assert drone.current_zone.name == graph.end_hub.name, (
                f"D{drone.drone_id} ended at {drone.current_zone.name} "
                f"instead of {graph.end_hub.name}")

    def test_output_lines_not_empty(self, map_path: str) -> None:
        _, sim = self._run(map_path)
        assert len(sim.output_lines) > 0

    def test_turn_count_positive(self, map_path: str) -> None:
        _, sim = self._run(map_path)
        assert sim.current_turn > 0

    def test_output_lines_count_within_turns(self, map_path: str) -> None:
        """Output lines (moves) should not exceed total turns."""
        _, sim = self._run(map_path)
        assert len(sim.output_lines) <= sim.current_turn


# ═══════════════════════════════════════════════════════════════════════════════
#  Output format validation
# ═══════════════════════════════════════════════════════════════════════════════


@pytest.mark.parametrize("map_path", MAP_FILES, ids=MAP_IDS)
class TestMapOutputFormat:

    def _run(self, map_path: str) -> Tuple[Graph, Simulator]:
        data = Parser().parse(map_path)
        assert data is not None
        graph = Graph(**data)
        sim = Simulator(graph)
        sim.run(max_turns=_max_turns_for(map_path))
        return graph, sim

    def test_each_token_matches_format(self, map_path: str) -> None:
        """Every move token must look like D{id}-{zone} or D{id}-{z1}-{z2}."""
        _, sim = self._run(map_path)
        for line in sim.output_lines:
            for token in line.split():
                assert MOVE_TOKEN_RE.match(token), (
                    f"Invalid token format: '{token}' in line: {line}")

    def test_no_duplicate_drone_per_line(self, map_path: str) -> None:
        """Each drone appears at most once per output line."""
        _, sim = self._run(map_path)
        for line in sim.output_lines:
            tokens = line.split()
            drone_ids = [t.split("-")[0] for t in tokens]
            assert len(drone_ids) == len(set(drone_ids)), (
                f"Duplicate drone in line: {line}")

    def test_drone_ids_in_valid_range(self, map_path: str) -> None:
        """All drone IDs mentioned in output are 1..nb_drones."""
        graph, sim = self._run(map_path)
        valid_ids = set(range(1, graph.nb_drones + 1))
        for line in sim.output_lines:
            for token in line.split():
                match = MOVE_TOKEN_RE.match(token)
                assert match is not None
                drone_id = int(match.group(1))
                assert drone_id in valid_ids, (
                    f"Drone ID {drone_id} not in 1..{graph.nb_drones}")

    def test_every_drone_appears_in_output(self, map_path: str) -> None:
        """Every drone must have at least one move in the output."""
        graph, sim = self._run(map_path)
        seen_ids: set[AnyStr_co] = set()
        for line in sim.output_lines:
            for token in line.split():
                match = MOVE_TOKEN_RE.match(token)
                if match:
                    seen_ids.add(int(match.group(1)))
        expected = set(range(1, graph.nb_drones + 1))
        assert seen_ids == expected, (
            f"Missing drones in output: {expected - seen_ids}")

    def test_no_empty_output_lines(self, map_path: str) -> None:
        _, sim = self._run(map_path)
        for i, line in enumerate(sim.output_lines):
            assert line.strip(), f"Output line {i} is empty"
