from pathlib import Path
from typing import Dict, List

import pytest
from src.parser import Parser
from src.zone import Zone, ZoneType
from src.connection import Connection
from src.drone import Drone, DroneStatus
from src.graph import Graph
from src.path_finding import Dijkstra
from src.simulator import Simulator


# ─── Helpers ─────────────────────────────────────────────


def write_map(tmp_path: Path, content: str) -> str:
    f = tmp_path / "map.txt"
    f.write_text(content)
    return str(f)


def make_zone(name: str, x: int = 0, y: int = 0,
              zone_type: ZoneType = ZoneType.NORMAL,
              max_capacity: int = 1) -> Zone:
    return Zone(name=name, x=x, y=y, zone_type=zone_type,
                max_capacity=max_capacity)


def build_graph(
    zones_dict: Dict[str, Zone],
    connections_list: List[Connection],
    nb_drones: int,
    start_name: str = "start",
    end_name: str = "goal",
) -> Graph:
    return Graph(
        nb_drones=nb_drones,
        start_hub=zones_dict[start_name],
        end_hub=zones_dict[end_name],
        zones=zones_dict,
        connections=connections_list,
    )


def linear_graph(n_waypoints: int = 2, nb_drones: int = 1,
                 wp_capacity: int = 1,
                 link_capacity: int = 1) -> Graph:
    start = make_zone("start", x=0, y=0)
    goal = make_zone("goal", x=n_waypoints + 1, y=0)
    zones: Dict[str, Zone] = {"start": start, "goal": goal}
    conns: List[Connection] = []
    prev = "start"
    for i in range(1, n_waypoints + 1):
        wp = make_zone(f"wp{i}", x=i, y=0,
                       max_capacity=wp_capacity)
        zones[f"wp{i}"] = wp
        conns.append(Connection(prev, f"wp{i}", link_capacity))
        prev = f"wp{i}"
    conns.append(Connection(prev, "goal", link_capacity))
    return build_graph(zones, conns, nb_drones)


def diamond_graph(nb_drones: int = 2, cap_a: int = 1,
                  cap_b: int = 1) -> Graph:
    """start -> a -> goal
       start -> b -> goal"""
    start = make_zone("start")
    a = make_zone("a", x=1, y=0, max_capacity=cap_a)
    b = make_zone("b", x=1, y=1, max_capacity=cap_b)
    goal = make_zone("goal", x=2, y=0)
    zones: Dict[str, Zone] = {
        "start": start, "a": a, "b": b, "goal": goal,
    }
    conns = [
        Connection("start", "a", 1),
        Connection("start", "b", 1),
        Connection("a", "goal", 1),
        Connection("b", "goal", 1),
    ]
    return build_graph(zones, conns, nb_drones)


# ═════════════════════════════════════════════════════════
#  Zone
# ═════════════════════════════════════════════════════════


class TestZone:
    def test_movement_cost_normal(self) -> None:
        z = make_zone("a", zone_type=ZoneType.NORMAL)
        assert z.movement_cost == 1

    def test_movement_cost_restricted(self) -> None:
        z = make_zone("a", zone_type=ZoneType.RESTRICTED)
        assert z.movement_cost == 2

    def test_movement_cost_priority(self) -> None:
        z = make_zone("a", zone_type=ZoneType.PRIORITY)
        assert z.movement_cost == 1

    def test_is_blocked_true(self) -> None:
        z = make_zone("a", zone_type=ZoneType.BLOCKED)
        assert z.is_blocked is True

    def test_is_blocked_false(self) -> None:
        z = make_zone("a", zone_type=ZoneType.NORMAL)
        assert z.is_blocked is False

    def test_has_space_initially(self) -> None:
        z = make_zone("a", max_capacity=1)
        assert z.has_space is True

    def test_has_space_when_full(self) -> None:
        z = make_zone("a", max_capacity=1)
        z.add_drone()
        assert z.has_space is False

    def test_add_drone_success(self) -> None:
        z = make_zone("a", max_capacity=2)
        assert z.add_drone() is True
        assert z.current_occupancy == 1
        assert z.add_drone() is True
        assert z.current_occupancy == 2

    def test_add_drone_rejected_when_full(self) -> None:
        z = make_zone("a", max_capacity=1)
        z.add_drone()
        assert z.add_drone() is False
        assert z.current_occupancy == 1

    def test_remove_drone_success(self) -> None:
        z = make_zone("a")
        z.add_drone()
        assert z.remove_drone() is True
        assert z.current_occupancy == 0

    def test_remove_drone_from_empty(self) -> None:
        z = make_zone("a")
        assert z.remove_drone() is False
        assert z.current_occupancy == 0

    def test_reset_occupancy(self) -> None:
        z = make_zone("a", max_capacity=5)
        z.add_drone()
        z.add_drone()
        z.add_drone()
        z.reset_occupancy()
        assert z.current_occupancy == 0


# ═════════════════════════════════════════════════════════
#  Connection
# ═════════════════════════════════════════════════════════


class TestConnection:
    def test_name_format(self) -> None:
        c = Connection("X", "Y", 1)
        assert c.name == "X-Y"

    def test_is_connected_true(self) -> None:
        c = Connection("A", "B", 1)
        assert c.is_connected(make_zone("A")) is True
        assert c.is_connected(make_zone("B")) is True

    def test_is_connected_false(self) -> None:
        c = Connection("A", "B", 1)
        assert c.is_connected(make_zone("C")) is False

    def test_get_other_zone(self) -> None:
        c = Connection("A", "B", 1)
        assert c.get_other_zone(make_zone("A")) == "B"
        assert c.get_other_zone(make_zone("B")) == "A"

    def test_get_other_zone_unrelated(self) -> None:
        c = Connection("A", "B", 1)
        assert c.get_other_zone(make_zone("C")) is None

    def test_zones_property(self) -> None:
        c = Connection("X", "Y", 1)
        assert c.zones == ("X", "Y")

    def test_has_capacity(self) -> None:
        c = Connection("A", "B", 2)
        assert c.has_capacity is True
        c.use_capacity()
        assert c.has_capacity is True
        c.use_capacity()
        assert c.has_capacity is False

    def test_use_capacity_returns_false_when_full(self) -> None:
        c = Connection("A", "B", 1)
        assert c.use_capacity() is True
        assert c.use_capacity() is False

    def test_reset_usage(self) -> None:
        c = Connection("A", "B", 1)
        c.use_capacity()
        c.reset_usage()
        assert c.current_usage == 0
        assert c.has_capacity is True

    def test_key_is_sorted(self) -> None:
        c1 = Connection("B", "A", 1)
        c2 = Connection("A", "B", 1)
        assert c1.key == ("A", "B")
        assert c2.key == ("A", "B")
        assert c1.key == c2.key


# ═════════════════════════════════════════════════════════
#  Drone
# ═════════════════════════════════════════════════════════


class TestDrone:
    def test_initial_state(self) -> None:
        z = make_zone("home")
        d = Drone(1, z)
        assert d.drone_id == 1
        assert d.current_zone is z
        assert d.drone_status == DroneStatus.WAITING
        assert d.can_move is True
        assert d.is_reached is False
        assert d.path == []
        assert d.path_index == 0
        assert d.wait_turns == 0

    def test_complete_move(self) -> None:
        z1, z2 = make_zone("A"), make_zone("B")
        d = Drone(1, z1)
        d.complete_move(z2)
        assert d.current_zone is z2
        assert d.drone_status == DroneStatus.WAITING
        assert d.target_zone is None

    def test_start_restricted_transit(self) -> None:
        z1 = make_zone("A")
        z2 = make_zone("B", zone_type=ZoneType.RESTRICTED)
        d = Drone(1, z1)
        d.start_restricted_transit(z2)
        assert d.drone_status == DroneStatus.IN_TRANSIT
        assert d.target_zone is z2
        assert d.turns_remaining == 2
        assert d.can_move is False

    def test_progress_transit_decrements(self) -> None:
        z1, z2 = make_zone("A"), make_zone("B")
        d = Drone(1, z1)
        d.start_restricted_transit(z2)
        d.progress_transit()
        assert d.turns_remaining == 1
        assert d.drone_status == DroneStatus.IN_TRANSIT

    def test_progress_transit_completes(self) -> None:
        z1, z2 = make_zone("A"), make_zone("B")
        d = Drone(1, z1)
        d.start_restricted_transit(z2)
        d.progress_transit()
        d.progress_transit()
        assert d.turns_remaining == 0
        assert d.drone_status == DroneStatus.WAITING
        assert d.current_zone is z2

    def test_mark_reached(self) -> None:
        d = Drone(1, make_zone("A"))
        d.mark_reached()
        assert d.is_reached is True
        assert d.drone_status == DroneStatus.REACHED
        assert d.target_zone is None


# ═════════════════════════════════════════════════════════
#  Graph
# ═════════════════════════════════════════════════════════


class TestGraph:
    def test_drones_created_at_start_hub(self) -> None:
        g = linear_graph(n_waypoints=1, nb_drones=3)
        assert len(g.drones) == 3
        for d in g.drones:
            assert d.current_zone is g.start_hub

    def test_adjacency_is_bidirectional(self) -> None:
        g = linear_graph(n_waypoints=1, nb_drones=1)
        start_nbrs = [z.name for z, _ in g.adjacency["start"]]
        wp1_nbrs = [z.name for z, _ in g.adjacency["wp1"]]
        assert "wp1" in start_nbrs
        assert "start" in wp1_nbrs

    def test_get_neighbors(self) -> None:
        g = linear_graph(n_waypoints=1, nb_drones=1)
        neighbors = g.get_neighbors(g.zones["start"])
        assert len(neighbors) == 1
        assert neighbors[0][0].name == "wp1"

    def test_get_neighbors_unknown_zone(self) -> None:
        g = linear_graph(n_waypoints=1, nb_drones=1)
        fake = make_zone("unknown")
        assert g.get_neighbors(fake) == []

    def test_get_connection_exists(self) -> None:
        g = linear_graph(n_waypoints=1, nb_drones=1)
        conn = g.get_connection(
            g.zones["start"], g.zones["wp1"])
        assert conn is not None
        assert "start" in conn.zones and "wp1" in conn.zones

    def test_get_connection_none(self) -> None:
        g = linear_graph(n_waypoints=2, nb_drones=1)
        assert g.get_connection(
            g.zones["start"], g.zones["goal"]) is None

    def test_zone_cost_normal(self) -> None:
        g = linear_graph()
        assert g.get_zone_cost(make_zone("n")) == 1.0

    def test_zone_cost_restricted(self) -> None:
        g = linear_graph()
        z = make_zone("r", zone_type=ZoneType.RESTRICTED)
        assert g.get_zone_cost(z) == 2.0

    def test_zone_cost_priority(self) -> None:
        g = linear_graph()
        z = make_zone("p", zone_type=ZoneType.PRIORITY)
        assert g.get_zone_cost(z) == 0.5

    def test_zone_cost_blocked(self) -> None:
        g = linear_graph()
        z = make_zone("b", zone_type=ZoneType.BLOCKED)
        assert g.get_zone_cost(z) == float("inf")


# ═════════════════════════════════════════════════════════
#  PathFinding (Dijkstra)
# ═════════════════════════════════════════════════════════


class TestDijkstra:
    def test_single_drone_direct_path(self) -> None:
        g = linear_graph(n_waypoints=2, nb_drones=1)
        pf = Dijkstra(g)
        path = pf.find_path(g.start_hub, g.end_hub)
        names = [z.name for z in path]
        assert names == ["start", "wp1", "wp2", "goal"]

    def test_same_start_and_goal(self) -> None:
        g = linear_graph(n_waypoints=1, nb_drones=1)
        pf = Dijkstra(g)
        path = pf.find_path(g.start_hub, g.start_hub)
        assert len(path) == 1
        assert path[0].name == "start"

    def test_no_path_returns_empty(self) -> None:
        start = make_zone("start")
        goal = make_zone("goal", x=1)
        island = make_zone("island", x=2)
        zones: Dict[str, Zone] = {
            "start": start, "goal": goal, "island": island,
        }
        conns = [Connection("start", "island", 1)]
        g = build_graph(zones, conns, 1)
        pf = Dijkstra(g)
        assert pf.find_path(start, goal) == []

    def test_blocked_zone_is_avoided(self) -> None:
        start = make_zone("start")
        blocked = make_zone(
            "blk", x=1, zone_type=ZoneType.BLOCKED)
        alt = make_zone("alt", x=1, y=1)
        goal = make_zone("goal", x=2)
        zones: Dict[str, Zone] = {
            "start": start, "blk": blocked,
            "alt": alt, "goal": goal,
        }
        conns = [
            Connection("start", "blk", 1),
            Connection("start", "alt", 1),
            Connection("blk", "goal", 1),
            Connection("alt", "goal", 1),
        ]
        g = build_graph(zones, conns, 1)
        pf = Dijkstra(g)
        names = [z.name for z in pf.find_path(start, goal)]
        assert "blk" not in names
        assert names == ["start", "alt", "goal"]

    def test_forbidden_zones_excluded(self) -> None:
        start = make_zone("start")
        a = make_zone("a", x=1)
        b = make_zone("b", x=1, y=1)
        goal = make_zone("goal", x=2)
        zones: Dict[str, Zone] = {
            "start": start, "a": a, "b": b, "goal": goal,
        }
        conns = [
            Connection("start", "a", 1),
            Connection("start", "b", 1),
            Connection("a", "goal", 1),
            Connection("b", "goal", 1),
        ]
        g = build_graph(zones, conns, 1)
        pf = Dijkstra(g)
        path = pf.find_path(
            start, goal, forbidden_zones={"a"})
        names = [z.name for z in path]
        assert "a" not in names
        assert names == ["start", "b", "goal"]

    def test_forbidden_connections_excluded(self) -> None:
        start = make_zone("start")
        a = make_zone("a", x=1)
        b = make_zone("b", x=1, y=1)
        goal = make_zone("goal", x=2)
        conn_sa = Connection("start", "a", 1)
        conn_sb = Connection("start", "b", 1)
        conn_ag = Connection("a", "goal", 1)
        conn_bg = Connection("b", "goal", 1)
        zones: Dict[str, Zone] = {
            "start": start, "a": a, "b": b, "goal": goal,
        }
        conns = [conn_sa, conn_sb, conn_ag, conn_bg]
        g = build_graph(zones, conns, 1)
        pf = Dijkstra(g)
        path = pf.find_path(
            start, goal,
            forbidden_connections={conn_sa.key})
        names = [z.name for z in path]
        assert names == ["start", "b", "goal"]

    def test_prefers_priority_zone(self) -> None:
        start = make_zone("start")
        normal = make_zone("normal", x=1)
        prio = make_zone(
            "prio", x=1, y=1, zone_type=ZoneType.PRIORITY)
        goal = make_zone("goal", x=2)
        zones: Dict[str, Zone] = {
            "start": start, "normal": normal,
            "prio": prio, "goal": goal,
        }
        conns = [
            Connection("start", "normal", 1),
            Connection("start", "prio", 1),
            Connection("normal", "goal", 1),
            Connection("prio", "goal", 1),
        ]
        g = build_graph(zones, conns, 1)
        pf = Dijkstra(g)
        names = [z.name for z in pf.find_path(start, goal)]
        assert "prio" in names

    def test_avoids_restricted(self) -> None:
        start = make_zone("start")
        fast = make_zone("fast", x=1)
        slow = make_zone(
            "slow", x=1, y=1, zone_type=ZoneType.RESTRICTED)
        goal = make_zone("goal", x=2)
        zones: Dict[str, Zone] = {
            "start": start, "fast": fast,
            "slow": slow, "goal": goal,
        }
        conns = [
            Connection("start", "fast", 1),
            Connection("start", "slow", 1),
            Connection("fast", "goal", 1),
            Connection("slow", "goal", 1),
        ]
        g = build_graph(zones, conns, 1)
        pf = Dijkstra(g)
        names = [z.name for z in pf.find_path(start, goal)]
        assert "fast" in names
        assert "slow" not in names


# ═════════════════════════════════════════════════════════
#  Parser
# ═════════════════════════════════════════════════════════


VALID_MAP = (
    "nb_drones: 1\n"
    "start_hub: start 0 0\n"
    "hub: mid 1 0\n"
    "end_hub: goal 2 0\n"
    "connection: start-mid\n"
    "connection: mid-goal\n"
)


class TestParser:

    def test_valid_map_parses(self, tmp_path: Path) -> None:
        data = Parser().parse(write_map(tmp_path, VALID_MAP))
        assert data is not None
        assert data["nb_drones"] == 1
        assert data["start_hub"].name == "start"
        assert data["end_hub"].name == "goal"
        assert len(data["zones"]) == 3
        assert len(data["connections"]) == 2

    def test_comments_and_blanks_ignored(
        self, tmp_path: Path,
    ) -> None:
        content = (
            "# header\n\n"
            "nb_drones: 2\n"
            "# comment\n\n"
            "start_hub: s 0 0\n"
            "end_hub: g 1 0\n"
            "connection: s-g\n"
            "\n# trailing\n"
        )
        data = Parser().parse(write_map(tmp_path, content))
        assert data is not None
        assert data["nb_drones"] == 2

    def test_zone_with_all_metadata(
        self, tmp_path: Path,
    ) -> None:
        content = (
            "nb_drones: 1\n"
            "start_hub: s 0 0 [color=green]\n"
            "hub: m 1 0 "
            "[zone=restricted max_drones=3 color=blue]\n"
            "end_hub: g 2 0 [color=red]\n"
            "connection: s-m\n"
            "connection: m-g\n"
        )
        data = Parser().parse(write_map(tmp_path, content))
        assert data is not None
        assert data["zones"]["s"].color == "green"
        m = data["zones"]["m"]
        assert m.zone_type == ZoneType.RESTRICTED
        assert m.max_capacity == 3
        assert m.color == "blue"

    def test_connection_with_capacity(
        self, tmp_path: Path,
    ) -> None:
        content = (
            "nb_drones: 1\n"
            "start_hub: s 0 0\n"
            "end_hub: g 1 0\n"
            "connection: s-g [max_link_capacity=5]\n"
        )
        data = Parser().parse(write_map(tmp_path, content))
        assert data is not None
        assert data["connections"][0].max_link_capacity == 5

    def test_rejects_nb_drones_not_first(
        self, tmp_path: Path,
    ) -> None:
        content = (
            "start_hub: s 0 0\n"
            "nb_drones: 1\n"
            "end_hub: g 1 0\n"
            "connection: s-g\n"
        )
        result = Parser().parse(write_map(tmp_path, content))
        assert result is None

    def test_duplicate_nb_drones(
        self, tmp_path: Path,
    ) -> None:
        content = (
            "nb_drones: 1\n"
            "nb_drones: 2\n"
            "start_hub: s 0 0\n"
            "end_hub: g 1 0\n"
            "connection: s-g\n"
        )
        result = Parser().parse(write_map(tmp_path, content))
        assert result is None

    def test_nb_drones_zero(
        self, tmp_path: Path,
    ) -> None:
        content = (
            "nb_drones: 0\n"
            "start_hub: s 0 0\n"
            "end_hub: g 1 0\n"
            "connection: s-g\n"
        )
        result = Parser().parse(write_map(tmp_path, content))
        assert result is None

    def test_nb_drones_negative(
        self, tmp_path: Path,
    ) -> None:
        content = (
            "nb_drones: -3\n"
            "start_hub: s 0 0\n"
            "end_hub: g 1 0\n"
            "connection: s-g\n"
        )
        result = Parser().parse(write_map(tmp_path, content))
        assert result is None

    def test_nb_drones_non_numeric(
        self, tmp_path: Path,
    ) -> None:
        content = (
            "nb_drones: abc\n"
            "start_hub: s 0 0\n"
            "end_hub: g 1 0\n"
            "connection: s-g\n"
        )
        result = Parser().parse(write_map(tmp_path, content))
        assert result is None

    def test_missing_start_hub(
        self, tmp_path: Path,
    ) -> None:
        content = (
            "nb_drones: 1\n"
            "hub: m 1 0\n"
            "end_hub: g 2 0\n"
            "connection: m-g\n"
        )
        result = Parser().parse(write_map(tmp_path, content))
        assert result is None

    def test_missing_end_hub(
        self, tmp_path: Path,
    ) -> None:
        content = (
            "nb_drones: 1\n"
            "start_hub: s 0 0\n"
            "hub: m 1 0\n"
            "connection: s-m\n"
        )
        result = Parser().parse(write_map(tmp_path, content))
        assert result is None

    def test_multiple_start_hubs(
        self, tmp_path: Path,
    ) -> None:
        content = (
            "nb_drones: 1\n"
            "start_hub: s1 0 0\n"
            "start_hub: s2 1 0\n"
            "end_hub: g 2 0\n"
            "connection: s1-s2\n"
            "connection: s2-g\n"
        )
        result = Parser().parse(write_map(tmp_path, content))
        assert result is None

    def test_multiple_end_hubs(
        self, tmp_path: Path,
    ) -> None:
        content = (
            "nb_drones: 1\n"
            "start_hub: s 0 0\n"
            "end_hub: g1 1 0\n"
            "end_hub: g2 2 0\n"
            "connection: s-g1\n"
            "connection: g1-g2\n"
        )
        result = Parser().parse(write_map(tmp_path, content))
        assert result is None

    def test_blocked_start_hub(
        self, tmp_path: Path,
    ) -> None:
        content = (
            "nb_drones: 1\n"
            "start_hub: s 0 0 [zone=blocked]\n"
            "end_hub: g 1 0\n"
            "connection: s-g\n"
        )
        result = Parser().parse(write_map(tmp_path, content))
        assert result is None

    def test_blocked_end_hub(
        self, tmp_path: Path,
    ) -> None:
        content = (
            "nb_drones: 1\n"
            "start_hub: s 0 0\n"
            "end_hub: g 1 0 [zone=blocked]\n"
            "connection: s-g\n"
        )
        result = Parser().parse(write_map(tmp_path, content))
        assert result is None

    def test_duplicate_zone_name(
        self, tmp_path: Path,
    ) -> None:
        content = (
            "nb_drones: 1\n"
            "start_hub: s 0 0\n"
            "hub: s 1 0\n"
            "end_hub: g 2 0\n"
            "connection: s-g\n"
        )
        result = Parser().parse(write_map(tmp_path, content))
        assert result is None

    def test_zone_name_with_dash(
        self, tmp_path: Path,
    ) -> None:
        content = (
            "nb_drones: 1\n"
            "start_hub: start-zone 0 0\n"
            "end_hub: g 1 0\n"
            "connection: start-zone-g\n"
        )
        result = Parser().parse(write_map(tmp_path, content))
        assert result is None

    def test_zone_missing_coordinates(
        self, tmp_path: Path,
    ) -> None:
        content = (
            "nb_drones: 1\n"
            "start_hub: s 0\n"
            "end_hub: g 1 0\n"
            "connection: s-g\n"
        )
        result = Parser().parse(write_map(tmp_path, content))
        assert result is None

    def test_zone_non_integer_coordinates(
        self, tmp_path: Path,
    ) -> None:
        content = (
            "nb_drones: 1\n"
            "start_hub: s abc def\n"
            "end_hub: g 1 0\n"
            "connection: s-g\n"
        )
        result = Parser().parse(write_map(tmp_path, content))
        assert result is None

    def test_invalid_zone_type_metadata(
        self, tmp_path: Path,
    ) -> None:
        content = (
            "nb_drones: 1\n"
            "start_hub: s 0 0\n"
            "hub: m 1 0 [zone=teleport]\n"
            "end_hub: g 2 0\n"
            "connection: s-m\n"
            "connection: m-g\n"
        )
        result = Parser().parse(write_map(tmp_path, content))
        assert result is None

    def test_metadata_not_in_brackets(
        self, tmp_path: Path,
    ) -> None:
        content = (
            "nb_drones: 1\n"
            "start_hub: s 0 0 color=green\n"
            "end_hub: g 1 0\n"
            "connection: s-g\n"
        )
        result = Parser().parse(write_map(tmp_path, content))
        assert result is None

    def test_metadata_missing_equals(
        self, tmp_path: Path,
    ) -> None:
        content = (
            "nb_drones: 1\n"
            "start_hub: s 0 0 [color]\n"
            "end_hub: g 1 0\n"
            "connection: s-g\n"
        )
        result = Parser().parse(write_map(tmp_path, content))
        assert result is None

    def test_unknown_zone_metadata_key(
        self, tmp_path: Path,
    ) -> None:
        content = (
            "nb_drones: 1\n"
            "start_hub: s 0 0 [speed=fast]\n"
            "end_hub: g 1 0\n"
            "connection: s-g\n"
        )
        result = Parser().parse(write_map(tmp_path, content))
        assert result is None

    def test_connection_unknown_zone(
        self, tmp_path: Path,
    ) -> None:
        content = (
            "nb_drones: 1\n"
            "start_hub: s 0 0\n"
            "end_hub: g 1 0\n"
            "connection: s-nowhere\n"
        )
        result = Parser().parse(write_map(tmp_path, content))
        assert result is None

    def test_connection_same_zone(
        self, tmp_path: Path,
    ) -> None:
        content = (
            "nb_drones: 1\n"
            "start_hub: s 0 0\n"
            "end_hub: g 1 0\n"
            "connection: s-s\n"
        )
        result = Parser().parse(write_map(tmp_path, content))
        assert result is None

    def test_duplicate_connection(
        self, tmp_path: Path,
    ) -> None:
        content = (
            "nb_drones: 1\n"
            "start_hub: s 0 0\n"
            "end_hub: g 1 0\n"
            "connection: s-g\n"
            "connection: g-s\n"
        )
        result = Parser().parse(write_map(tmp_path, content))
        assert result is None

    def test_connection_missing_dash(
        self, tmp_path: Path,
    ) -> None:
        content = (
            "nb_drones: 1\n"
            "start_hub: s 0 0\n"
            "end_hub: g 1 0\n"
            "connection: sg\n"
        )
        result = Parser().parse(write_map(tmp_path, content))
        assert result is None

    def test_connection_unknown_metadata(
        self, tmp_path: Path,
    ) -> None:
        content = (
            "nb_drones: 1\n"
            "start_hub: s 0 0\n"
            "end_hub: g 1 0\n"
            "connection: s-g [speed=fast]\n"
        )
        result = Parser().parse(write_map(tmp_path, content))
        assert result is None

    def test_unknown_key(self, tmp_path: Path) -> None:
        content = (
            "nb_drones: 1\n"
            "start_hub: s 0 0\n"
            "end_hub: g 1 0\n"
            "foobar: something\n"
            "connection: s-g\n"
        )
        result = Parser().parse(write_map(tmp_path, content))
        assert result is None

    def test_file_not_found(self) -> None:
        assert Parser().parse("/nonexistent/file.txt") is None

    def test_empty_file(self, tmp_path: Path) -> None:
        result = Parser().parse(write_map(tmp_path, ""))
        assert result is None

    def test_only_comments(self, tmp_path: Path) -> None:
        content = "# just a comment\n# another comment\n"
        result = Parser().parse(write_map(tmp_path, content))
        assert result is None


# ═════════════════════════════════════════════════════════
#  Simulator
# ═════════════════════════════════════════════════════════


class TestSimulator:

    def test_single_drone_direct_connection(self) -> None:
        start = make_zone("start")
        goal = make_zone("goal", x=1)
        zones: Dict[str, Zone] = {
            "start": start, "goal": goal,
        }
        g = build_graph(
            zones, [Connection("start", "goal", 1)], 1)
        sim = Simulator(g)
        sim.run()
        assert sim.output_lines == ["D1-goal"]

    def test_single_drone_linear_path(self) -> None:
        g = linear_graph(n_waypoints=2, nb_drones=1)
        sim = Simulator(g)
        sim.run()
        lines = sim.output_lines
        assert len(lines) == 3
        assert "D1-wp1" in lines[0]
        assert "D1-wp2" in lines[1]
        assert "D1-goal" in lines[2]

    def test_two_drones_both_arrive(self) -> None:
        g = linear_graph(n_waypoints=2, nb_drones=2)
        sim = Simulator(g)
        sim.run()
        all_output = " ".join(sim.output_lines)
        assert "D1-goal" in all_output
        assert "D2-goal" in all_output

    def test_all_delivered_flag(self) -> None:
        g = linear_graph(n_waypoints=1, nb_drones=1)
        sim = Simulator(g)
        assert sim.all_delivered() is False
        sim.run()
        assert sim.all_delivered() is True

    def test_restricted_zone_output(self) -> None:
        start = make_zone("start")
        mid = make_zone(
            "mid", x=1, zone_type=ZoneType.RESTRICTED)
        goal = make_zone("goal", x=2)
        zones: Dict[str, Zone] = {
            "start": start, "mid": mid, "goal": goal,
        }
        conns = [
            Connection("start", "mid", 1),
            Connection("mid", "goal", 1),
        ]
        g = build_graph(zones, conns, 1)
        sim = Simulator(g)
        sim.run()
        lines = sim.output_lines
        has_conn_fmt = any(
            "D1-start-mid" in line for line in lines)
        assert has_conn_fmt, (
            f"Expected connection-name format, "
            f"got {lines}")
        assert lines[-1] == "D1-goal"

    def test_restricted_zone_extra_turn(self) -> None:
        start = make_zone("start")
        mid = make_zone(
            "mid", x=1, zone_type=ZoneType.RESTRICTED)
        goal = make_zone("goal", x=2)
        zones: Dict[str, Zone] = {
            "start": start, "mid": mid, "goal": goal,
        }
        conns = [
            Connection("start", "mid", 1),
            Connection("mid", "goal", 1),
        ]
        g = build_graph(zones, conns, 1)
        sim = Simulator(g)
        sim.run()
        assert len(sim.output_lines) == 2
        assert sim.current_turn == 3

    def test_capacity_one_blocks_second(self) -> None:
        g = linear_graph(
            n_waypoints=1, nb_drones=2, wp_capacity=1)
        sim = Simulator(g)
        sim.run()
        assert len(sim.output_lines) >= 3

    def test_link_capacity_limits(self) -> None:
        start = make_zone("start")
        goal = make_zone("goal", x=1)
        zones: Dict[str, Zone] = {
            "start": start, "goal": goal,
        }
        g = build_graph(
            zones, [Connection("start", "goal", 1)], 2)
        sim = Simulator(g)
        sim.run()
        for line in sim.output_lines:
            assert len(line.strip().split()) <= 1

    def test_high_link_capacity_parallel(self) -> None:
        start = make_zone("start")
        goal = make_zone("goal", x=1)
        zones: Dict[str, Zone] = {
            "start": start, "goal": goal,
        }
        g = build_graph(
            zones, [Connection("start", "goal", 2)], 2)
        sim = Simulator(g)
        sim.run()
        lines = sim.output_lines
        assert len(lines) == 1
        assert "D1-goal" in lines[0]
        assert "D2-goal" in lines[0]

    def test_hub_capacity_unlimited(self) -> None:
        start = make_zone("start")
        goal = make_zone("goal", x=1)
        zones: Dict[str, Zone] = {
            "start": start, "goal": goal,
        }
        g = build_graph(
            zones, [Connection("start", "goal", 5)], 5)
        sim = Simulator(g)
        sim.run()
        lines = sim.output_lines
        assert len(lines) == 1
        for i in range(1, 6):
            assert f"D{i}-goal" in lines[0]

    def test_reroute_finds_alternative(self) -> None:
        start = make_zone("start")
        a = make_zone("a", x=1, y=0, max_capacity=1)
        b = make_zone("b", x=1, y=1, max_capacity=1)
        goal = make_zone("goal", x=2, y=0)
        zones: Dict[str, Zone] = {
            "start": start, "a": a, "b": b, "goal": goal,
        }
        conns = [
            Connection("start", "a", 1),
            Connection("start", "b", 1),
            Connection("a", "goal", 1),
            Connection("b", "goal", 1),
        ]
        g = build_graph(zones, conns, 2)
        sim = Simulator(g)
        sim.run()
        all_out = " ".join(sim.output_lines)
        assert "D1-goal" in all_out
        assert "D2-goal" in all_out
        used_a = "D1-a" in all_out or "D2-a" in all_out
        used_b = "D1-b" in all_out or "D2-b" in all_out
        assert used_a and used_b, (
            f"Expected both paths, got: {sim.output_lines}")

    def test_reroute_three_drones_diamond(self) -> None:
        start = make_zone("start")
        a = make_zone("a", x=1, y=0, max_capacity=1)
        b = make_zone("b", x=1, y=1, max_capacity=1)
        goal = make_zone("goal", x=2, y=0)
        zones: Dict[str, Zone] = {
            "start": start, "a": a, "b": b, "goal": goal,
        }
        conns = [
            Connection("start", "a", 1),
            Connection("start", "b", 1),
            Connection("a", "goal", 1),
            Connection("b", "goal", 1),
        ]
        g = build_graph(zones, conns, 3)
        sim = Simulator(g)
        sim.run()
        all_out = " ".join(sim.output_lines)
        for i in range(1, 4):
            assert f"D{i}-goal" in all_out

    def test_no_path_returns_empty(self) -> None:
        start = make_zone("start")
        goal = make_zone("goal", x=1)
        island = make_zone("island", x=2)
        zones: Dict[str, Zone] = {
            "start": start, "goal": goal, "island": island,
        }
        g = build_graph(
            zones, [Connection("start", "island", 1)], 1)
        sim = Simulator(g)
        sim.run(max_turns=10)
        assert sim.output_lines == []

    def test_one_move_per_drone_per_line(self) -> None:
        g = linear_graph(n_waypoints=3, nb_drones=3)
        sim = Simulator(g)
        sim.run()
        for line in sim.output_lines:
            ids = [tok.split("-")[0] for tok in line.split()]
            assert len(ids) == len(set(ids)), (
                f"Duplicate drone in line: {line}")

    def test_turn_count_matches_output(self) -> None:
        g = linear_graph(n_waypoints=2, nb_drones=1)
        sim = Simulator(g)
        sim.run()
        assert len(sim.output_lines) <= sim.current_turn

    def test_easy_01_linear_path(self) -> None:
        import os
        path = os.path.join(
            os.path.dirname(__file__),
            "..", "maps", "easy", "01_linear_path.txt")
        if not os.path.exists(path):
            pytest.skip("easy/01_linear_path.txt not found")
        data = Parser().parse(path)
        assert data is not None
        g = Graph(**data)
        sim = Simulator(g)
        sim.run()
        assert len(sim.output_lines) > 0
        assert sim.all_delivered()
