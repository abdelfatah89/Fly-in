import os
from src.parser import Parser
from src.graph import Graph
from src.simulator import Simulator

MAPS_DIR = os.path.join(os.path.dirname(__file__), "..", "maps")


def _run_map(rel_path: str) -> int:
    """Parse, build, simulate, return number of output lines."""
    full = os.path.join(MAPS_DIR, rel_path)
    data = Parser().parse(full)
    assert data is not None, f"Failed to parse {rel_path}"
    graph = Graph(**data)
    sim = Simulator(graph)
    sim.run(max_turns=200)
    assert sim.all_delivered(), (
        f"{rel_path}: not all drones delivered "
        f"after {sim.current_turn} turns")
    return len(sim.output_lines)


# ═══════════════════════════════════════════════════════
#  Easy Maps — must be solved in < 10 turns
# ═══════════════════════════════════════════════════════


class TestEasyPerformance:

    def test_linear_path_under_10(self) -> None:
        turns = _run_map("easy/01_linear_path.txt")
        assert turns < 10, f"< 10 turns, got {turns}"

    def test_linear_path_target(self) -> None:
        turns = _run_map("easy/01_linear_path.txt")
        assert turns <= 6, f"Target <= 6, got {turns}"

    def test_simple_fork_under_10(self) -> None:
        turns = _run_map("easy/02_simple_fork.txt")
        assert turns < 10, f"< 10 turns, got {turns}"

    def test_simple_fork_target(self) -> None:
        turns = _run_map("easy/02_simple_fork.txt")
        assert turns <= 6, f"Target <= 6, got {turns}"

    def test_basic_capacity_under_10(self) -> None:
        turns = _run_map("easy/03_basic_capacity.txt")
        assert turns < 10, f"< 10 turns, got {turns}"

    def test_basic_capacity_target(self) -> None:
        turns = _run_map("easy/03_basic_capacity.txt")
        assert turns <= 8, f"Target <= 8, got {turns}"


# ═══════════════════════════════════════════════════════
#  Medium Maps — must be solved in 10–30 turns
# ═══════════════════════════════════════════════════════


class TestMediumPerformance:

    def test_dead_end_trap_under_30(self) -> None:
        turns = _run_map("medium/01_dead_end_trap.txt")
        assert turns <= 30, f"<= 30 turns, got {turns}"

    def test_dead_end_trap_target(self) -> None:
        turns = _run_map("medium/01_dead_end_trap.txt")
        assert turns <= 15, f"Target <= 15, got {turns}"

    def test_circular_loop_under_30(self) -> None:
        turns = _run_map("medium/02_circular_loop.txt")
        assert turns <= 30, f"<= 30 turns, got {turns}"

    def test_circular_loop_target(self) -> None:
        turns = _run_map("medium/02_circular_loop.txt")
        assert turns <= 20, f"Target <= 20, got {turns}"

    def test_priority_puzzle_under_30(self) -> None:
        turns = _run_map("medium/03_priority_puzzle.txt")
        assert turns <= 30, f"<= 30 turns, got {turns}"

    def test_priority_puzzle_target(self) -> None:
        turns = _run_map("medium/03_priority_puzzle.txt")
        assert turns <= 12, f"Target <= 12, got {turns}"


# ═══════════════════════════════════════════════════════
#  Hard Maps — must be solved in < 60 turns
# ═══════════════════════════════════════════════════════


class TestHardPerformance:

    def test_maze_nightmare_under_60(self) -> None:
        turns = _run_map("hard/01_maze_nightmare.txt")
        assert turns < 60, f"< 60 turns, got {turns}"

    def test_maze_nightmare_target(self) -> None:
        turns = _run_map("hard/01_maze_nightmare.txt")
        assert turns <= 45, f"Target <= 45, got {turns}"

    def test_capacity_hell_under_60(self) -> None:
        turns = _run_map("hard/02_capacity_hell.txt")
        assert turns < 60, f"< 60 turns, got {turns}"

    def test_capacity_hell_target(self) -> None:
        turns = _run_map("hard/02_capacity_hell.txt")
        assert turns <= 60, f"Target <= 60, got {turns}"

    def test_ultimate_challenge_under_60(self) -> None:
        turns = _run_map("hard/03_ultimate_challenge.txt")
        assert turns < 60, f"< 60 turns, got {turns}"

    def test_ultimate_challenge_target(self) -> None:
        turns = _run_map("hard/03_ultimate_challenge.txt")
        assert turns <= 35, f"Target <= 35, got {turns}"


# ═══════════════════════════════════════════════════════
#  Challenger Map (optional) — reference record: 45
# ═══════════════════════════════════════════════════════


class TestChallengerPerformance:

    def test_impossible_dream_record(self) -> None:
        turns = _run_map(
            "challenger/01_the_impossible_dream.txt")
        assert turns <= 45, f"Record <= 45, got {turns}"
