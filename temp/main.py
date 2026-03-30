"""Main entry point for Fly-in drone simulation.

Usage:
    python main.py <map_file> [--gui] [--output <file>]

Examples:
    python main.py maps/easy/01_linear_path.txt
    python main.py maps/medium/03_priority_puzzle.txt --gui
    python main.py maps/hard/01_maze_nightmare.txt --output solution.txt
"""

import sys
import argparse
from pathlib import Path

# Add src to path
sys.path.insert(0, str(Path(__file__).parent))

from src.parser import MapParser, MapBuilder  # noqa: E402
from src.simulation import Simulator  # noqa: E402
from src.gui import SimulationRenderer  # noqa: E402


def run_simulation(
    map_file: str,
    use_gui: bool = False,
    output_file: str = None
) -> None:
    """Run the drone simulation.

    Args:
        map_file: Path to the map file
        use_gui: Whether to show GUI visualization
        output_file: Optional file to write output to
    """
    try:
        # Parse and build map
        print(f"Loading map: {map_file}")
        parser = MapParser(map_file)
        parser.parse()
        parsed_data = parser.get_parsed_data()
        map_data = MapBuilder.build_from_parsed_data(parsed_data)

        print(f"Map loaded: {len(map_data.zones)} zones, {len(map_data.drones)} drones")
        print(f"Path: {map_data.start_zone_name} -> {map_data.end_zone_name}")

        # Create simulator
        simulator = Simulator(map_data)

        if use_gui:
            # Run with GUI
            print("Starting GUI visualization...")
            renderer = SimulationRenderer(map_data, simulator)
            renderer.run()
        else:
            # Run without GUI
            print("Running simulation...")
            output_lines = simulator.run(max_turns=1000)

            print(f"\n{'='*60}")
            print(f"Simulation completed in {simulator.get_turn_count()} turns")
            print(f"{'='*60}\n")

            # Print output
            for line in output_lines:
                print(line)

            # Save to file if requested
            if output_file:
                with open(output_file, 'w') as f:
                    f.write('\n'.join(output_lines) + '\n')
                print(f"\nOutput saved to: {output_file}")

            # Print summary
            simulator.print_summary()

    except FileNotFoundError:
        print(f"Error: Map file not found: {map_file}")
        sys.exit(1)
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


def main() -> None:
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description="Fly-in Drone Routing Simulation",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python main.py maps/easy/01_linear_path.txt
  python main.py maps/medium/03_priority_puzzle.txt --gui
  python main.py maps/hard/01_maze_nightmare.txt --output solution.txt
        """
    )

    parser.add_argument(
        'map_file',
        help='Path to the map file'
    )

    parser.add_argument(
        '--gui',
        action='store_true',
        help='Show GUI visualization'
    )

    parser.add_argument(
        '--output', '-o',
        help='Save output to file'
    )

    args = parser.parse_args()

    run_simulation(
        map_file=args.map_file,
        use_gui=args.gui,
        output_file=args.output
    )


if __name__ == "__main__":
    main()
