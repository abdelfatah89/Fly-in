*This project has been created as part of the 42 curriculum by alaktaou.*

# Fly-in -- Drone Delivery Simulator

## Description

Fly-in is a turn-based drone delivery simulator. Given a map that describes zones, connections, and constraints, the program computes the optimal sequence of moves to deliver all drones from a **start hub** to an **end hub** in the fewest turns possible.

The map defines a graph of zones connected by links. Each zone has a type (normal, restricted, priority, or blocked) and a maximum drone capacity. Each connection has a maximum link capacity per turn. Drones must navigate this graph while respecting all capacity constraints, rerouting when blocked, and minimizing total delivery time.

The project includes:

- A **parser** that reads and validates map files.
- A **graph** data structure with adjacency lists and cost computation.
- A **pathfinding** engine based on Dijkstra's algorithm with support for forbidden zones and connections.
- A **simulator** that orchestrates turn-based movement, capacity enforcement, and dynamic rerouting.
- A **renderer** built with Tkinter for real-time visual playback of the simulation.
- A comprehensive **test suite** with 437 tests covering unit, integration, and performance benchmarks.

### Performance Results

| Difficulty | Map | Drones | Target | Status |
|---|---|---|---|---|
| Easy | Linear path | 2 | <= 6 | Pass |
| Easy | Simple fork | 3 | <= 6 | Pass |
| Easy | Basic capacity | 4 | <= 8 | Pass |
| Medium | Dead end trap | 5 | <= 15 | Pass |
| Medium | Circular loop | 6 | <= 20 | Pass |
| Medium | Priority puzzle | 4 | <= 12 | Pass |
| Hard | Maze nightmare | 8 | <= 45 | Pass |
| Hard | Capacity hell | 12 | <= 60 | Pass |
| Hard | Ultimate challenge | 15 | <= 35 | Pass |
| Challenger | The Impossible Dream | 25 | <= 45 | Pass |

---

## Instructions

### Prerequisites

- Python 3.10 or higher
- Tkinter (included with most Python distributions; required only for the renderer)

### Installation

```bash
# Clone the repository
git clone <repository-url>
cd Fly-in

# Create and activate a virtual environment
python3 -m venv .fly_env
source .fly_env/bin/activate

# Install dependencies
make install
```

### Running the Simulator

```bash
# Run with the default challenge map
make run

# Run with a specific map file
python3 fly-in.py maps/easy/01_linear_path.txt

# Run with the visual renderer (requires Tkinter)
# Edit fly-in.py to enable renderer mode, or use:
python3 fly-in.py map.txt
```

### Running Tests

```bash
# Run the full test suite (437 tests)
python3 -m pytest tests/ -v

# Run only performance benchmarks
python3 -m pytest tests/test_performance.py -v

# Run only map integration tests
python3 -m pytest tests/test_maps.py -v
```

### Linting

```bash
# Standard lint (flake8 + mypy)
make lint

# Strict mode
make lint-strict
```

### Other Commands

```bash
make debug    # Run with Python debugger (pdb)
make clean    # Remove all cache files
```

---

## Project Structure

```
Fly-in/
├── fly-in.py              # Entry point
├── Makefile               # Build and run commands
├── requirements.txt       # Python dependencies
├── map.txt                # Default challenge map
├── src/
│   ├── parser.py          # Map file parser and validator
│   ├── zone.py            # Zone model (types, capacity, occupancy)
│   ├── connection.py       # Connection model (links between zones)
│   ├── drone.py           # Drone model (status, movement, transit)
│   ├── graph.py           # Graph structure (adjacency, costs)
│   ├── path_finding.py    # Dijkstra's algorithm
│   ├── simulator.py       # Turn-based simulation engine
│   └── renderer.py        # Tkinter visual renderer
├── maps/
│   ├── easy/              # 3 easy maps (2-4 drones)
│   ├── medium/            # 3 medium maps (4-6 drones)
│   ├── hard/              # 3 hard maps (8-15 drones)
│   └── challenger/        # 2 challenger maps (25 drones)
└── tests/
    ├── test_fly_in.py     # Unit tests for all components
    ├── test_maps.py       # Integration tests for all map files
    └── test_performance.py # Performance benchmark tests
```

---

## Algorithm Choices and Implementation Strategy

### 1. Pathfinding: Dijkstra's Algorithm

The core pathfinding uses **Dijkstra's shortest-path algorithm** with a priority queue (`heapq`). Zone types influence path cost:

- **Normal** zones cost 1.0
- **Priority** zones cost 0.5 (preferred by the algorithm)
- **Restricted** zones cost 2.0 (avoided when alternatives exist; take 2 turns to traverse)
- **Blocked** zones cost infinity (never entered)

The algorithm supports **forbidden zones** and **forbidden connections** as parameters, which enables dynamic rerouting without modifying the graph.

### 2. Diverse Path Pre-computation

Before the simulation starts, the simulator computes up to **k diverse paths** from start to goal. It iteratively runs Dijkstra, each time forbidding restricted capacity-1 zones found in previous paths. This forces the algorithm to discover alternative routes.

Paths with transit times more than 1 turn longer than the fastest path are discarded. This ensures drones arrive at bottlenecks at a steady, even rate rather than in bursts.

### 3. Round-Robin Path Assignment

Drones are assigned to the diverse paths in a **round-robin** fashion (`drone[i] = paths[i % len(paths)]`). This distributes traffic across all viable routes from the very first turn, preventing initial congestion on the shortest path.

### 4. Priority-Based Move Ordering

Each turn, move plans are **sorted by remaining hops to the goal** (ascending). Drones closest to finishing get priority. Drones still at the start hub are given lowest priority (`float('inf')`) to preserve the round-robin interleaving among them.

This prevents pipeline stalls where a drone near the goal is blocked by one that just started.

### 5. Surgical Rerouting

When a drone is blocked (cannot move for 1+ turns), it reroutes. The rerouting is **surgical**: only the immediate blocking zone is forbidden, not all zones used in the current turn. This prevents drones from being forced into extremely long detour paths.

The rerouting also forbids the specific blocked connection, pushing the drone to discover a genuinely different route.

### 6. Capacity Management

The simulator tracks **zone occupancy** and **connection usage** per turn. Before applying any move, it computes expected occupancy including drones already in transit. Start and end hubs are exempt from capacity limits, allowing all drones to queue and arrive freely.

---

## Visual Representation

The project includes a **Tkinter-based renderer** that provides real-time animated visualization of the simulation.

### Features

- **Zone rendering**: Each zone is drawn as a colored circle at its grid position. Colors come from the map file (e.g., green for start, red for bottlenecks, gold for priority zones). The special "rainbow" end hub is drawn with concentric colored rings.
- **Zone labels**: Each zone displays its name above the circle and an occupancy indicator (`[current/max]`) below it. Start and end hubs show drone counts relative to the total fleet.
- **Connection rendering**: All connections are drawn as black lines between zone centers.
- **Drone rendering**: Drones are drawn as small orange circles with dark red outlines. When multiple drones share a zone, they are arranged in a **circular fan-out** pattern around the zone center so they don't overlap.
- **In-transit drone display**: Drones traversing restricted zones (2-turn transit) are rendered at their **target zone**, not their departure zone, so they remain visible during transit.
- **Animated playback**: The simulation advances one turn per frame (configurable delay), with the canvas fully redrawn each frame. The terminal prints each turn's move output in real time.
- **Completion reporting**: When all drones are delivered, the renderer prints the total turn count to the terminal.

### How the Renderer Enhances Understanding

The renderer transforms the abstract simulation output (text lines like `D1-wp1 D2-wp2`) into an intuitive visual flow. You can immediately see:

- Where **bottlenecks** form (zones with high occupancy ratios)
- How **diverse paths** distribute traffic across the graph
- When and where **rerouting** kicks in (a drone suddenly changes direction)
- The **pipeline effect** as drones flow through the gauntlet in sequence
- How **restricted zones** slow transit (drones visibly pause for 2 turns)

---

## Resources

### Documentation and References

- [Dijkstra's Algorithm - Wikipedia](https://en.wikipedia.org/wiki/Dijkstra%27s_algorithm)
- [Python heapq module](https://docs.python.org/3/library/heapq.html) -- priority queue used in pathfinding
- [Tkinter documentation](https://docs.python.org/3/library/tkinter.html) -- GUI framework for the renderer
- [pytest documentation](https://docs.pytest.org/en/stable/) -- testing framework
- [mypy documentation](https://mypy.readthedocs.io/en/stable/) -- static type checking
- [Graph Theory - Introduction](https://en.wikipedia.org/wiki/Graph_theory) -- foundational concepts for zone/connection modeling

### Use of AI

During this project, I used AI assistants (ChatGPT and Claude/Cursor) as technical assistants and reviewers, not as code generators to blindly copy from.

#### ChatGPT was used to:

1. **Understand and clarify requirements**
   - Help interpret the subject specifications (movement rules, restricted zones, capacity constraints, etc.)
   - Break down complex requirements into smaller, implementable steps

2. **Design architecture**
   - Discuss and refine the structure of the project (Graph, Drone, Simulator, Renderer)
   - Decide where specific logic should live (e.g., pathfinding vs simulation vs rendering)
   - Improve separation of concerns between components

3. **Debugging and code review**
   - Identify logical bugs in my implementation (e.g., pathfinding issues, occupancy handling, rerouting mistakes)
   - Suggest corrections and improvements
   - Review updated versions of my code and point out remaining issues

4. **Algorithm guidance**
   - Explain concepts such as Dijkstra, priority-based scheduling, and rerouting strategies
   - Help design improvements like rerouting only when necessary, avoiding congestion, and distributing drones across multiple paths

5. **Renderer design**
   - Guide the design of the Tkinter renderer
   - Explain layout concepts like grid spacing and padding
   - Suggest clean rendering architecture and visualization strategies (zones, drones, connections, animations)

6. **Incremental development approach**
   - Help structure the project step-by-step: parser, graph, pathfinding, simulator, optimization, renderer
   - Ensure each stage was working correctly before moving forward

#### Claude (via Cursor IDE) was used to:

1. **Bug diagnosis and fixing**
   - Identified a critical pathfinding bug where `current_zone_name == goal` compared a string to a Zone object, causing Dijkstra to never find the goal
   - Diagnosed renderer issues (e.g., `mainloop()` called before `animate()`, double `generate_output_line` calls, string-vs-object comparisons)

2. **Performance optimization**
   - Analyzed the challenge map structure to identify bottlenecks (gate pipeline, convergence points, gauntlet)
   - Implemented and iterated on three optimization strategies: diverse path pre-computation, priority-based move ordering, and surgical rerouting
   - Reduced the challenger map from 62 turns to 43 turns through targeted algorithmic improvements

3. **Test suite creation**
   - Created comprehensive pytest suites: unit tests for all components (Zone, Connection, Drone, Graph, Dijkstra, Parser, Simulator), integration tests for all map files, and performance benchmark tests
   - 437 tests total covering parsing, graph construction, pathfinding, simulation correctness, output format validation, and performance targets

4. **Code quality and type safety**
   - Fixed all flake8 and mypy errors across the codebase
   - Added complete type annotations to all functions and methods
   - Cleaned up imports, removed duplicates, and ensured strict mypy compliance

5. **Renderer analysis and improvements**
   - Analyzed the renderer architecture and identified coordinate scaling, drone overlap, in-transit visibility, and hub occupancy display issues
   - Provided detailed implementation guidance for each fix
