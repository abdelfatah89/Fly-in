# Fly-in Drone Routing Simulation

A complete Python implementation of the 42 school "Fly-in" project - a drone routing simulation system that moves drones from a start zone to an end zone while respecting capacity constraints and movement costs.

## 🎯 Project Overview

This system simulates drone navigation through a graph of zones (nodes) connected by pathways (edges). Each zone has capacity limits, movement costs, and type-specific behaviors. The goal is to deliver all drones to the destination in minimum turns.

## 📐 Architecture

```
fly_in/
├── src/
│   ├── models/          # Core data structures
│   │   ├── zone.py      # Zone (node) with capacity and type
│   │   ├── connection.py # Connection (edge) with capacity
│   │   ├── drone.py     # Drone entity with state
│   │   └── map_data.py  # Complete map container
│   ├── parser/          # Map file parsing
│   │   ├── map_parser.py   # Parser for map format
│   │   └── map_builder.py  # Builds models from parsed data
│   ├── pathfinding/     # Path algorithms
│   │   └── dijkstra.py  # Simple Dijkstra implementation
│   ├── simulation/      # Core simulation logic
│   │   └── simulator.py # Turn-by-turn execution
│   └── gui/             # Visualization
│       └── renderer.py  # Tkinter GUI
└── main.py              # Entry point
```

## 🚀 Features

### Zone Types
- **Normal**: 1-turn movement cost
- **Restricted**: 2-turn movement cost (drone in transit)
- **Priority**: 1-turn cost but preferred in pathfinding
- **Blocked**: Cannot be entered
- **Start/End**: Unlimited capacity

### Capacity Management
- **Zone capacity**: Limited drones per zone
- **Connection capacity**: Limited simultaneous traversals
- **Two-phase update**: Collect intentions → Validate → Apply

### Pathfinding
- Simple Dijkstra algorithm
- Cost-based routing (prefers priority zones, avoids restricted)
- Handles blocked zones automatically

### Visualization
- Tkinter GUI with play/pause/step controls
- Color-coded zones by type
- Real-time drone position display
- Turn counter and statistics

## 💻 Usage

### Basic Simulation (No GUI)
```bash
python main.py maps/easy/01_linear_path.txt
```

### With GUI Visualization
```bash
python main.py maps/medium/03_priority_puzzle.txt --gui
```

### Save Output to File
```bash
python main.py maps/hard/01_maze_nightmare.txt --output solution.txt
```

## 📋 Map File Format

```
nb_drones: 5

start_hub: start 0 0 [color=green]
hub: waypoint1 1 0 [zone=normal color=blue]
hub: waypoint2 2 0 [zone=restricted color=red max_drones=2]
hub: waypoint3 3 0 [zone=priority color=green]
end_hub: goal 4 0 [color=yellow]

connection: start-waypoint1
connection: waypoint1-waypoint2 [max_link_capacity=2]
connection: waypoint2-waypoint3
connection: waypoint3-goal
```

### Attributes
- `zone=<type>`: Zone type (normal/restricted/priority/blocked)
- `max_drones=<n>`: Zone capacity (default: 1)
- `max_link_capacity=<n>`: Connection capacity (default: 1)
- `color=<color>`: Visualization color

## 📊 Output Format

Each turn produces a line showing drone movements:
```
D0-waypoint1
D0-waypoint2 D1-waypoint1
D0-goal D1-waypoint2
D1-goal
```

Format: `D<ID>-<zone>` where ID is drone number and zone is destination.

## 🧪 Testing

Run individual component tests:
```bash
python test_parser.py      # Test map parsing
python test_models.py      # Test data models
python test_pathfinding.py # Test Dijkstra
python test_simulation.py  # Test simulation
python test_gui.py         # Test GUI
```

## 🛠️ Technical Details

### Requirements
- Python 3.10+
- No external dependencies (pure stdlib)
- Tkinter for GUI (included with Python)

### Code Quality
- Type hints everywhere
- Docstrings for all classes and methods
- flake8 compatible
- mypy compatible
- Clean OOP design

### Simulation Algorithm

1. **Initialization**
   - Parse map file
   - Build graph structure
   - Compute paths using Dijkstra
   - Place all drones at start

2. **Turn Execution**
   - Progress restricted transits (drones in 2-turn movement)
   - Collect movement intentions from idle drones
   - Validate capacity constraints (zones + connections)
   - Apply valid movements
   - Generate output line

3. **Termination**
   - Simulation ends when all drones reach goal
   - Output shows turn-by-turn movements

## 📈 Performance

Expected turn counts:
- **Easy maps**: < 10 turns
- **Medium maps**: 10-30 turns
- **Hard maps**: < 60 turns

The simple greedy approach (each drone follows its pre-computed path) works well due to capacity-based queuing. More sophisticated coordination could reduce turn counts but adds complexity.

## 🔧 Design Decisions

### Why Simple Dijkstra?
- Easy to understand and explain
- Sufficient for zone-cost based routing
- No need for A* on small graphs
- Peer review friendly

### Why Greedy Simulation?
- Simple to implement and debug
- Capacity constraints naturally create queuing
- Avoids complex coordination logic
- Meets turn count requirements

### Why Two-Phase Updates?
- Prevents race conditions
- Ensures fair capacity allocation
- Easy to reason about
- Industry standard for simultaneous movement

## 🤝 Contributing

This is a 42 school project built for learning purposes. The design emphasizes:
- Simplicity over optimization
- Clarity over abstraction
- Testability over features
- Explainability in peer reviews

## 📝 License

Educational project for 42 school.

## 👤 Author

Built following 42 school project guidelines.
