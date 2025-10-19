# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is an ECE276B course project implementing 3D motion planning algorithms. The project compares search-based (A*) and sampling-based motion planning approaches in various 3D environments with obstacles.

## Project Structure

The project consists of three main parts:

### Part 1: Collision Detection Implementation
- Implement collision detection between path segments (lines) and obstacles (axis-aligned bounding boxes/AABB)
- This is a prerequisite for path planning algorithms
- Currently missing proper implementation in main.py (line 104 TODO) and Planner.py

### Part 2: Search-based Motion Planning
- Implement search-based algorithms (A*, Dijkstra, etc.)
- Complete the skeleton code in astar.py
- Uses the collision detection from Part 1

### Part 3: Sampling-based Motion Planning
- Implement sampling-based algorithms (RRT, RRT*, PRM)
- Can either implement from scratch or use OMPL library
- Requires collision detection from Part 1

## Key Commands

### Running Tests
```bash
# Run all test environments
python3 starter_code/main.py

# Run individual test environments (modify main.py to call specific functions)
# Available test functions: test_single_cube(), test_maze(), test_window(),
# test_tower(), test_flappy_bird(), test_room(), test_pillars()
```

### Dependencies
```bash
# Core dependencies required:
pip3 install numpy matplotlib

# For A* implementation (currently not installed):
pip3 install pqdict
```

## Architecture

### Core Components

1. **main.py**: Test harness and visualization
   - `runtest()`: Main test runner that loads maps, calls planner, validates paths
   - `test_*()` functions: Pre-configured test scenarios with start/goal positions
   - `draw_map()`, `draw_block_list()`: 3D visualization using matplotlib
   - **TODO**: Line 104 needs collision checking implementation

2. **Planner.py**: Motion planner interface
   - `MyPlanner` class: Base planner implementation (currently a simple greedy approach)
   - `plan(start, goal)`: Returns numpy array of waypoints
   - Current implementation explores 26 directions, picks closest to goal
   - **Limitation**: Gets stuck in complex environments, inadequate collision checking

3. **astar.py**: A* algorithm skeleton
   - `AStarNode`: Node representation with g, h values and parent tracking
   - `AStar.plan()`: Incomplete - needs full A* implementation
   - Uses pqdict for priority queue (requires installation)

### Map Format

Maps in `starter_code/maps/` use text format:
- First line: `boundary xmin ymin zmin xmax ymax zmax r g b`
- Subsequent lines: `block xmin ymin zmin xmax ymax zmax r g b`
- RGB values are 0-255 for visualization

### Test Environments

Seven environments with increasing complexity:
- `single_cube.txt`: Simple obstacle avoidance
- `maze.txt`: Complex maze navigation
- `window.txt`: Narrow passage problem
- `tower.txt`: Vertical navigation challenge
- `flappy_bird.txt`: Sequential obstacles
- `room.txt`: Indoor navigation with furniture
- `pillars.txt`: Forest of vertical obstacles

## Implementation Notes

### Path Validation Requirements
- Paths must not intersect boundary walls or obstacle blocks in continuous space
- Need segment-AABB (Axis-Aligned Bounding Box) intersection checking
- Goal is reached when distance < 0.1 units
- Path length computed as sum of Euclidean distances between waypoints

### Coordinate System
- 3D Cartesian coordinates [x, y, z]
- Boundary defines valid workspace
- Blocks are axis-aligned rectangular obstacles
- All coordinates in floating-point units

### Expected Improvements
1. Implement proper collision checking for path segments
2. Complete A* algorithm implementation
3. Add sampling-based planners (RRT, RRT*, PRM)
4. Improve baseline planner to avoid getting stuck