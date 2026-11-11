# Multi-Robot Mapping and Planning

University group project by Campbell Reid and Kenneth Foley, May–June 2024.

Two simulated TurtleBot3 robots explore an arena, merge laser point clouds into an occupancy map, identify target patterns and request routes from a shared planning service. See the [main README](../../README.md#2-multi-robot-mapping-and-planning) for the exploration and traversal videos.

## Source map

| File | Purpose |
|---|---|
| `src/assignment3/scripts/exploration_node.py` | Scan collection, world-coordinate conversion, exploration and PCD output |
| `src/assignment3/scripts/discretise.py` | Point-cloud merging, occupancy grid, geometric target detection and graph paths |
| `src/assignment3/scripts/global_map_server.py` | Shared map processing, goal allocation and path service |
| `src/assignment3/scripts/path_controller.py` | Path requests and Gazebo model-state waypoint traversal |
| `src/assignment3/scripts/populate_world.cpp` | Environment population |
| `src/assignment3/Test_Worlds` | Original primitive models and Test 1–3 layouts |
| `src/assignment3/launch` | World, environment and robot launch files |
| `src/assignment1_setup` | Original supplied setup copy, retained independently |

## Pipeline and observed behavior

Both robots explore using velocity commands and front-obstacle avoidance. They save laser point clouds, then trigger a shared 0.25 m occupancy grid. A fixed pattern detector identifies target-shaped objects. Robots request goals from a ROS service, which uses a NetworkX grid graph to generate routes.

The original Test 3 arena was built and run in ROS Noetic/Gazebo 11. Both robots collected scans, all three target patterns were detected, and three routes were generated and stepped through. Exploration uses continuous motion; subsequent route traversal uses `SetModelState` teleportation. No physical collection is implemented.

## Environment and build

Verified runtime: Ubuntu 20.04, ROS Noetic, Gazebo 11, Python 3.8, GCC 9.4, catkin, official TurtleBot3 Burger simulation/description packages, Eigen and TF. Python runtime packages included Open3D CPU 0.18.0, NumPy 1.24.4 and NetworkX 2.8.8. The original `environment.yml` and Conda activation scripts are retained as environment references; the verified container used a Python venv with access to ROS system packages instead.

The package built with `catkin_make -j2`. Before reproducing it in another location:

- Set up a proper catkin workspace (`src/CMakeLists.txt` may be a plain-text symlink target on Windows).
- Update original absolute workspace/PCD paths in `exploration_node.py`, `discretise.py` and `env.launch` to the chosen workspace.
- The original `world.launch` names `turtlebot3_empty_world.world`; the installed Noetic package used `empty.world`.
- Ensure the Python nodes are executable with Python 3 shebangs, and their scripts directory is importable for sibling imports. These portability adjustments were made only in the isolated runtime copy.
- Install the official TurtleBot3 dependencies separately; bundled third-party simulation source is not included here.

The original launch sequence is `roscore`, `roslaunch assignment3 world.launch`, `roslaunch assignment3 env.launch`, then `roslaunch assignment3 bots.launch`, each in a sourced environment. Wait for Gazebo and environment spawning before starting the robots. These commands require the path and environment preparation above; this is not a claim that the unmodified checkout launches directly on a new machine. No automated test cases are defined.

## Known limitations

- Waypoint traversal teleports models; reliable closed-loop route following was not completed.
- Mapping uses odometry and scan trigonometry, not full SLAM. Infinite ranges are not filtered before coordinate conversion.
- Unknown cells are treated as free and robot-footprint inflation is absent. Collision-free navigation is not established.
- Target detection uses an exact geometric kernel, not machine learning.
- Goal selection is random/request-driven, removes goals before acknowledged completion and is not globally optimised.
- `networkx.astar_path` is used without a custom heuristic. No original optimised A* implementation is claimed.
- Package dependency/licence metadata and historical absolute paths remain incomplete. Only Test 3 was freshly exercised.

## Attribution and licensing

Co-developed by Campbell Reid and Kenneth Foley. Supplied setup retains its original notices and licence text; TurtleBot3 is a third-party dependency. The assignment3 licence field is TODO; no blanket licence grant is inferred.