# ROS Robotics Projects

Two university projects exploring robot control, sensing and multi-robot mapping in ROS and Gazebo, developed in 2024. The first focuses on estimating and controlling one robot's position; the second combines two robots' observations to build a shared map and plan routes to targets.

Each project has its own workspace below. Shared setup files remain in both folders so the assignments can be read independently.

| Project | Focus | Implementation | Contribution |
|---|---|---|---|
| [Robot control and sensor fusion](projects/robot-control-sensor-fusion) | One robot, sonar, position estimation and proportional control | C++, Eigen, ROS | Individual coursework by Campbell Reid |
| [Multi-robot mapping and planning](projects/multi-robot-mapping-planning) | Two robots, laser scans, shared mapping, target detection and route generation | Python, C++, Open3D, NetworkX, ROS | Group project by Campbell Reid and Kenneth Foley |

## 1. Robot control and sensor fusion

**April–May 2024 · C++, ROS, Gazebo, Eigen**

A C++ controller combines sonar-derived measurements with position estimates through a two-dimensional Kalman filter and publishes bounded velocity commands. Three task modes compare sonar-only input with simulator position and a supplied noisy-position service.

The small dark robot below approaches the blue cylindrical target using the noisy-position mode. A centimetre/metre mismatch affects the estimate, so this shows execution rather than successful target arrival. The unchanged controller runs at normal speed in ROS Noetic/Gazebo 11; the recording ends after 60 seconds.

https://github.com/user-attachments/assets/7ed49962-6002-4632-8c4a-ef5697441089

<details>
<summary>Implementation, task modes and limitations</summary>

- ROS sonar subscription and service-based position queries.
- Eigen state vectors and covariance matrices for a two-dimensional Kalman filter.
- Nominal 5 Hz proportional-control loop with velocity bounded to ±0.15 m/s.
- Mode 1 uses sonar without the Kalman update; mode 2 adds Gazebo position; mode 3 uses the supplied noisy-position service.
- Stopping logic is incomplete, and mode 3 mixes centimetres/cm² with metres/m². No localisation-accuracy or hardware-deployment claim is made.

[Source, build context and detailed limitations](projects/robot-control-sensor-fusion/README.md)

</details>

## 2. Multi-robot mapping and planning

**May–June 2024 · Python, C++, ROS, Gazebo, Open3D, NetworkX**

The later group project moves from controlling one robot to combining two robots' observations. Both explore an arena, save laser point clouds and contribute to a shared occupancy map. A planning service identifies target patterns, allocates goals and returns graph routes.

### Exploring and mapping the arena · 4× speed

The two small dark objects are TurtleBot3 robots. **Red blocks are obstacles, green blocks are targets to visit, and the dark outer wall is the arena boundary.** Both robots drive around, turning when obstacles are detected and collecting laser scans; neither is a leader. Their point clouds are then merged into a shared map to identify targets and generate routes. This recording uses the original Test 3 arena.

https://github.com/user-attachments/assets/a39608b1-a1ee-4fdc-a4de-cc5f5264d941

### Goal allocation and route traversal · 4× speed

**The small dark objects are robots, green blocks are targets to visit, and red blocks are obstacles to plan around.** Each robot requests a target from the shared service and receives a route from its current position. It steps through the route's waypoints, then requests another remaining target. Neither robot has a permanently assigned region. Green blocks remain after a visit because object collection was not implemented.

The visible jumps use Gazebo model-state updates rather than continuous driving. This demonstrates goal allocation and graph-based route planning, not validated physical path following or collision-free execution.

https://github.com/user-attachments/assets/fae7b337-806e-46ed-a0ab-662c81c32b4b

<details>
<summary>Mapping and planning pipeline, execution limits and source</summary>

1. Two robots collect laser scans during obstacle-reactive exploration.
2. Scan points are transformed into world coordinates and saved as per-robot point clouds.
3. Open3D loads the point clouds; a 0.25 m occupancy grid combines observations.
4. A fixed geometric pattern identifies target-shaped objects.
5. A ROS service allocates remaining targets and uses NetworkX to generate grid routes.
6. The historical path controller steps through waypoints with Gazebo `SetModelState`.

This is geometric detection, not learned recognition, and mapping does not implement full SLAM. Unknown cells are treated as free, robot-footprint inflation is absent, and goal allocation is not globally optimised. The graph search uses the library's default zero heuristic. Reliable closed-loop navigation and physical collection were not completed.

[Source, dependencies, original launch sequence and detailed limitations](projects/multi-robot-mapping-planning/README.md)

</details>

## Workspace layout

```text
projects/
  robot-control-sensor-fusion/    # Independent controller workspace
    src/assignment1_setup/       # Supplied setup, messages and services
    src/assignment2/             # C++ controller
  multi-robot-mapping-planning/  # Independent group-project workspace
    src/assignment1_setup/       # Its original supplied setup copy
    src/assignment3/             # Mapping, planning, models and arena layouts
```

Both projects use ROS 1 Noetic and Gazebo Classic 11. Builds and real simulator runs were verified in isolated Ubuntu 20.04 containers. They are separate catkin workspaces: build one project directory at a time, not the repository root. Historical path and environment assumptions are documented in each project README.

## Attribution and licensing

Robot-control coursework by Campbell Reid. Multi-robot mapping/planning co-developed by Campbell Reid and Kenneth Foley. Supplied setup code retains its author and licence notices; TurtleBot3 is a third-party dependency. Package licence metadata remains incomplete, and supplied setup headers and licence text differ. No blanket repository licence is asserted.