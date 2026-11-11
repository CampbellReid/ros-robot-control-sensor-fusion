# Robot Control and Sensor Fusion

Individual university coursework by Campbell Reid, April–May 2024.

A C++ ROS controller combining sonar-derived measurements with a two-dimensional Kalman-filter implementation and bounded proportional velocity control. See the [main README](../../README.md#1-robot-control-and-sensor-fusion) for its video.

## Implementation

- `src/assignment2/src/controller.cpp`: controller, filter and task modes.
- `src/assignment1_setup`: supplied sonar, noisy position service, messages and service definitions; original notices retained.
- Mode 1: sonar-derived scalar without the Kalman update.
- Mode 2: Gazebo model position in the Kalman update.
- Mode 3: supplied noisy-position service in the Kalman update.
- Eigen state/covariance matrices, nominal 5 Hz loop and proportional gains `(0.1, 0, 0)`, with velocity bounded to ±0.15 m/s.

## Build and simulation context

The unchanged packages built and all three modes ran in Ubuntu 20.04 with ROS Noetic, Gazebo 11, Eigen, TF, geometry/gazebo messages and catkin. The validation workspace contained these two packages and a proper catkin top-level symlink; the checkout's `src/CMakeLists.txt` may be a plain-text symlink target on Windows. From an appropriately configured workspace, the verified build command was `catkin_make -j2` after sourcing `/opt/ros/noetic/setup.bash`.

The original bowl world/model was not recovered. Validation used official TurtleBot3 packages and a replacement cylindrical target named `bowl`, plus the supplied sonar and position nodes. The controller resets the robot pose and queries target position from Gazebo; it requires the matching model names, velocity topic/subscriber and running setup services. `rosrun assignment2 controller 1`, `2` or `3` starts a mode only after those simulator prerequisites are in place. This checkout does not contain a complete launchable replacement scene.

No automated tests are defined. Runtime observations do not establish filter accuracy or reliable target arrival.

## Known limitations

- `argv[1]` is read without argument validation.
- The target-reached block is commented out; the negative-estimate stop cannot trigger for a nonnegative position norm.
- The target-offset comment and calculation disagree.
- Mode 3 mixes position-service centimetres/cm² with sonar metres/m².
- Package dependency declarations are incomplete. The simulator supplies state; physical deployment has not been evaluated.

## Attribution and licensing

Controller by Campbell Reid. Supplied setup retains its original headers and licence file, which contain differing MIT/GPL notices. The controller package has a TODO licence field. The TurtleBot3 simulation submodule remains an external dependency.