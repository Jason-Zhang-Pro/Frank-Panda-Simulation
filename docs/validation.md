# Validation record

This separates offline regression checks from observed Gazebo experiments. Results are development observations, not a benchmark suite or a success-rate estimate.

## Publication status

This initial publication was pushed before completing a fresh validation of the publication checkout, at the owner’s request. Historical observations below must not be read as a fresh test result for this commit.

## Offline regression suite

Build this checkout, source its installation, then run:

```bash
source /opt/ros/humble/setup.bash
colcon build --base-paths src --symlink-install
source install/setup.bash
/usr/bin/python3 -m pytest src/panda_planning/test -m 'not linter' -q
```

The functional suite covers RRT-Connect connectivity and failures, shortcut endpoints/length/collision validity, return-home logic, scene geometry consistency, grasp geometry, and the model-to-path-to-trajectory/controller pipeline. Lint scaffolds are excluded; this is not a claim of passing all colcon/lint tests. These tests do not launch a physical Gazebo pick/place task.

## Physical and planning observations

| Scene / experiment | Observed result | Scope |
|---|---|---|
| Simple, one carrying query, original RRT | No path within 15 s | One query; not a general failure rate |
| Simple, corresponding RRT-Connect query, seed 3 | 33 points in approximately 0.8 s | Single-query comparison |
| Simple, full RRT-Connect task, seed 3 | Pick, place and home completed; max final error ≈0.0005 rad | Physical simulation observation |
| Simple, shortcut and stronger fingers, seed 3 | Full task completed; max home error ≈0.0003 rad | Transfer 32→3 points; command cap 5 N/finger |
| Complex, offline carried-object search | Approximately 58→5 points; 1062 dense states valid | Does not prove execution success |
| Complex, first physical run | Placement and retreat completed; return trajectory started | Harness stopped before return completion |
| Complex, extended physical run | Placement and retreat completed; home search timed out | `pick_place_home_failed`; full cycle not passed |

Recorded development environment: Ubuntu 22.04.5, ROS 2 Humble, Gazebo Fortress (ignition-gazebo 6), system Python 3, PyBullet 3.2.7, NumPy 2.2.6, SciPy 1.15.3, pytest 6.2.5. Timings depend on hardware, scene, initial state and sampling; no portable timing guarantee is intended.

## Repeat a physical trial

Start a fresh scene using `./start.sh`; source this checkout's installation in the command terminal. Set `/panda_planner` parameter `rrt_seed` to `3`, select `rrt` or `rrt_connect`, and set `shortcut_enabled` as needed before calling `/pick_and_place`. Observe `/planner_status` through final completion or failure. Record scene version, actual starting joints and cube pose, seed, budget, search time, path length, execution outcome and recovery state.

The initial simpler scene is included under `examples/scenes/simple`. Copy **both** its world files into `src/panda_planning/worlds` and restart before comparing old-scene observations. Do not change only the Gazebo scene while leaving the collision scene unchanged.

A Trigger response acknowledges acceptance. Task completion additionally requires actual joint convergence, physical placement verification and return home. On a carrying failure, the current implementation retains its payload state; there is no recovery service yet.

## Limits that affect interpretation

- Collision checks sample states/edges. Conservative convex meshes may reject feasible motion; sampling may miss narrow collisions.
- Static scene changes made through the Gazebo GUI are not synchronized. The target cube is synchronized separately.
- Each quintic segment has zero endpoint velocity. Shortcut reduces waypoints but does not blend velocities.
- Reported finger force is a command limit, not a sensor measurement.
- No repeated-seed statistical study, hardware test, robustness proof or safety certification has been performed.
