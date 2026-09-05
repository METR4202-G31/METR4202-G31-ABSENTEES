# Simple Frontier Explorer

A basic ROS 2 frontier-exploration demonstration using SLAM Toolbox and Nav2.

The package:

* Reads the occupancy grid produced by SLAM.
* Detects frontier cells between known free space and unknown space.
* Groups connected frontier cells.
* Selects a frontier goal.
* Sends the goal to Nav2 for navigation.
* Repeats as the map expands.

## Build

Place the package inside the `src` directory of a ROS 2 workspace, then run:

```bash id="c5i5m6"
colcon build --packages-select simple_explorer
source install/setup.bash
```

Dependencies can be installed with:

```bash id="64bh1p"
rosdep install --from-paths src --ignore-src -r -y
```

## Run

Launch the demonstration with:

```bash id="4441vx"
ros2 launch simple_explorer exploration_demo.launch.py
```

A specific Gazebo world can be supplied with:

```bash id="l4mvnq"
ros2 launch simple_explorer exploration_demo.launch.py \
    world:=/path/to/world.world
```

The robot start position can also be changed:

```bash id="hqzlcf"
ros2 launch simple_explorer exploration_demo.launch.py \
    x_pose:=0.0 \
    y_pose:=0.0
```

## Exploration Flow

```text id="6w4qh7"
LiDAR
  ↓
SLAM
  ↓
Occupancy Map
  ↓
Frontier Detection
  ↓
Frontier Selection
  ↓
Nav2 Goal
  ↓
Robot Navigation
  ↓
Updated Map
  ↓
Repeat
```

## Main ROS 2 Interfaces

* `/scan` — LiDAR data
* `/odom` — robot odometry
* `/map` — occupancy grid
* `/tf` and `/tf_static` — coordinate transforms
* `/navigate_to_pose` — Nav2 navigation action
* `/cmd_vel` — robot velocity commands

## Known Limitations

This is a simple proof-of-concept rather than a complete exploration system.

Current limitations include:

* Basic frontier-selection heuristic.
* No explicit obstacle-clearance scoring.
* No pre-check of candidate goals using Nav2 path planning.
* Simple handling of previous and failed goals.
* Approximate exploration-completion detection.
* No optimisation of exploration time or travelled distance.
