# METR4202-G31-ABSENTEES

Team project for METR4202 semester 2 2026.

The project implements autonomous exploration, navigation and ArUco target
detection/localisation using ROS 2, Nav2, SLAM Toolbox and TurtleBot3.

## Prerequisites

The project assumes that ROS 2 Humble is already installed.

This project also requires Gazebo, Nav2, SLAM Toolbox and the TurtleBot3 ROS packages.

## Clone and Build

Create a workspace if required:

```bash
mkdir -p ~/metr4202_ws/src
cd ~/metr4202_ws/src
```

Clone the repository:

```bash
git clone https://github.com/METR4202-G31/METR4202-G31-ABSENTEES.git
```

Alternatively if SSH access is configured:

```bash
git clone git@github.com:METR4202-G31/METR4202-G31-ABSENTEES.git
```

Build from the workspace root:

```bash
cd ~/metr4202_ws

source /opt/ros/humble/setup.bash

colcon build --symlink-install

source install/setup.bash
```

The ROS installation and workspace setup scripts must be sourced in each new
terminal before running the project:

```bash
source /opt/ros/humble/setup.bash
source ~/metr4202_ws/install/setup.bash
```

# Running the Integrated Demo

The following sequence reproduces the current demonstration.

The demonstration uses a generated Gazebo environment, autonomous frontier
exploration, Nav2 navigation, ArUco detection and global marker localisation.

## Terminal 1 - Gazebo Test World

```bash
source /opt/ros/humble/setup.bash
source ~/metr4202_ws/install/setup.bash

export TURTLEBOT3_MODEL=waffle_pi

ros2 launch test_worlds explore.launch.py \
    rows:=20 \
    cols:=20 \
    cell_size:=0.75 \
    features:="" \
    seed:=42
```

This generates a deterministic Gazebo world, places the TurtleBot3 and ArUco
marker models, and starts the simulation.

Wait for the robot to spawn before starting the remaining nodes.

## Terminal 2 - Navigation Supervisor

```bash
source /opt/ros/humble/setup.bash
source ~/metr4202_ws/install/setup.bash

ros2 run navigation_supervisor navigation_supervisor \
    --ros-args -p use_sim_time:=true
```

This starts the `ManagedNavigate` action server so that it is available
before the Frontier Explorer is launched.

## Terminal 3 - ArUco Detector

```bash
source /opt/ros/humble/setup.bash
source ~/metr4202_ws/install/setup.bash

ros2 run aruco_detector aruco_detector_node \
    --ros-args -p use_sim_time:=true
```

The detector processes the TurtleBot3 camera feed and publishes detections on:

```text
/aruco
```

## Terminal 4 - Mission Control

```bash
source /opt/ros/humble/setup.bash
source ~/metr4202_ws/install/setup.bash

ros2 run mission_control mission_control_node \
    --ros-args -p use_sim_time:=true
```

Mission Control transforms detected ArUco marker positions from the camera frame
into the global `map` frame and stores the resulting marker positions.


## Terminal 5 - SLAM, Nav2 and Frontier Explorer

```bash
source /opt/ros/humble/setup.bash
source ~/metr4202_ws/install/setup.bash

ros2 launch simple_explorer explore.launch.py use_sim_time:=true
```

This starts SLAM Toolbox, Nav2, RViz and the Frontier Explorer.

Once SLAM, Nav2, the required costmaps and transforms are available, the
Frontier Explorer begins selecting and sending navigation goals.

# Expected Demo Behaviour

When all nodes are running:

1. SLAM Toolbox builds a map from the robot's LiDAR data.
2. Frontier Explorer detects boundaries between explored and unexplored space.
3. A frontier goal is selected and sent to the Navigation Supervisor using the
   `ManagedNavigate` action.
4. The Navigation Supervisor validates the goal and passes it to Nav2.
5. Nav2 plans and controls the robot toward the selected frontier.
6. Frontier selection repeats as new areas of the map are revealed.
7. Final Nav2 navigation failures are recorded on `/failed_regions`.
8. The ArUco Detector processes the robot camera feed during exploration.
9. Detected markers are published on `/aruco`.
10. Mission Control transforms detections into the global `map` frame.

Exploration continues until no suitable frontier goals remain.

Useful runtime checks include:

```bash
ros2 topic echo /explore/status
```

```bash
ros2 action info /managed_navigate
```

```bash
ros2 topic echo /aruco
```

Failed navigation regions can be inspected with:

```bash
ros2 topic echo /failed_regions \
    navigation_interfaces/msg/FailedRegionArray \
    --qos-reliability reliable \
    --qos-durability transient_local \
    --qos-history keep_last \
    --qos-depth 1
```

Mission Control publishes detected marker positions on:

```text
/marker
```

which can be displayed in RViz using a `Marker` display.

# Packages

The project is split into several ROS 2 packages in the `project/` directory.

## test_worlds

This package is responsible for randomly generating Gazebo worlds and launching them.

Example:

```bash
TURTLEBOT3_MODEL=waffle_pi ros2 launch test_worlds explore.launch.py rows:=50 cols:=50 \
features:="room:3,dead_end:2,narrow_passage:4,large_empty_space:1,l_corridor:3,wall_section:2,plus:2" min_feature_gap:=2 seed:=97
```

World generation can be made deterministic by specifying a fixed `seed`.

The generated environments can include rooms, corridors, dead ends, narrow
passages, walls, open spaces and ArUco marker models.

## simple_explorer

Frontier exploration. The `frontier_explorer` node finds frontiers in the SLAM map,
checks candidate goals against the Nav2 costmaps, the robot's position and a
blacklist, and sends each goal to the Navigation Supervisor's `ManagedNavigate`
action.

Every goal is also published on `/explore` as a plain `PoseStamped`.

Exploration can be stopped or resumed using `/explore/resume` (`Bool`), and the
current exploration state is published on `/explore/status`.

See `project/simple_explorer/README.md` for the interfaces.

Example:

```bash
ros2 launch simple_explorer explore.launch.py
```

Example which also loads a world file and spawns the TurtleBot3:

```bash
TURTLEBOT3_MODEL=waffle_pi ros2 launch simple_explorer exploration_demo.launch.py [world:=/path/to/world.world]
```

`exploration_demo.launch.py` should not be used at the same time as
`test_worlds explore.launch.py`, as both launch a Gazebo environment.

## navigation_supervisor

The Navigation Supervisor provides a single managed navigation interface between
higher level behaviours and Nav2.

It accepts navigation requests through the custom `ManagedNavigate` action,
validates the supplied goal, executes it using Nav2 Simple Commander and reports
the final result to the requesting module.

Final Nav2 navigation failures are recorded as failed regions and published on:

```text
/failed_regions
```

Failed regions are navigation history rather than a mandatory blacklist. The
Frontier Explorer may use this information when selecting future goals.

See `project/navigation_supervisor/README.md` for further details.

## navigation_interfaces

Shared ROS 2 interfaces used by the exploration and navigation packages.

This package defines:

```text
ManagedNavigate.action
FailedRegion.msg
FailedRegionArray.msg
```

See `project/navigation_interfaces/README.md` for the interface definitions.

## aruco_detector

Detects ArUco markers in the TurtleBot3 camera image and estimates marker pose
relative to the camera.

The detector subscribes to:

```text
/camera/image_raw
/camera/camera_info
```

and publishes detected markers on:

```text
/aruco
```

## aruco_detector_interfaces

Contains the ROS 2 message interfaces used by the ArUco detector.

The primary interface is:

```text
ArucoVerdict
```

which contains the detection state, marker ID and estimated marker pose.

## mission_control

Mission Control receives ArUco detections and transforms their estimated poses
into the global `map` frame.

Detected marker locations are stored and periodically reported.

Marker positions are also published as RViz visualisation markers on:

```text
/marker
```

The current implementation provides marker localisation and storage.
Stage 2 autonomous target search behaviour is planned for later development.

# Current Scope

The current integrated system demonstrates:

- autonomous frontier exploration;
- SLAM map generation;
- Nav2 path planning and navigation;
- managed navigation through the Navigation Supervisor;
- failed-navigation region reporting;
- ArUco marker detection;
- global marker localisation;
- integrated operation of the exploration, navigation and perception modules.

The Navigation Supervisor currently reports final Nav2 failures as
`NAVIGATION_FAILED`. More detailed classifications such as `PLANNING_FAILED` and
`STUCK` are reserved for later development.

Stage 2 autonomous target-search behaviour is not yet implemented.
