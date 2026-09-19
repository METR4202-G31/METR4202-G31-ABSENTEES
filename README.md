# METR4202-G31-ABSENTEES
Team project for METR4202 semester 2 2026.

# Usage
todo: link to metr4202 ros/gazebo install guide, then provide specific guide for this repo. basic idea will be: the repo is a ros workspace. clone repo, cd inside, run `colcon build --symlink-install`, source the setup script.

# Packages
The project is split up into several ROS2 packages in the `src/` directory.

## test_worlds
This package is responsible for randomly generating gazebo worlds and launching them.

Example:
```bash
TURTLEBOT3_MODEL=waffle_pi ros2 launch test_worlds explore.launch.py rows:=50 cols:=50 \
features:="room:3,dead_end:2,narrow_passage:4,large_empty_space:1,l_corridor:3,wall_section:2,plus:2" min_feature_gap:=2 seed:=97
```

## simple_explorer
Frontier exploration. The `frontier_explorer` node finds frontiers in the SLAM map, checks candidate goals against the Nav2 costmaps, the robot's position and a blacklist, and sends each goal to the navigation supervisor's `ManagedNavigate` action. Every goal is also published on `/explore` as a plain `PoseStamped`. Mission control can stop or resume it with `/explore/resume` (`Bool`) and reads `/explore/status`. See `project/simple_explorer/README.md` for the interfaces.

Example:
```bash
ros2 launch simple_explorer explore.launch.py
```

Example (also loads world file and spawns turtlebot):
```bash
TURTLEBOT3_MODEL=waffle_pi ros2 launch simple_explorer exploration_demo.launch.py [world:=/path/to/world.world]
```
## navigation_supervisor
This package will handle a given pose from the frontier detection algorithm to investigate. Locations found to be inaccessible will be added to a list to avoid for future poses.